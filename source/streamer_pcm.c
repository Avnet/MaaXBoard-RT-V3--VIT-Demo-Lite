/*
 * Copyright 2018-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

 // Modified 28/02/2022 - Seb Sikora (ssikora)

#include "osa_common.h"

#include "main.h"
#include "board.h"
#include "streamer_pcm.h"
#include "fsl_codec_common.h"
#include "fsl_sgtl5000.h"
#include "app_definitions.h"
#include "osa_memory.h"
#include "fsl_cache.h"
#include "fsl_debug_console.h"
#define MA_NO_DEVICE_IO
#define MA_NO_DECODING						// Define this to leave-out decoding/encoding functionality
#define MINIAUDIO_IMPLEMENTATION			// Include the library
#include "miniaudio.h"			// " "
#include "dynamic_compressor.h"

#define EN_DITHER	1
#define FRAME_SIZE_SAMPLES	  	(160)
#define BUFFER_SIZE   		  	(FRAME_SIZE_SAMPLES*2*2*NUM_OF_CH)
#define BUFFER_NUMBER 			(2)
AT_NONCACHEABLE_SECTION_INIT(static pcm_rtos_t pcmHandle) = {0};
AT_NONCACHEABLE_SECTION_ALIGN(edma_tcd_t s_edmaTcd_0[2], 32U);
/* current buffer index for PDM (it is either 0 or 1) */
static uint32_t buffer_index = 0;

extern uint8_t pcmBuffer[PCM_SIZE];
extern uint8_t pcmBufferTest[PCM_SIZE];
static uint32_t pcmRecordIndex = 0;
static uint32_t pcmRecordIndexTest = 0;
static uint32_t recordEnable = 0;

static uint8_t deinterleavedInputBuffer[FRAME_SIZE_SAMPLES * 3 * NUM_OF_CH];	// 3 bytes per sample

static float compressor_buffer[FRAME_SIZE_SAMPLES];


/* create twice the size of VIT sample chunk. 160(vit)*2 */
AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t s_buffer[BUFFER_SIZE * BUFFER_NUMBER], 4);
pdm_edma_transfer_t pdmXfer[2] = {
    {
        .data         = s_buffer,
        .dataSize     = BUFFER_SIZE,
        .linkTransfer = &pdmXfer[1],
    },

    {.data = &s_buffer[BUFFER_SIZE], .dataSize = BUFFER_SIZE, .linkTransfer = &pdmXfer[0]},
};
static const pdm_config_t pdmConfig         = {
    .enableDoze        = false,
    .fifoWatermark     = DEMO_PDM_FIFO_WATERMARK,
    .qualityMode       = DEMO_PDM_QUALITY_MODE,
    .cicOverSampleRate = DEMO_PDM_CIC_OVERSAMPLE_RATE,
};
static const pdm_channel_config_t channelConfig = {
    .cutOffFreq = kPDM_DcRemoverCutOff152Hz,
    .gain       = kPDM_DfOutputGain7,
};

extern codec_handle_t codecHandle;

static TaskHandle_t *player_task_handle=NULL;
/*
 * Function prototypes
 * */

static void deinterleave_input_data(uint8_t *dataInput, uint8_t *dataDeinterleaved, uint16_t FrameSize, uint8_t NumberOfChannels);
static void reduce_width_and_output(uint8_t *dataDeinterleaved, uint8_t *dataOutput, uint16_t FrameSize, uint8_t NumberOfChannels);
static void DeInterleave(uint8_t *dataInput, uint8_t *dataOutput, uint16_t FrameSize, uint8_t ChannelNumber);
//static void copy2Byte(uint8_t *dataInput, uint8_t *dataOutput, uint16_t FrameSize);


void set_player_task_handle(void *t_handle)
{
	player_task_handle = (TaskHandle_t *)t_handle;
}

/*! @brief SAI EDMA transmit callback
 *
 * This function is called by the EDMA interface after a block of data has been
 * successfully written to the SAI.
 */
static void saiTxCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData)
{
    pcm_rtos_t *pcm = (pcm_rtos_t *)userData;
    BaseType_t reschedule;
    xSemaphoreGiveFromISR(pcm->semaphoreTX, &reschedule);
    portYIELD_FROM_ISR(reschedule);
}

static void pdmCallback(PDM_Type *base, pdm_edma_handle_t *handle, status_t status, void *userData)
{
    pcm_rtos_t *pcm = (pcm_rtos_t *)userData;
	BaseType_t reschedule;
	/* if value is 0, 0th buffer has the PDM data, 1st buffer is being copied by DMA */
	buffer_index ^= 1;
	xSemaphoreGiveFromISR(pcm->semaphoreRX, &reschedule);
	portYIELD_FROM_ISR(reschedule);
}

static void saiRxCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData)
{
    pcm_rtos_t *pcm = (pcm_rtos_t *)userData;
    BaseType_t reschedule;
    xSemaphoreGiveFromISR(pcm->semaphoreRX, &reschedule);
    portYIELD_FROM_ISR(reschedule);
}

void streamer_pcm_init(void)
{
	NVIC_SetPriority(DEMO_PDM_IRQ, 5U);
	NVIC_SetPriority(DMA1_DMA17_IRQn, 4U);
    /* Create DMA handle. */
    EDMA_CreateHandle(&pcmHandle.dmaRxHandle, DEMO_DMA, DEMO_PDM_EDMA_CHANNEL_1);
    /* Setup pdm */
    PDM_Init(DEMO_PDM, &pdmConfig);
    pcmHandle.isFirstRx = 1;
    EnableIRQ(DEMO_PDM_IRQ);

    dynamic_compression_init(5.0, 20.0, 0.005, 0.2, 1.0, 16000);
}

pcm_rtos_t *streamer_pcm_open(uint32_t num_buffers)
{
	/* this task is only responsible for reading PCM data from MIC.*/
    return &pcmHandle;
}

pcm_rtos_t *streamer_pcm_rx_open(uint32_t num_buffers)
{
    pcmHandle.semaphoreRX = xSemaphoreCreateBinary();
    PDM_TransferCreateHandleEDMA(DEMO_PDM, &pcmHandle.pdmRxHandle, pdmCallback, (void *)&pcmHandle, &pcmHandle.dmaRxHandle);
	PDM_TransferInstallEDMATCDMemory(&pcmHandle.pdmRxHandle, s_edmaTcd_0, 2);
	PDM_TransferSetChannelConfigEDMA(DEMO_PDM, &pcmHandle.pdmRxHandle, DEMO_PDM_ENABLE_CHANNEL_MIC0, &channelConfig);
#if (NUM_OF_CH==2)
	PDM_TransferSetChannelConfigEDMA(DEMO_PDM, &pcmHandle.pdmRxHandle, DEMO_PDM_ENABLE_CHANNEL_MIC1, &channelConfig);
#elif (NUM_OF_CH==3)
	PDM_TransferSetChannelConfigEDMA(DEMO_PDM, &pcmHandle.pdmRxHandle, DEMO_PDM_ENABLE_CHANNEL_MIC1, &channelConfig);
	PDM_TransferSetChannelConfigEDMA(DEMO_PDM, &pcmHandle.pdmRxHandle, DEMO_PDM_ENABLE_CHANNEL_MIC2, &channelConfig);
#endif
	if (PDM_SetSampleRateConfig(DEMO_PDM, DEMO_PDM_CLK_FREQ, DEMO_PDM_SAMPLE_RATE) != kStatus_Success)
	{
	   PRINTF("PDM configure sample rate failed.\r\n");
	   return NULL;
	}
	PDM_Reset(DEMO_PDM);
    return &pcmHandle;
}

void streamer_pcm_start(pcm_rtos_t *pcm)
{
    /* Interrupts already enabled - nothing to do.
     * App/streamer can begin writing data to SAI. */
}

void streamer_pcm_close(pcm_rtos_t *pcm)
{
    /* Stop playback.  This will flush the SAI transmit buffers. */
	/* this task is only responsible for reading PCM data from MIC.*/
}

void streamer_pcm_rx_close(pcm_rtos_t *pcm)
{
    /* Stop playback.  This will flush the SAI transmit buffers. */
    PDM_TransferTerminateReceiveEDMA(DEMO_PDM, &pcm->pdmRxHandle);
    vSemaphoreDelete(pcmHandle.semaphoreRX);
}

int streamer_pcm_write(pcm_rtos_t *pcm, uint8_t *data, uint32_t size)
{
	/* this task is only responsible for reading PCM data from MIC.*/
return 0;
}

void streamer_read_stop()
{
	PDM_TransferTerminateReceiveEDMA(DEMO_PDM, &pcmHandle.pdmRxHandle);
	PDM_Reset(DEMO_PDM);
	pcmHandle.pdmRxHandle.receivedBytes = 0;
	buffer_index = 0;
}

void streamer_read_start()
{
	pcmHandle.pdmXfer_pt = pdmXfer;
	PDM_TransferReceiveEDMA(DEMO_PDM, &pcmHandle.pdmRxHandle, pcmHandle.pdmXfer_pt);
	pcmHandle.isFirstRx = 0;
}

void enable_record(bool state)
{
	if (state)
	{
		recordEnable = 1;
		pcmRecordIndex = 0;
		pcmRecordIndexTest = 0;
	}else
	{
		recordEnable = 0;
	}
}

int streamer_pcm_read(pcm_rtos_t *pcm, uint8_t *data, uint8_t *next_buffer, uint32_t size)
{
    /* Start the first transfer */
    if (pcm->isFirstRx)
    {
    	/* pdmXfer uses circular linked list buffer for storing Audio samples continuously */
    	pcm->pdmXfer_pt = pdmXfer;
    	PDM_TransferReceiveEDMA(DEMO_PDM, &pcm->pdmRxHandle, pcm->pdmXfer_pt);
        pcm->isFirstRx = 0;
    }

    /* Wait for transfer to finish */
    if (xSemaphoreTake(pcm->semaphoreRX, 3000) != pdTRUE)
    {
    	PRINTF("PDM transfer never finishes");
        return -1;
    }

    /*
     * de-interleave the multi-channel samples like
     * A1, B1, C1, A2, B2 ... => A1, A2, A3, .. B1, B2,...
     * and then convert from 24 to 16-bit, applying triangle mode dithering using the miniaudio library
     */

#ifdef EN_DITHER
    deinterleave_input_data((uint8_t *)&(pdmXfer[buffer_index ^ 1].data[0]), ( uint8_t *)&deinterleavedInputBuffer, FRAME_SIZE_SAMPLES, NUM_OF_CH);
    reduce_width_and_output(( uint8_t *)&deinterleavedInputBuffer, data, FRAME_SIZE_SAMPLES, NUM_OF_CH);
#else
    DeInterleave((uint8_t *)&(pdmXfer[buffer_index ^ 1].data[0]), data, FRAME_SIZE_SAMPLES, NUM_OF_CH);
#endif
    if (recordEnable==1)
    {
    	if (pcmRecordIndex < PCM_SIZE)
    	{
			uint32_t tempSize = 0;
			if (pcmRecordIndex+BUFFER_SIZE < PCM_SIZE)
			{
				tempSize = BUFFER_SIZE;
			} else
			{
				tempSize = PCM_SIZE - pcmRecordIndex;
			}
			memcpy(pcmBuffer+pcmRecordIndex, (uint8_t *)&(pdmXfer[buffer_index ^ 1].data[0]), tempSize);
			pcmRecordIndex += tempSize;

			/* store the dithered, noise compressed samples */
			memcpy(pcmBufferTest+pcmRecordIndexTest, data, size);
			pcmRecordIndexTest += size;

    	}
    	else
    	{
    		recordEnable = 0;
    		xTaskNotify(*player_task_handle, 0, eSetValueWithOverwrite); /*record complete*/
    	}
    }
    return 0;
}

/*! @brief Map an integer sample rate (Hz) to internal SAI enum */
static sai_sample_rate_t _pcm_map_sample_rate(uint32_t sample_rate)
{
    switch (sample_rate)
    {
        case 8000:
            return kSAI_SampleRate8KHz;
        case 11025:
            return kSAI_SampleRate11025Hz;
        case 12000:
            return kSAI_SampleRate12KHz;
        case 16000:
            return kSAI_SampleRate16KHz;
        case 24000:
            return kSAI_SampleRate24KHz;
        case 22050:
            return kSAI_SampleRate22050Hz;
        case 32000:
            return kSAI_SampleRate32KHz;
        case 44100:
            return kSAI_SampleRate44100Hz;
        case 48000:
        default:
            return kSAI_SampleRate48KHz;
    }
}

/*! @brief Map an integer bit width (bits) to internal SAI enum */
static sai_word_width_t _pcm_map_word_width(uint32_t bit_width)
{
    switch (bit_width)
    {
        case 8:
            return kSAI_WordWidth8bits;
        case 16:
            return kSAI_WordWidth16bits;
        case 24:
            return kSAI_WordWidth24bits;
        case 32:
            return kSAI_WordWidth32bits;
        default:
            return kSAI_WordWidth16bits;
    }
}

/*! @brief Map an integer number of channels to internal SAI enum */
static sai_mono_stereo_t _pcm_map_channels(uint8_t num_channels)
{
    if (num_channels >= 2)
        return kSAI_Stereo;
    else
        return kSAI_MonoRight;
}

int streamer_pcm_setparams(
    pcm_rtos_t *pcm, uint32_t sample_rate, uint32_t bit_width, uint8_t num_channels, bool transfer, bool dummy_tx)
{
     sai_transfer_format_t format = {0};
    sai_transceiver_t saiConfig, saiConfig2;
    uint32_t masterClockHz = 0U;

    pcm->sample_rate     = sample_rate;
    pcm->bit_width       = bit_width;
    pcm->num_channels    = num_channels;
    pcm->dummy_tx_enable = dummy_tx;

   masterClockHz = streamer_set_master_clock(sample_rate);

   format.channel       = 0U;
   format.bitWidth      = _pcm_map_word_width(bit_width);
   format.sampleRate_Hz = _pcm_map_sample_rate(sample_rate);
   format.stereo        = _pcm_map_channels(num_channels);
#if (defined FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER && FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER)
   format.masterClockHz = masterClockHz;
#endif

    /* I2S transfer mode configurations */
   if (transfer)
   {
       SAI_GetClassicI2SConfig(&saiConfig, _pcm_map_word_width(bit_width), format.stereo, 1U << DEMO_SAI_CHANNEL);
       /* If there wasn't first RX transfer, we need to set SAI mode to Async */
       if (pcm->isFirstRx)
       {
           saiConfig.syncMode = kSAI_ModeAsync;
       }
       else
       /* Otherwise we need to sync the SAI for the loopback */
       {
           saiConfig.syncMode = kSAI_ModeSync;
       }
       saiConfig.masterSlave = kSAI_Master;
   }
   else
   {
       /* I2S receive mode configurations */
       SAI_GetClassicI2SConfig(&saiConfig, _pcm_map_word_width(bit_width), format.stereo, 1U << DEMO_SAI_CHANNEL);
       if (dummy_tx)
           SAI_GetClassicI2SConfig(&saiConfig2, _pcm_map_word_width(bit_width), format.stereo, 1U << DEMO_SAI_CHANNEL);
       saiConfig.syncMode    = kSAI_ModeAsync;
       saiConfig.masterSlave = kSAI_Master;

       if (dummy_tx)
       {
           saiConfig2.syncMode    = kSAI_ModeSync;
           saiConfig2.masterSlave = kSAI_Master;
       }
   }
    return 0;
}

void streamer_pcm_getparams(pcm_rtos_t *pcm, uint32_t *sample_rate, uint32_t *bit_width, uint8_t *num_channels)
{
    *sample_rate  = pcm->sample_rate;
    *bit_width    = pcm->bit_width;
    *num_channels = pcm->num_channels;
}

int streamer_pcm_mute(pcm_rtos_t *pcm, bool mute)
{
    CODEC_SetMute(&codecHandle, kCODEC_PlayChannelHeadphoneRight | kCODEC_PlayChannelHeadphoneLeft, mute);
    return 0;
}

int streamer_pcm_set_volume(uint32_t volume)
{
    uint32_t adjvol;
    return 0;
}

int streamer_set_master_clock(int sample_rate)
{
    int master_clock;
#if DEMO_CODEC_CS42448
    int divider    = DEMO_SAI1_CLOCK_SOURCE_DIVIDER;
    int predivider = DEMO_SAI1_CLOCK_SOURCE_PRE_DIVIDER;
#endif

#if DEMO_CODEC_CS42448
    switch (sample_rate)
    {
        case 11025:
        case 12000:
        case 24000:
        {
            divider = 63;
            break;
        }
        case 8000:
        {
            predivider = 1;
        }
        case 16000:
        {
            divider = 47;
            break;
        }
        case 32000:
        {
            divider = 23;
            break;
        }
        case 22050:
        case 44100:
        case 48000:
        default:
            divider = 31;
            break;
    }

    CLOCK_SetDiv(kCLOCK_Sai1PreDiv, predivider);
    CLOCK_SetDiv(kCLOCK_Sai1Div, divider);
    master_clock = CLOCK_GetFreq(kCLOCK_AudioPllClk) / (divider + 1U) / (predivider + 1U);
#else
    master_clock = DEMO_SAI_CLK_FREQ;
#endif
    return master_clock;
}

/****************************************************************************************/
/*                                                                                      */
/*                          Functions                                                   */
/*                                                                                      */
/****************************************************************************************/
//  de-Interleave Multichannel signal
//   example:  A1.B1.C1.A2.B2.C2.A3.B3.C3....An.Bn.Cn   (3 Channels case : A, B, C)
//             will become
//             A1.A2.A3....An.B1.B2.B3....Bn.C1.C2.C3....Cn

// Simple helper function for de-interleaving Multichannel stream
// The caller function shall ensure that all arguments are correct.
// In place processing not supported
static void deinterleave_input_data(uint8_t *dataInput, uint8_t *dataDeinterleaved, uint16_t FrameSize, uint8_t NumberOfChannels)
{
	int index_d = 0;
    int index_i;
	for (uint8_t channel_index = 0; channel_index < NumberOfChannels; channel_index ++)
	{
		for (uint16_t sample_index = 0; sample_index < FrameSize; sample_index ++)
		{
			// * 4 as 4 bytes per input data sample, least significant byte is padding so is ignored
			index_i = (NumberOfChannels * sample_index * 4) + (channel_index * 4) + 2;
			dataDeinterleaved[index_d] = dataInput[index_i - 1];
			dataDeinterleaved[index_d + 1] = dataInput[index_i];
			dataDeinterleaved[index_d + 2] = dataInput[index_i + 1];
			index_d += 3;
		}
	}

    return;
}

static void reduce_width_and_output(uint8_t *dataDeinterleaved, uint8_t *dataOutput, uint16_t FrameSize, uint8_t NumberOfChannels)
{
	int offset = 0;
	for (uint8_t channel_index = 0; channel_index < NumberOfChannels; channel_index ++)
	{
		offset = channel_index * (FrameSize * 3);		// 3 bytes per sample
		// Convert from 24-bit to 16-bit samples and apply triangle dithering
		ma_pcm_convert(compressor_buffer, ma_format_f32 , (void*)(dataDeinterleaved + offset), ma_format_s24, FrameSize, ma_dither_mode_none);
		dynamic_compression_apply(((float*)compressor_buffer), ((float*)compressor_buffer), FrameSize);
		ma_pcm_convert((void*)(dataOutput), ma_format_s16, compressor_buffer, ma_format_f32, FrameSize, ma_dither_mode_none);
	}
	return ;
}


static void DeInterleave(uint8_t *dataInput, uint8_t *dataOutput, uint16_t FrameSize, uint8_t ChannelNumber)
{
	int index_o=0;
	int index_i;
	int32_t sample;
	for (uint8_t ichan = 0; ichan < ChannelNumber; ichan++)
	{
		for (uint16_t i = 0; i < FrameSize; i++)
		{
			index_i = ChannelNumber*i*4+ichan*4+2;
			sample = (dataInput[index_i+1]<<16) | (dataInput[index_i]<<8) | (dataInput[index_i-1]);
			sample = sample >> 8;
			dataOutput[index_o] = sample & 0xFF;
			dataOutput[index_o+1] = (sample >> 8) & 0xFF;
			index_o = index_o+2;
		}
	}
    return;
 }
