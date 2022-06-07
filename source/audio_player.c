/*
 * mp3_player.c
 *
 *  Created on: Dec 3, 2021
 *      Author: gulziibayar
 */
#include <audio_player.h>
#include <string.h>
#include <stdint.h>
#include "fsl_debug_console.h"
#include "fsl_shell.h"

#include "app_streamer.h"
#include "fsl_usb_disk.h"

#ifdef VIT_PROC
#include "PL_platformTypes_CortexM7.h"
#include "VIT.h"
#include "vit_proc.h"
#endif

#include "main.h"
#include "app_definitions.h"
#include "fsl_codec_common.h"
#include <sample_mono.h>
#include "beep_mono.h"

#include "fsl_gpio.h"
#include "streamer_pcm.h"

/* Audio Codec Handle */
extern codec_handle_t codecHandle;

/* Function prototypes */
void audio_playback_task(void *param);
static void sendData2Audio(uint8_t * ptr, uint32_t data_size);
static uint32_t configure_audio(uint32_t bit_width, uint32_t sample_rate);
static void play_music(uint8_t select);

/* pointer for task parameter */
static app_handle_t *app_ptr=NULL;

/* Freertos mp3 command queue*/
static QueueHandle_t *player_commandQ;
static queue_command_t audio_recvd_cmd;

AT_NONCACHEABLE_SECTION_INIT(static playback_t playbackHandle) = {0};

static int32_t curr_volume = 80; /* audio volume */
static uint8_t audio_buffer0[4096];	/* DMA buffer for SAI */
static uint8_t audio_buffer1[4096]; /* DMA buffer for SAI */
static int available_buff = 0;

/* array in SDRAM for storing microphone PCM */
__attribute__ ((section(".secSdram"))) uint8_t pcmBuffer[PCM_SIZE] = {[0 ... PCM_SIZE-1] = 0x00} ;
__attribute__ ((section(".secSdram"))) uint8_t pcmBufferTest[PCM_SIZE] = {[0 ... PCM_SIZE-1] = 0x00} ;
static uint32_t chunk_index = 0;
extern const int16_t sample[];
extern const int16_t beep[];

static void SAI_UserTxIRQHandler(void)
{
    /* Clear the FEF (Tx FIFO underrun) flag. */
    SAI_TxClearStatusFlags(DEMO_SAI, kSAI_FIFOErrorFlag);
    SAI_TxSoftwareReset(DEMO_SAI, kSAI_ResetTypeFIFO);

    SDK_ISR_EXIT_BARRIER;
}

/*! @brief SAI Receive IRQ handler.
 *
 * This function is used to handle or clear error state.
 */
static void SAI_UserRxIRQHandler(void)
{
    /* Clear the FEF (Rx FIFO overflow) flag. */
    SAI_RxClearStatusFlags(DEMO_SAI, kSAI_FIFOErrorFlag);
    SAI_RxSoftwareReset(DEMO_SAI, kSAI_ResetTypeFIFO);

    SDK_ISR_EXIT_BARRIER;
}

/*! @brief SAI IRQ handler.
 *
 * This function checks FIFO overrun/underrun errors and clears error state.
 */
void SAI2_IRQHandler(void)
{
    if (DEMO_SAI->TCSR & kSAI_FIFOErrorFlag)
        SAI_UserTxIRQHandler();

    if (DEMO_SAI->RCSR & kSAI_FIFOErrorFlag)
        SAI_UserRxIRQHandler();
}

/*!
 * @brief dump buffer
 * It dumps both pcmBuffer(32bit uncompressed samples), pcmBufferTest(16bit compressed, dithered samples).
 * @return void
 */
static void dump_buffer()
{
	uint32_t sample_32;
	uint16_t sample_16;
	uint8_t *ptr;

	for (int i=0; i<sizeof(pcmBuffer)/4; i++)
	{
		sample_32 = 0;
		sample_32 |= pcmBuffer[4*i];
		sample_32 |= pcmBuffer[4*i+1]<<8;
		sample_32 |= pcmBuffer[4*i+2]<<16;
		sample_32 |= pcmBuffer[4*i+3]<<24;
		PRINTF("%08X\r\n", sample_32);
	}

	for (int i=0; i<sizeof(pcmBuffer)/4; i++)
	{
		sample_16 = 0;
		sample_16 |= pcmBufferTest[2*i];
		sample_16 |= pcmBufferTest[2*i+1]<<8;
		PRINTF("%04X\r\n", sample_16);
	}
}

/*!
 * @brief play chosen audio through index number
 *
 * @param select - index for selecting Audio array
 * @return void
 */
static void play_music(uint8_t select)
{
	uint8_t *data_ptr;
	uint32_t data_size;
	uint32_t bit_width = kSAI_WordWidth16bits;
	switch(select)
	{
		case 0:
				data_ptr = (uint8_t *)sample;
				data_size = sizeof(sample);
				break;
		case 1:
				data_ptr = (uint8_t *)beep;
				data_size = sizeof(beep);
				break;
		case 2:
				/* Play the 32bit mono uncompressed recorded audio */
				data_ptr = pcmBuffer;
				data_size = sizeof(pcmBuffer);
				bit_width = kSAI_WordWidth32bits;
				break;
		case 3:
				/* Play the 16bit mono compressed recorded audio */
				data_ptr = pcmBufferTest;
				data_size = sizeof(pcmBufferTest)/2;
				break;
		default:
				data_ptr = pcmBuffer;
				data_size = sizeof(pcmBuffer);
				break;
	}

	if (configure_audio(bit_width, 16000U)==1)
	{
		PRINTF("[AUDIO!] CONF ERROR\r\n");
		return;
	}
	sendData2Audio(data_ptr, data_size);
	SAI_TransferTerminateSendEDMA(DEMO_SAI, &playbackHandle.saiTxHandle);
	vSemaphoreDelete(playbackHandle.semaphoreTX);
}

/*!
 * @brief send PCM data to Audio codec
 *
 * This function used for sending PCM data to SAI interface through DMA
 *
 * @param ptr - Pointer to Audio array
 * @param data_size - Size of the Audio array
 * @return void
 */
static void sendData2Audio(uint8_t * ptr, uint32_t data_size)
{
	uint32_t curr_bytes =0 ;
	uint32_t remaining_bytes;
	uint8_t buffer_select = 0;
	buffer_select = 0;
	uint8_t * samples;
	uint32_t buff_size = 0;
	uint32_t total_size = data_size - data_size%32;
	available_buff = 0;
	while(curr_bytes < total_size)
	{
		if (available_buff < 2)
		{
			if (!buffer_select)
			{
				samples = audio_buffer0;
			}
			else
			{
				samples = audio_buffer1;
			}
			remaining_bytes = total_size - curr_bytes;
			if (remaining_bytes > 4096)
			{
				buff_size = 4096;
			}
			else
			{
				buff_size = remaining_bytes;
			}
			memcpy(samples, ptr+curr_bytes, buff_size);
			curr_bytes += buff_size;
			playbackHandle.saiTx.dataSize = buff_size;
			playbackHandle.saiTx.data = samples;

			status_t sai_status;
			sai_status = SAI_TransferSendEDMA(DEMO_SAI, &playbackHandle.saiTxHandle, &playbackHandle.saiTx);
			if (sai_status == kStatus_Success)
			{

			}
			else if (sai_status == kStatus_SAI_QueueFull)
			{
				/* Wait for transfer to finish */
				if (xSemaphoreTake(playbackHandle.semaphoreTX, portMAX_DELAY) != pdTRUE)
				{
					return;
				}
				SAI_TransferSendEDMA(DEMO_SAI, &playbackHandle.saiTxHandle, &playbackHandle.saiTx);
			}
			else
			{
				PRINTF("sai error\r\n");
			}
			buffer_select ^= 1;
			available_buff++;
		}
		vTaskDelay(10);
	}
}

/*!
 * @brief SAI call back when data transfer complete
 */
static void saiTxCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData)
{
    if (kStatus_SAI_TxError == status)
    {
        /* Handle the error. */
    }
    else
    {
    	if (available_buff)
    	{
    		available_buff--;
    	}
    	playback_t *tempHandle = (playback_t *)userData;
		BaseType_t reschedule;
		xSemaphoreGiveFromISR(tempHandle->semaphoreTX, &reschedule);
		portYIELD_FROM_ISR(reschedule);
    }
}


/*!
 * @brief configure Audio
 *
 * This function used for configuring SAI(Serial Audio Interface), DMA handle
 * And the associated interrupts.
 */
static uint32_t configure_audio(uint32_t bit_width, uint32_t sample_rate)
{
	sai_transceiver_t config;
	if (bit_width != kSAI_WordWidth16bits && bit_width != kSAI_WordWidth32bits)
	{
		return 1;
	}

	/* set up NVIC priorities */
	NVIC_SetPriority(LPI2C5_IRQn, 5);
	NVIC_SetPriority(DEMO_SAI_TX_IRQ, 5U);
	NVIC_SetPriority(DMA0_DMA16_IRQn, 4U);

	/* Create DMA handle. */
	EDMA_CreateHandle(&playbackHandle.dmaTxHandle, DEMO_DMA, DEMO_TX_CHANNEL);

	/* SAI init */
	SAI_Init(DEMO_SAI);

	SAI_TransferTxCreateHandleEDMA(DEMO_SAI, &playbackHandle.saiTxHandle, saiTxCallback, (void *)&playbackHandle,
								   &playbackHandle.dmaTxHandle);

	/* I2S mode configurations */
	SAI_GetClassicI2SConfig(&config, bit_width, kSAI_Stereo, kSAI_Channel0Mask);

    config.bitClock.bclkSource = (sai_bclk_source_t)DEMO_SAI_CLOCK_SOURCE;
    SAI_TransferTxSetConfigEDMA(DEMO_SAI, &playbackHandle.saiTxHandle, &config);

    SAI_TxSetBitClockRate(DEMO_SAI, DEMO_AUDIO_MASTER_CLOCK, sample_rate, bit_width,
                          DEMO_AUDIO_DATA_CHANNEL);

    playbackHandle.semaphoreTX = xSemaphoreCreateBinary();

    /* Enable SAI transmit and FIFO error interrupts. */
    SAI_TxEnableInterrupts(DEMO_SAI, kSAI_FIFOErrorInterruptEnable);
    return 0;
}

/*!
 * @brief FREERTOS task: "Audio playback task"
 *
 * This task handles playing audio based on the queue data received from Voice task.
 */
void audio_playback_task(void *param)
{
	app_ptr = (app_handle_t *)param;
	int play_error = 0;
	player_commandQ = app_ptr->cmd_queue;

	uint32_t count = 0;
	int eap_par = 0;

	CODEC_SetVolume(&codecHandle, kCODEC_PlayChannelHeadphoneLeft | kCODEC_PlayChannelHeadphoneRight, curr_volume);

	/* start the track pointer from the beginning by default */
	BaseType_t xResult;
	uint32_t ulNotifiedValue;

	while(1)
	{
		xResult = xQueueReceive(*player_commandQ, &(audio_recvd_cmd), 100);

		if (xResult == pdTRUE)
		{
			switch(audio_recvd_cmd.command_type)
			{
				case PLAYER_CMD_PLAY:
					PRINTF("[Audio]  Playing recorded data\r\n");
					play_music(audio_recvd_cmd.buffer[0]);
					PRINTF("[Audio]  *** Player stopped ***\r\n");
					break;
				case PLAYER_CMD_RECORD:
					PRINTF("[Audio]  Recording data\r\n");
					play_music(1);
					 vTaskDelay(500 / portTICK_PERIOD_MS);
					enable_record(true);
					xResult = xTaskNotifyWait(pdFALSE, 0xffffffff, &ulNotifiedValue, 12000/portTICK_PERIOD_MS);
					if (xResult == pdTRUE)
					{
						PRINTF("[Audio]  Record success\r\n");
						enable_record(false);
						play_music(1);
					}else
					{
						PRINTF("[Audio!]  Record error\r\n");
					}
					break;
				case 2:

				default:
					PRINTF("[Audio]  Unknown command received from voice task");
					break;
			}
		}
		if (GPIO_PinRead(GPIO13, 0)==0)
		{
			PRINTF("[Audio]  *** Dump memory starts *****\r\n");
			/* dump recorded samples. */
			dump_buffer();
			PRINTF("[Audio]  *** END ***\r\n");
		}
		vTaskDelay(100);
	}
}
