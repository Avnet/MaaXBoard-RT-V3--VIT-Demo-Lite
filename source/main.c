/*
 * Copyright 2020-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* Board includes */
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "main.h"

#include "fsl_debug_console.h"

#include "fsl_codec_common.h"
#include "fsl_sgtl5000.h"
#include "fsl_codec_adapter.h"
#include "fsl_dmamux.h"
#include "app_definitions.h"

/* Freertos headers add */
#include "FreeRTOS.h"
#include "message_buffer.h"
#include "task.h"

/* voice task */
#include "voice_task.h"

/* audio player */
#include "audio_player.h"
#include "fsl_edma.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define CPU_NAME "iMXRT1176"

#define TASK_STACK_SIZE  (1024)

/*******************************************************************************
 * Definitions
 ******************************************************************************/
static int BOARD_CODEC_Init(void);
void BOARD_InitHardware(void);

/*******************************************************************************
 * Code
 ******************************************************************************/
/*******************************************************************************
 * Variables
 ******************************************************************************/
/* counter for CPU load measurement */
static uint32_t perfCounter = 0;
/*******************************************************************************
 * Variables
 ******************************************************************************/
codec_handle_t codecHandle   = {0};
/* codec config */
sgtl_config_t sgtlConfig = {
	.i2cConfig        = {.codecI2CInstance = BOARD_CODEC_I2C_INSTANCE, .codecI2CSourceClock = BOARD_CODEC_I2C_CLOCK_FREQ},
	.route            = kSGTL_RoutePlaybackandRecord,
	.bus              = kSGTL_BusI2S,
	.slaveAddress     = SGTL5000_I2C_ADDR,
	.format           = {
			.mclk_HZ       = 24576000U,
			.sampleRate    = 44100,
			.bitWidth      = 16U
	},
	.master_slave     = false,
};

codec_config_t boardCodecConfig = {.codecDevType = kCODEC_SGTL5000, .codecDevConfig = &sgtlConfig};

/*
 * AUDIO PLL setting: Frequency = Fref * (DIV_SELECT + NUM / DENOM) / (2^POST)
 *                              = 24 * (32 + 96/125)  / 2
 *                              = 393.216 MHZ
 */
/* Configure Audio PLL clock to 393.216 MHz to  to be divisible by 48000 Hz */
const clock_audio_pll_config_t audioPllConfig = {
    .loopDivider = 32,  /* PLL loop divider. Valid range for DIV_SELECT divider value: 27~54. */
    .postDivider = 1,   /* Divider after the PLL, should only be 1, 2, 4, 8, 16. */
    .numerator   = 96,  /* 30 bit numerator of fractional loop divider. */
    .denominator = 125, /* 30 bit denominator of fractional loop divider */
};


static app_handle_t app;
static voice_task_param_t voice_params;
static TaskHandle_t xVoiceTaskHandle = NULL;
static TaskHandle_t xPlayerTaskHandle = NULL;
static QueueHandle_t player_commandQ = NULL;

/* Function Prototypes */
static void BOARD_EnableSaiMclkOutput(bool enable);
static void Voice_Task(void *param);

int main(void)
{
    int ret;
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    CLOCK_InitAudioPll(&audioPllConfig);

    /*Clock setting for LPI2C*/
    CLOCK_SetRootClockMux(kCLOCK_Root_Lpi2c5, 1);

    /*Clock setting for SAI2*/
    CLOCK_SetRootClockMux(kCLOCK_Root_Sai2, kCLOCK_SAI2_ClockRoot_MuxAudioPllOut);
    CLOCK_SetRootClockDiv(kCLOCK_Root_Sai2, 16);

    /* 24.576m mic root clock */
    CLOCK_SetRootClockMux(kCLOCK_Root_Mic, kCLOCK_MIC_ClockRoot_MuxAudioPllOut);
    CLOCK_SetRootClockDiv(kCLOCK_Root_Mic, 16);

    clock_root_config_t rootCfg = {0};
    rootCfg.mux = kCLOCK_LPI2C5_ClockRoot_MuxOscRc16M;
    rootCfg.div = 1;
    CLOCK_SetRootClock(kCLOCK_Root_Lpi2c5, &rootCfg);

    /*Enable MCLK clock*/
    BOARD_EnableSaiMclkOutput(true);

    /* Init DMAMUX */
    DMAMUX_Init(DEMO_DMAMUX);
    DMAMUX_SetSource(DEMO_DMAMUX, DEMO_TX_CHANNEL, (uint8_t)DEMO_SAI_TX_SOURCE);
    DMAMUX_EnableChannel(DEMO_DMAMUX, DEMO_TX_CHANNEL);

	/* Init DMA for PDM peripheral */
    DMAMUX_SetSource(DEMO_DMAMUX, DEMO_RX_CHANNEL, DEMO_PDM_REQUEST_SOURCE);
    DMAMUX_EnableChannel(DEMO_DMAMUX, DEMO_RX_CHANNEL);

    edma_config_t dmaConfig;
    EDMA_GetDefaultConfig(&dmaConfig);
    EDMA_Init(DEMO_DMA, &dmaConfig);

    PRINTF("\r\n");
    PRINTF("**********************************\r\n");
    PRINTF("MaaXBoard Voice Base Demo start\r\n");
    PRINTF("**********************************\r\n");

    ret = BOARD_CODEC_Init();
    if (ret)
    {
        PRINTF("CODEC_Init failed\r\n");
        return -1;
    }
    /* Queue for communicating between "voice_task" and "playback task" */
    player_commandQ = xQueueCreate(10, sizeof(queue_command_t));
    if (player_commandQ != NULL)
    {
    	vQueueAddToRegistry(player_commandQ, "playerCommandQ");
    }

	app.cmd_queue = &player_commandQ;
	/* voice control task */
	if (xTaskCreate(audio_playback_task, "playback Task", TASK_STACK_SIZE, &app, configMAX_PRIORITIES - 5,
			&xPlayerTaskHandle) != pdPASS)
	{
		PRINTF("\r\nFailed to create application task\r\n");
		while (1);
	}

    voice_params.cmd_queue = &player_commandQ;
    voice_params.player_task_handle = &xPlayerTaskHandle;
	/* voice control task */
	if (xTaskCreate(Voice_Task, "Voice Task", TASK_STACK_SIZE, &voice_params, configMAX_PRIORITIES - 5,
			&xVoiceTaskHandle) != pdPASS)
	{
		PRINTF("\r\nFailed to create application task\r\n");
		while (1);
	}

    /* Run RTOS */
    vTaskStartScheduler();

    /* Should not reach this statement */
    return 0;
}

/*******************************************************************************
 * Code
 ******************************************************************************/
static void BOARD_EnableSaiMclkOutput(bool enable)
{
    if (enable)
    {
        IOMUXC_GPR->GPR1 |= IOMUXC_GPR_GPR1_SAI2_MCLK_DIR_MASK;
    }
    else
    {
        IOMUXC_GPR->GPR1 &= (~IOMUXC_GPR_GPR1_SAI2_MCLK_DIR_MASK);
    }
}

static int BOARD_CODEC_Init(void)
{
    if (CODEC_Init(&codecHandle, &boardCodecConfig) != kStatus_Success)
    {
        return 1;
    }

    /* Initial volume kept low for hearing safety. */
    if (CODEC_SetVolume(&codecHandle, kCODEC_PlayChannelHeadphoneLeft | kCODEC_PlayChannelHeadphoneRight, 90) != kStatus_Success)
    {
    	return 1;
    }
    return 0;
}

static void Voice_Task(void *param)
{
	if (!start_vit(param))
	{
		PRINTF("[VIT] voice task error\r\n");
		vTaskDelete(NULL);
		return;
	}
	while(1)
	{
		vTaskDelay(10/portTICK_PERIOD_MS);
	}
}

/*!
 * @brief PIT1 Timer interrupt service ISR
 */
void PIT1_IRQHANDLER(void)
{
	PIT_ClearStatusFlags(PIT1_PERIPHERAL, PIT1_CHANNEL_0, kPIT_TimerFlag);
	perfCounter++;
	__DSB();
}

/*!
 * @brief Configures the PIT timer, it will be called by Freertos
 */
void AppConfigureTimerForRuntimeStats(void) {
	pit_config_t config;

	PIT_GetDefaultConfig(&config);
	config.enableRunInDebug = false;
	PIT_Init(PIT1_PERIPHERAL, &config);
	PIT_SetTimerPeriod(PIT1_PERIPHERAL, PIT1_CHANNEL_0, PIT1_CHANNEL_0_TICKS);
	PIT_EnableInterrupts(PIT1_PERIPHERAL, PIT1_CHANNEL_0, kPIT_TimerInterruptEnable);
	EnableIRQ(PIT1_IRQN);
	PIT_StartTimer(PIT1_PERIPHERAL, PIT1_CHANNEL_0);
}

/*!
 * @brief Returns 32bit counter value. Used for freertos runtime analysis
 */
uint32_t AppGetRuntimeCounterValueFromISR(void) {
	return perfCounter;
}

/**
 * @brief Loop forever if stack overflow is detected.
 *
 * If configCHECK_FOR_STACK_OVERFLOW is set to 1,
 * this hook provides a location for applications to
 * define a response to a stack overflow.
 *
 * Use this hook to help identify that a stack overflow
 * has occurred.
 *
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    portDISABLE_INTERRUPTS();

    /* Loop forever */
    for (;;)
        ;
}

/**
 * @brief Warn user if pvPortMalloc fails.
 *
 * Called if a call to pvPortMalloc() fails because there is insufficient
 * free memory available in the FreeRTOS heap.  pvPortMalloc() is called
 * internally by FreeRTOS API functions that create tasks, queues, software
 * timers, and semaphores.  The size of the FreeRTOS heap is set by the
 * configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h.
 *
 */
void vApplicationMallocFailedHook()
{
    PRINTF(("\r\nERROR: Malloc failed to allocate memory\r\n"));
}
