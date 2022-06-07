/*
 * Copyright 2020-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __MAIN_H__
#define __MAIN_H__

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "ff.h"
//usb stuff added
#include "usb_host_config.h"
#include "usb_host.h"
#include "fsl_device_registers.h"

/* PIT timer header */
#include "fsl_pit.h"
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* PIT Timer used for runtime analysis on Freertos */
#define PIT1_PERIPHERAL PIT1
/* Definition of clock source frequency. */
#define PIT1_CLK_FREQ 240000000UL
/* Definition of ticks count for channel 0 - deprecated. */
#define PIT1_0_TICKS 23999U
/* PIT1 interrupt vector ID (number) - deprecated. */
#define PIT1_0_IRQN PIT0_IRQn
/* PIT1 interrupt handler identifier - deprecated. */
#define PIT1_0_IRQHANDLER PIT0_IRQHandler
/* Definition of channel number for channel 0. */
#define PIT1_CHANNEL_0 kPIT_Chnl_0
/* Definition of ticks count for channel 0. */
#define PIT1_CHANNEL_0_TICKS 23999U
/* PIT1 interrupt vector ID (number). */
#define PIT1_IRQN PIT1_IRQn
/* PIT1 interrupt handler identifier. */
#define PIT1_IRQHANDLER PIT1_IRQHandler

/* Command list from VIT */
#define CMD_PLAY_SAMPLE		1
#define CMD_RECORD 			2
#define CMD_PLAY_RECORD		3
#define CMD_LED_RED 		4
#define CMD_LED_GREEN 		5
#define CMD_LED_BLUE	 	6
#define CMD_LED_OFF			7
#define CMD_PLAY_COMPRESSED	8

#define PLAYER_CMD_PLAY		0
#define PLAYER_CMD_RECORD	1

/* size for pcmBuffer */
#define PCM_CHUNK_SIZE	2048
#define NUM_OF_CHUNK	100
#define PCM_SIZE	PCM_CHUNK_SIZE * NUM_OF_CHUNK

typedef struct _app_handle
{
	QueueHandle_t *cmd_queue;
} app_handle_t;

/* struct wrapper for freertos task */
typedef struct
{
	QueueHandle_t *cmd_queue;
	TaskHandle_t *player_task_handle;
}voice_task_param_t;

typedef struct _queue_command
{
	uint8_t command_type;
	uint8_t taskId;
	uint8_t buffer[24];
}queue_command_t;


#endif /* __MAIN_H__ */
