/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _VIT_PROC_H_
#define _VIT_PROC_H_

#include "PL_platformTypes_CortexM7.h"
#include "VIT.h"

/* Enum for VOICE commands for readability */
enum {
	PLAY 		=   1,
	NEXT 		=	2,
	PREVIOUS 	=  	3,
	STOP 		=	4,
	VOLUME_UP 	=   5,
	VOLUME_DOWN =   6,
	LED_RED 	=	7,
	LED_GREEN 	=   8,
	LED_BLUE 	=   9,
	LED_STATUS  =   10,
	WIRELESS_OFF=   11,
	RESET		=   12
};

void set_task_handle(void *t_handle);
void set_task_handle_wifi(void *t_handle);

typedef int (*VIT_Initialize_T)(void *arg);
typedef int (*VIT_Execute_T)(void *arg, short *inputBuffer, int size);
typedef int (*VIT_Deinit_T)(void);

extern VIT_Initialize_T VIT_Initialize_func;
extern VIT_Execute_T VIT_Execute_func;
extern VIT_Deinit_T VIT_Deinit_func;

typedef enum
{
    EN,
    CN
} VIT_Language_T;
extern VIT_Language_T Vit_Language;
#endif
