/*
 * mp3_player.h
 *
 *  Created on: Dec 3, 2021
 *      Author: gulziibayar
 */

#ifndef AUDIO_PLAYER_H_
#define MP3_PLAYER_H_

#include "fsl_dmamux.h"
#include "fsl_sai_edma.h"
#include "FreeRTOS.h"
#include "portable.h"
#include "semphr.h"

struct track_node{
	char fileName[40];
	struct track_node *next;
	struct track_node *prev;
};

/*! @brief PCM interface structure */
typedef struct _playback_t
{
    sai_transfer_t saiTx;
    sai_edma_handle_t saiTxHandle;
    edma_handle_t dmaTxHandle;
    SemaphoreHandle_t semaphoreTX;
} playback_t;

void audio_playback_task(void *param);

#endif /* AUDIO_PLAYER_H_ */
