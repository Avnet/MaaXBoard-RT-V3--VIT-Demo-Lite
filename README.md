# VIT lite-version demo

VIT lite-version demo is the FreerTOS project has voice control, record, audio playback functionalities. On the recorded samples, triangular dithering, dynamic compression algorithm are applied in real time as to minimize audio clipping and minimize audio quality loss during conversion from 24bit to 16bit.

The project can be interacted with following 8 voice commands.

```
VIT_Model version : v5.4.0

WakeWord supported : " HEY AVNET "

Voice Commands supported
    Cmd_Id : Cmd_Name
      0    : UNKNOWN
      1    : PLAY SAMPLE
      2    : RECORD
      3    : PLAY RECORD
      4    : LED RED
      5    : LED GREEN
      6    : LED BLUE
      7    : LED OFF
      8    : PLAY COMPRESSED
```

*Note: </br>SAI peripheral is configured @(sample_rate: 16khz, bit_width: 16bit/32bit, channel: mono). </br>PDM peripheral is configured to read single channel microphone @(sample_rate: 16khz, bit_width: 32bit(24bit+8bit padding))*

### Commands Description:
1. PLAY SAMPLE - Playback demo audio (channel:1, bit_width:16, sample_rate:16Khz) is stored as byte array in the project source folder as `sample_mono.h`.
2. RECORD - Record the microphone PCM data in the SDRAM memory region. </br>
`__attribute__ ((section(".secSdram"))) uint8_t pcmBuffer[PCM_SIZE] = {[0 ... PCM_SIZE-1] = 0x00} ;` </br>
`PCM_SIZE` is defined in `main.h`. record duration ~6.4 sec. </br>`beep_mono.h` audio is used for signalling the begin and end of record.
3. Play RECORD - Playback pcm signal stored on the previously stored `pcmBuffer`. *Note: It is volatile memory, so after power on, it will be blank after power cycle*

To use their own `beep`, `sample` audio file. Please refer to following links.

* Use [audacity](https://www.audacityteam.org/) to convert any audio format to mono 16khz mono wav file. 
* Use [wavToCode](https://colinjs.com/wavtocode/wavtocode.htm) to generate C array. *Note: Windows OS only*

The Project runs 2 FreeRTOS tasks:
- Playback task - play sample/recorded audio stored on the sdram/flash.
- Voice task - VIT with 8 generated commands 

These tasks are communicated through FreeRTOS Queue. "Voice task" is the producer, "Playback task" is the consumer.

Queue data is has following structure `main.h`.
```
typedef struct _queue_command
{
	uint8_t command_type;
	uint8_t taskId;
	uint8_t buffer[24];
}queue_command_t;
```

"Voice task" handles the voice command as below. It communicates with "playback task" via queue.
Please refer to `source/vit_proc.c Line#363`.
```
/* Please enter your custom code in here. */
switch(VoiceCommand.Cmd_Id)
{
    case CMD_PLAY_SAMPLE:			// 1
        voice_command.command_type = PLAYER_CMD_PLAY;
        voice_command.buffer[0] = 0;
        xQueueSend(*player_commandQ, (void *) &voice_command, 10);
        break;
    case CMD_RECORD:				// 2
        voice_command.command_type = PLAYER_CMD_RECORD;
        xQueueSend(*player_commandQ, (void *) &voice_command, 10);
        break;
    case CMD_PLAY_RECORD:			// 3
        voice_command.command_type = PLAYER_CMD_PLAY;
        voice_command.buffer[0] = 2;
        xQueueSend(*player_commandQ, (void *) &voice_command, 10);
        break;
    case CMD_LED_RED:				// 4 LED RED
        set_led(RED);
        break;
    case CMD_LED_GREEN:				// 5 LED GREEN
        set_led(GREEN);
        break;
    case CMD_LED_BLUE:				// 6 LED BLUE
        set_led(BLUE);
        break;
    case CMD_LED_OFF: 				// 7 LED OFF
        set_led(BLACK);
        break;
    case CMD_PLAY_COMPRESSED:		// 8
        voice_command.command_type = PLAYER_CMD_PLAY;
        voice_command.buffer[0] = 3;
        xQueueSend(*player_commandQ, (void *) &voice_command, 10);
        break;
    case 9: 		// 9
        break;
    case 10: 		// 10
        break;
    case 11: 		// 11
        break;
    case 12:		// 12
        break;
    default:
        break;
}
```

At "Playback task", it waits for queue data. Once data is received, it will process the data and play the requested audio.
</br>
`audio_player.c line:#334`
```
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
```

# Reference links:
[VIT](https://vit.nxp.com/#/) - Creating custom voice command model.

[Miniaudio](https://github.com/mackron/miniaudio) - 24bit to 16bit conversion using dithering algorithm.

[OpenAudio_ArduinoLibrary](https://github.com/chipaudette/OpenAudio_ArduinoLibrary/blob/master/AudioEffectCompressor_F32.h) - dynamic range compression algorithm.






