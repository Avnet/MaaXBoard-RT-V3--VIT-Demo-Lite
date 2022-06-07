// -----------------------------------------------------------------------------------------------------------------------------
// Customer boilerplate
//
// -----------------------------------------------------------------------------------------------------------------------------
// 
// dyanamic_compressor.h
// Part of dynamic_signal_compressor (prototype, modified 1)
//
// Created: 		06/03/2022
// Modified (1):	08/03/2022
// Author:  		Seb Sikora (ssikora)
//
// A reimplementation of the signal compression algorithm used in AudioEffectCompressor by Chip Audette,
// 2016 - 2017, published under the MIT license, a part of the OpenAudio Arduino library
// (see: https://github.com/chipaudette/OpenAudio_ArduinoLibrary/blob/master/AudioEffectCompressor_F32.h)
//
// Makes use of ARM DSP extensions - https://www.keil.com/pack/doc/CMSIS/DSP/html/index.html
//
// -----------------------------------------------------------------------------------------------------------------------------

#ifndef H_DYN_COMP
#define H_DYN_COMP

#define PCM_CHUNK_SIZE_IN_FRAMES	160		// Set buffer sizes

#include <stdint.h>

// Interface

// Call dynamic_compression_init() first once to calculate necessary constants.
void dynamic_compression_init(double compression_ratio, double threshold_percent_fs, double attack_coeff_secs, double release_coeff_secs, float output_gain_boost, double sample_rate_hz);

// Call dynamic_compression_apply() to run the dynamic compression algorithm on the input buffer containing 32-bit float samples.
// The function can work on the values in-place, so the input and output buffer can be the same.
void dynamic_compression_apply(float *input_level_buffer_ptr, float* output_level_buffer_ptr, uint32_t buffer_legnth);

#endif
