// -----------------------------------------------------------------------------------------------------------------------------
// Customer boilerplate
//
// -----------------------------------------------------------------------------------------------------------------------------
// 
// dyanamic_compressor.c
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

#include <stdint.h>				// stipulated-width ints
#include <stdlib.h>
#include <math.h>				// log10f(), log10(), powf(), expf()
#include <stdio.h>

#include "dynamic_compressor.h"
//#include "fake_arm_math.h"		// Uncomment appropriately for build target
#include <arm_math.h>

// Compressor algorithm-specific private functions
static void lp_filter(double *lp_filter_persistent_state_ptr, float *sample_buffer, uint32_t buffer_legnth);
static void calculate_audio_level_db(double *lp_filter_persistent_state_ptr, float *input_audio_buffer_ptr, float *level_db_buffer_ptr, uint32_t buffer_legnth);
static void calculate_gain(float *level_db_buffer_ptr, float *gain_buffer_ptr, uint32_t buffer_legnth);
static void calculate_instantaneous_target_gain_db(float *level_db_buffer_ptr, float *instant_target_gain_db_buffer_ptr, uint32_t buffer_legnth);
static void calculate_smooth_target_gain_db(double *gain_smoothing_persistent_state_ptr, float *instant_target_gain_db_buffer_ptr, float *smooth_target_gain_db_buffer_ptr, uint32_t buffer_legnth);

// Misc private functions
static void apply_buffer_cap(float *src_buffer_ptr, float cap_value, float *dest_buffer_ptr, uint32_t buffer_legnth);
static void apply_buffer_floor(float *src_buffer_ptr, float floor_value, float *dest_buffer_ptr, uint32_t buffer_legnth);
static double max(double lhs, double rhs);
static double min(double lhs, double rhs);
static float pow10f(float x);

// User variable
static double m_comp_ratio = 0.0;
static double m_thold_percent_fs = 30.0;
static double m_attack_coeff_secs = 0.005;		// Change this with caution, significantly changes response. Sensible default.
static double m_release_coeff_secs = 0.2;		// " "
static double m_sample_rate_hz = 16000.0;
static float m_output_gain_boost = 1.0;

// Internal
static float m_thold_db_fs = 0.0;
static double m_lp_filter_secs = 0.0;
static double m_lp_filter_coeff = 0.0;
static double m_lp_filter_c1 = 0.0;
static double m_lp_filter_c2 = 0.0;
static double m_attack_constant = 0.0;
static double m_release_constant = 0.0;
static double m_lp_filter_persistent_state = 1.0;
static double m_gain_smoothing_persistent_state = 0.0;

// Buffers
float m_input_power_buffer[PCM_CHUNK_SIZE_IN_FRAMES];
float m_input_level_db_buffer[PCM_CHUNK_SIZE_IN_FRAMES];
float m_above_thold_db_buffer[PCM_CHUNK_SIZE_IN_FRAMES];
float m_instant_target_gain_db_buffer[PCM_CHUNK_SIZE_IN_FRAMES];
float m_smooth_target_gain_db_buffer[PCM_CHUNK_SIZE_IN_FRAMES];
float m_gain_buffer[PCM_CHUNK_SIZE_IN_FRAMES];

// Interface -------------------------------------------------------------------------------------------------------------------

// Call dynamic_compression_init() first once to calculate necessary constants.
void dynamic_compression_init(double compression_ratio, double threshold_percent_fs, double attack_coeff_secs, double release_coeff_secs, float output_gain_boost, double sample_rate_hz) {
	
	// User variables
	m_comp_ratio = compression_ratio;
	m_thold_percent_fs = threshold_percent_fs;
	m_attack_coeff_secs = attack_coeff_secs;
	m_release_coeff_secs = release_coeff_secs;
	m_sample_rate_hz = sample_rate_hz;
	m_output_gain_boost = output_gain_boost;
	
	// Coeffs setup
	m_lp_filter_secs = max(0.002, min(m_attack_coeff_secs, m_release_coeff_secs));	
	m_lp_filter_coeff = exp(-1.0 / (m_lp_filter_secs * m_sample_rate_hz));
	m_thold_db_fs = (float)(20.0 * log10((m_thold_percent_fs / 100.0)));
	m_lp_filter_c1 = m_lp_filter_coeff;
	m_lp_filter_c2 = 1.0 - m_lp_filter_c1;
	
	m_attack_constant = exp(-1.0 / (m_attack_coeff_secs * m_sample_rate_hz));
	m_release_constant = exp(-1.0 / (m_release_coeff_secs * m_sample_rate_hz));
	
	m_lp_filter_persistent_state = 1.0;
	m_gain_smoothing_persistent_state = 0.0;
	
	//~ printf(" --- Computed constants ---\n");
	//~ printf("m_lp_filter_secs = %f\n", m_lp_filter_secs);
	//~ printf("m_lp_filter_coeff = %f\n", m_lp_filter_coeff);
	//~ printf("m_thold_db_fs = %f\n", m_thold_db_fs);
	//~ printf("m_attack_constant = %f\n", m_attack_constant);
	//~ printf("m_release_constant = %f\n", m_release_constant);
	
}

// Call dynamic_compression_apply() to run the dynamic compression algorithm on the input buffer containing 32-bit float samples.
// The function can work on the values in-place, so the input and output buffer can be the same.
void dynamic_compression_apply(float *input_level_buffer_ptr, float* output_level_buffer_ptr, uint32_t buffer_legnth) {
	
	// Calculate the level (in dB) of the audio (ie, calculate a smoothed version of the signal power)
	calculate_audio_level_db(&m_lp_filter_persistent_state, input_level_buffer_ptr, &m_input_level_db_buffer[0], buffer_legnth);
	
	// Calculate the desired gain based on the observed audio level
	calculate_gain(&m_input_level_db_buffer[0], &m_gain_buffer[0], buffer_legnth);
	
	// Apply the calculated gain
	arm_mult_f32(input_level_buffer_ptr, &m_gain_buffer[0], output_level_buffer_ptr, buffer_legnth);
	
	// Apply the post compression gain boost (optional - can be used to level out overall input and post-compression output volumes)
	arm_scale_f32(output_level_buffer_ptr, m_output_gain_boost, output_level_buffer_ptr, buffer_legnth);
	
}

// Internal functions ----------------------------------------------------------------------------------------------------------

// Algorithm related.

static void calculate_audio_level_db(double *lp_filter_persistent_state_ptr, float *input_level_buffer_ptr, float *input_db_buffer_ptr, uint32_t buffer_legnth) {
	
	// Calculate the instantaneous signal power (amplitude^2)
	arm_mult_f32(input_level_buffer_ptr, input_level_buffer_ptr, &m_input_power_buffer[0], buffer_legnth);
	
	// Low-pass filter the instantaneous signal power
	lp_filter(lp_filter_persistent_state_ptr, &m_input_power_buffer[0], buffer_legnth);
	
	// Convert instantaneous signal power to dB (dB = 10.0 * log10(power))
	for (uint32_t index = 0; index < buffer_legnth; index ++) {
		*(input_db_buffer_ptr + index) = log10f(m_input_power_buffer[index]);		// log10(x)
	}
	arm_scale_f32(input_db_buffer_ptr, 10.0, input_db_buffer_ptr, buffer_legnth);	// x * 10
	
}

static void lp_filter(double *lp_filter_persistent_state_ptr, float *sample_buffer, uint32_t buffer_legnth) {
	
	double result;
	for (uint32_t index = 0; index < buffer_legnth; index ++) {
		result = (m_lp_filter_c1 * (*(lp_filter_persistent_state_ptr))) + (m_lp_filter_c2 * ((double)*(sample_buffer + index)));
		*(sample_buffer + index) = (float)(result);
		*(lp_filter_persistent_state_ptr) = result;	
	}
	
	// Limit the amount that the state of the smoothing filter can go toward negative infinity
	if (*(lp_filter_persistent_state_ptr) < (1.0E-13)) {	//never go less than -130 dBFS 
		*(lp_filter_persistent_state_ptr) = 1.0E-13;
	}
	
}

static void calculate_gain(float *level_db_buffer_ptr, float *gain_buffer_ptr, uint32_t buffer_legnth) {
	
	// First, calculate the instantaneous target gain as a function of the current level dB and compression ratio
	calculate_instantaneous_target_gain_db(level_db_buffer_ptr, &m_instant_target_gain_db_buffer[0], buffer_legnth);
	
	// Second, smooth in time (attack and release) by introducing some hysteresis
	calculate_smooth_target_gain_db(&m_gain_smoothing_persistent_state, &m_instant_target_gain_db_buffer[0], &m_smooth_target_gain_db_buffer[0], buffer_legnth);
	
	// Finally, convert from dB to linear gain: gain = 10^(gain_dB/20);  (ie this takes care of the sqrt, too!)
	arm_scale_f32(&m_smooth_target_gain_db_buffer[0], 1.0 / 20.0, &m_smooth_target_gain_db_buffer[0], buffer_legnth);		// x / 20
	for (uint32_t index = 0; index < buffer_legnth; index ++) {
		*(gain_buffer_ptr + index) = pow10f(m_smooth_target_gain_db_buffer[index]);											// 10 ^ x
	}
	
}

static void calculate_instantaneous_target_gain_db(float *level_db_buffer_ptr, float *instant_target_gain_db_buffer_ptr, uint32_t buffer_legnth) {
	
	// How much are we above the compression threshold?
	arm_offset_f32(level_db_buffer_ptr, -1.0 * m_thold_db_fs, &m_above_thold_db_buffer[0], buffer_legnth);
	
	// Scale the overshoot by the compression ratio to get the target level
	arm_scale_f32(&m_above_thold_db_buffer[0], 1.0 / m_comp_ratio, instant_target_gain_db_buffer_ptr, buffer_legnth);
	
	// Compute the instantaneous target gain, which is the difference between the target level and the original overshoot
	arm_sub_f32(instant_target_gain_db_buffer_ptr, &m_above_thold_db_buffer[0], instant_target_gain_db_buffer_ptr, buffer_legnth);
	
	// Limit the target gain to attenuation only (this part of the compressor should not make things louder!)
	apply_buffer_cap(instant_target_gain_db_buffer_ptr, 0.0, instant_target_gain_db_buffer_ptr, buffer_legnth);
	
}

static void calculate_smooth_target_gain_db(double *gain_smoothing_persistent_state_ptr, float *instant_target_gain_db_buffer_ptr, float *smooth_target_gain_db_buffer_ptr, uint32_t buffer_legnth) {
	
	double gain_db;
	double result;
	double one_minus_attack_constant = 1.0 - m_attack_constant;
	double one_minus_release_constant = 1.0 - m_release_constant;
	
	for (uint32_t index = 0; index < buffer_legnth; index ++) {
		gain_db = (double)*(instant_target_gain_db_buffer_ptr + index);
		
		// Smooth the gain using the attack or release constants
		if (gain_db < *(gain_smoothing_persistent_state_ptr)) {		  // Are we in the attack phase?
			result = (m_attack_constant * (*(gain_smoothing_persistent_state_ptr))) + (one_minus_attack_constant * gain_db);
		} else {													  // Else, we're in the release phase
			result = (m_release_constant * (*(gain_smoothing_persistent_state_ptr))) + (one_minus_release_constant * gain_db);
		}
		
		*(smooth_target_gain_db_buffer_ptr + index) = (float)result;
		*(gain_smoothing_persistent_state_ptr) = result;
	}
	
}

// Utils.

static void apply_buffer_cap(float *src_buffer_ptr, float cap_value, float *dest_buffer_ptr, uint32_t buffer_legnth) {
	
	for (uint32_t index = 0; index < buffer_legnth; index ++) {
		if (*(src_buffer_ptr + index) > cap_value) {
			*(dest_buffer_ptr + index) = cap_value;
		} else {
			*(dest_buffer_ptr + index) = *(src_buffer_ptr + index);
		}
	}
	
}

static void apply_buffer_floor(float *src_buffer_ptr, float floor_value, float *dest_buffer_ptr, uint32_t buffer_legnth) {
	
	for (uint32_t index = 0; index < buffer_legnth; index ++) {
		if (*(src_buffer_ptr + index) < floor_value) {
			*(dest_buffer_ptr + index) = floor_value;
		} else {
			*(dest_buffer_ptr + index) = *(src_buffer_ptr + index);
		}
	}
	
}

static double max(double lhs, double rhs) {
	if (lhs >= rhs) {
		return lhs;
	} else {
		return rhs;
	}
}

static double min(double lhs, double rhs) {
	if (lhs < rhs) {
		return lhs;
	} else {
		return rhs;
	}
}

// Accelerate the powf(10.0,x) function
static float pow10f(float x) {
	//return powf(10.0f,x)   //standard, but slower
	return expf(2.302585092994f*x);  //faster:  exp(log(10.0f)*x)
}

