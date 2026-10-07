/**
 * @file current_loop.hpp
 * @brief Current control loop (FOC) interfaces.
 */

#pragma once

#include "arm_math.h"                 // CMSIS-DSP
#include "constants.hpp"              // for ADC_GAIN
#include "dsp/controller_functions.h" // CMSIS-DSP
#include "main.h"
#include "motor_control.hpp"
#include "position.hpp"

// ---- Compile-time parameters ----
constexpr float32_t ADC_TO_VOLT = 3.3f / 4095.0f;
constexpr float32_t ADC_TO_CURR_SCALE = 3.3f / (ADC_GAIN * 4095.0f);
constexpr float32_t CURRENT_OFFSET = 1.65f / ADC_GAIN;

extern dq_t i_dq_sen;

/**
 * @brief Execute one iteration of the current control loop.
 * @param pInst Position instance (used for electrical angle).
 * @param ref_i Q-axis current reference. Measured current is in mechanical
 * frame.
 *
 *
 * The routine acquires ADC samples, performs Clarke/Park transforms,
 * runs the PI controllers, and updates the PWM duty cycle.
 */
void current_loop(position_instance_f32_t *pInst, float32_t ref_i);

/**
 * @brief Apply a fixed d-axis voltage command for alignment/fluxing.
 * @param deg Electrical angle in degrees.
 */
void flux_control(float32_t deg);

/**
 * @brief Initialize peripherals and PI controllers used by the current loop.
 */
void current_loop_init(void);

/**
 * @brief Update current loop cutoff frequency.
 * @param cutoff_hz Cutoff frequency in Hz.
 */
void current_loop_set_cutoff_freq(float32_t cutoff_hz);

/**
 * @brief Get the current loop cutoff frequency.
 * @return Cutoff frequency in Hz.
 */
float32_t current_loop_get_cutoff_freq(void);
