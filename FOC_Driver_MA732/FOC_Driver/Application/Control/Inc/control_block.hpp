/**
 * @file control_block.hpp
 * @brief PID control block utilities.
 */

#pragma once
#include "arm_math_memory.h"
#include "arm_math_types.h"
#include "clamp_function.hpp"

/**
 * @brief Floating-point PID control structure.
 *
 * Contains coefficients and state variables for a PID controller.
 */
typedef struct
{
    struct
    {
        float32_t i[2]; // i[0] = x[n-1], i[1] = y[n-1]
        float32_t d[2]; // d[0] = x[n-1], d[1] = y[n-1]
    } state;

    float32_t lower_bound;
    float32_t upper_bound;

    float32_t Kp; // Proportional gain coefficient
    float32_t Ki; // Integral gain coefficient
    float32_t Kd; // Derivative gain coefficient
} control_instance_f32_t;

/**
 * @brief Initialize the floating-point PID control instance.
 * @param S Pointer to the control instance structure.
 * @param Kp Proportional gain.
 * @param Ki Integral gain.
 * @param Kd Derivative gain.
 * @param lower_bound Output lower bound.
 * @param upper_bound Output upper bound.
 * @param resetStateFlag Reset state variables (0 = no change, 1 = reset).
 */
__STATIC_FORCEINLINE void control_init_f32(
    control_instance_f32_t *S, float32_t Kp, float32_t Ki, float32_t Kd,
    float32_t lower_bound, float32_t upper_bound,
    int32_t resetStateFlag)
{

    // Initialize control coefficients
    S->Kp = Kp;
    S->Ki = Ki;
    S->Kd = Kd;
    S->lower_bound = lower_bound;
    S->upper_bound = upper_bound;

    // Initialize state variables to zero if resetStateFlag is set
    if (resetStateFlag)
    {
        S->state.i[0] = 0.0f;
        S->state.i[1] = 0.0f;
        S->state.d[0] = 0.0f;
        S->state.d[1] = 0.0f;
    }
}

/**
 * @brief Process one step of the PID controller.
 * @param[in,out] S Pointer to the control instance structure.
 * @param[in] in Input sample.
 * @return Output sample.
 */
__STATIC_FORCEINLINE float32_t control_f32(
    control_instance_f32_t *S,
    float32_t in)
{
    float32_t y_p, y_i, y_d, y;

    // Calculate output using PID formula
    y_p = S->Kp * in;                                     // Proportional term
    y_i = S->Ki * (in + S->state.i[0]) + S->state.i[1];   // Integral term
    y_i = clamp_f32(y_i, S->lower_bound, S->upper_bound); // Clamp integral term
    y_d = S->Kd * (in - S->state.d[0]) - S->state.d[1];   // Derivative
    y = y_p + y_i + y_d;
    S->state.i[0] = in; // Update state variable for next iteration
    S->state.i[1] = y_i;
    S->state.d[0] = in; // Update state variable for next iteration
    S->state.d[1] = y_d;
    return y;
}