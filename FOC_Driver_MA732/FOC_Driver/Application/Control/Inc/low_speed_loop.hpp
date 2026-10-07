#ifndef LOW_SPEED_LOOP_HPP
#define LOW_SPEED_LOOP_HPP

#include "main.h"
#include "constants.hpp"
#include "position.hpp"

/**
 * @brief Initialize the velocity control loop.
 */
void velocity_loop_init(void);
void position_loop_init(void);

/**
 * @brief Run one cycle of the velocity control loop.
 * @param pInst Position instance.
 * @param rpm_ref Speed reference in rpm.
 * @return Q-axis current reference.
 */
float32_t velocity_loop(position_instance_f32_t *pInst, float32_t rpm_ref);
float32_t position_loop(position_instance_f32_t *pInst, float32_t theta_ref);

/**
 * @brief Set velocity loop proportional gain.
 */
void velocity_loop_set_kp(float32_t kp);

/**
 * @brief Set velocity loop integral gain.
 */
void velocity_loop_set_ki(float32_t ki);

/**
 * @brief Get current velocity loop gains.
 * @param kp Output proportional gain.
 * @param ki Output integral gain.
 */
void velocity_loop_get_gains(float32_t *kp, float32_t *ki);

#endif // LOW_SPEED_LOOP_HPP
