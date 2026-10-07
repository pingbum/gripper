#include "low_speed_loop.hpp"
#include "arm_math_types.h"
#include "constants.hpp"
#include "control_block.hpp"
#include "motor_control.hpp" // For dq_t
#include "stm32g4xx_hal_hrtim.h"

// PI controller for the velocity loop.
static control_instance_f32_t velocity_control;
static control_instance_f32_t position_control;

// Velocity loop gains.
static float32_t Kp_vel = 0.003f;
static float32_t Ki_vel = 0.000005f;


// Position loog gains.
static float32_t Kp_pos = 0.05f; //0.2 for MCP, 0.1 for PIP, 0.05 for DIP
static float32_t Kd_pos = 0.003f;
static float32_t f_sat = 100;    //D gain saturation frequency
static float32_t alpha = (2*5000-f_sat*(2*PI)) / (2*5000+f_sat*(2*PI));
static float32_t beta = (f_sat*(2*PI)*2*5000)/(f_sat*(2*PI)+2*5000);


static void apply_velocity_gains(bool reset_state) {
  control_init_f32(&velocity_control, Kp_vel, Ki_vel, 0.000000000000f,
                   -CURRENT_MAX, CURRENT_MAX, reset_state ? 1 : 0);
}

static void apply_position_gains(bool reset_state) {
  control_init_f32(&position_control, Kp_pos, 0.00000f, Kd_pos,
                   -CURRENT_MAX, CURRENT_MAX, reset_state ? 1 : 0);
}

/**
 * @brief Initialize the velocity control loop.
 */
void velocity_loop_init() {
  // The controller output is the q-axis current reference.
  apply_velocity_gains(true);
}

void position_loop_init() {
  // The controller output is the q-axis current reference.
  apply_position_gains(true);
}

/**
 * @brief Run one cycle of the velocity control loop.
 * @param pInst Pointer to the position instance struct.
 * @return The calculated q-axis current reference.
 */
float32_t velocity_loop(position_instance_f32_t *pInst, float32_t rpm_ref) {
  // 1) Calculate velocity error.
  float32_t omega_error = rpm_ref * RPM_TO_DEG_PER_SEC - pInst->omega_m;

  // 2) PI control for velocity.
  // iq_ref in mechanical sign;
  float32_t iq_ref = control_f32(&velocity_control, omega_error);
  if (rpm_ref != 0.0f) {
    // saturation
    if (iq_ref > CURRENT_MAX) {
      return CURRENT_MAX;
    } else if (iq_ref < -CURRENT_MAX) {
      return -CURRENT_MAX;
    } else {
      return iq_ref;
    }
  }
  else
    return 0.0f;
}

float32_t position_loop(position_instance_f32_t *pInst, float32_t theta_ref) {
  float32_t theta_meas = (pInst->rev * 360.0f + pInst->theta_m) / GEAR_RATIO;
  float32_t theta_error = theta_ref - theta_meas;
  float32_t u_p = position_control.Kp * theta_error;
  float32_t theta_delta = theta_meas - position_control.state.d[0];
  float32_t u_d =
      position_control.state.d[1] * alpha
      - position_control.Kd * beta * theta_delta;

  float32_t iq_ref = u_p + u_d;

  // state update
  position_control.state.d[1] = u_d;
  position_control.state.d[0] = theta_meas;

  // saturation
  if (iq_ref > CURRENT_MAX) {
    return CURRENT_MAX;
  } else if (iq_ref < -CURRENT_MAX) {
    return -CURRENT_MAX;
  } else {
    return iq_ref;
  }
}

void velocity_loop_set_kp(float32_t kp) {
  Kp_vel = kp;
  apply_velocity_gains(false);
}

void velocity_loop_set_ki(float32_t ki) {
  Ki_vel = ki;
  apply_velocity_gains(false);
}

void velocity_loop_get_gains(float32_t *kp, float32_t *ki) {
  if (kp)
    *kp = Kp_vel;
  if (ki)
    *ki = Ki_vel;
}
