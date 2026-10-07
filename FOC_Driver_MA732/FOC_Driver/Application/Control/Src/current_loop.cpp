#include "current_loop.hpp"
#include "constants.hpp"
#include "control_block.hpp"
#include "math_tools.hpp"
#include "motor_control.hpp"

extern State_t state;

extern "C" {
extern TIM_HandleTypeDef htim2;
}

float32_t duty[3] = {0.5f, 0.5f, 0.5f}; // Initial duty cycle per phase.
uint32_t data[3];

// DSP buffers must be 32-byte aligned for best throughput.
static float32_t current_sen[ADC_BUF_SIZE] __attribute__((aligned(32)));

float32_t cutoff_freq_cur = 1800.0f;   // Hz
float32_t control_freq_cur = 50000.0f; // Control loop frequency in Hz

float32_t Kp_d = 0.0f;
float32_t Kp_q = 0.0f;
float32_t Ki_d = 0.0f;
float32_t Ki_q = 0.0f;

control_instance_f32_t id_control, iq_control;

dq_t i_ref = {0.0f, 0.0f}; // Reference current in dq frame.
alpha_beta_t i_ab_sen;
dq_t i_dq_sen;
dq_t i_error;
dq_t v_command;
dq_t v_cross; // Cross-coupling term.
alpha_beta_t v_SVM;

// Debug variables.
float32_t i_ref_abc[3];
uint32_t period_time;
uint32_t process_time;
uint32_t buf_time = 0;
uint32_t period = 0;

static void update_current_gains(void) {
  const float32_t exp_fc =
      exp(-2.0f * M_PI * cutoff_freq_cur / control_freq_cur);
  const float32_t exp_rl = exp(-MOTOR_Rs / MOTOR_Ls / control_freq_cur);

  Kp_d = MOTOR_Rs * (1.0f - exp_fc) / (1.0f - exp_rl) * (1.0f + exp_rl) / 2.0f;
  Kp_q = Kp_d;
  Ki_d = MOTOR_Rs * (1.0f - exp_fc) / 2.0f;
  Ki_q = Ki_d;
}

static void apply_current_gains(bool reset_state) {
  control_init_f32(&id_control, Kp_d, Ki_d, 0.0f, -VOLTAGE_MAX, VOLTAGE_MAX,
                   reset_state ? 1 : 0);
  control_init_f32(&iq_control, Kp_q, Ki_q, 0.0f, -VOLTAGE_MAX, VOLTAGE_MAX,
                   reset_state ? 1 : 0);
}

/**
 * @brief Clamp duty ratio and convert to PWM compare values.
 *
 * The duty array is expected to hold normalized values in the range [-0.5,0.5].
 */
__STATIC_FORCEINLINE void convertDuty(void) {

  duty[0] = clamp_f32(duty[0], 0.0f, 0.96f);
  duty[1] = clamp_f32(duty[1], 0.0f, 0.96f);
  duty[2] = clamp_f32(duty[2], 0.0f, 0.96f);

  data[0] = (uint32_t)(duty[0] * FULL_DUTY);
  data[1] = (uint32_t)(duty[1] * FULL_DUTY);
  data[2] = (uint32_t)(duty[2] * FULL_DUTY);
}

/**
 * @brief Initialize the PI controllers used in the current loop.
 */
void current_loop_init() {

  // Initialize control blocks.
  update_current_gains();
  apply_current_gains(true);
}

void current_loop_set_cutoff_freq(float32_t cutoff_hz) {
  cutoff_freq_cur = cutoff_hz;
  update_current_gains();
  apply_current_gains(false);
}

float32_t current_loop_get_cutoff_freq(void) { return cutoff_freq_cur; }

/**
 * @brief Run one cycle of the FOC current control loop.
 */

float32_t s, c;

void current_loop(position_instance_f32_t *pInst, float32_t ref_i) {
  // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // Toggle LED for
  // profiling.
  period = HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CNTxR; // Debug timing.

  static uint32_t time_prev = htim2.Instance->CNT;

  uint32_t time_current = htim2.Instance->CNT;
  uint32_t time_delta = time_current - time_prev;
  time_prev = time_current;

  updatePositionElec(pInst, (ENCODER_SCALER + (float32_t)time_delta) * DT);
  uint32_t t_start = htim2.Instance->CNT; // Debug timing.
  arm_sin_cos_f32(pInst->theta_e, &s, &c);

  // 1) Acquire new ADC samples.
  current_sen[0] = (float32_t)ADC1->JDR1 * ADC_TO_CURR_SCALE - CURRENT_OFFSET;
  current_sen[1] = (float32_t)ADC3->JDR1 * ADC_TO_CURR_SCALE - CURRENT_OFFSET;
  current_sen[2] = (float32_t)ADC2->JDR1 * ADC_TO_CURR_SCALE - CURRENT_OFFSET;

  clarke_f32(current_sen[0], current_sen[1], current_sen[2], &i_ab_sen);
  park_f32(&i_ab_sen, &i_dq_sen, s, c);
  // 2) Calculate current error.
  // Convert mechanical q-axis reference to electrical frame using direction.
  i_ref.q = ref_i * pInst->direction;
  i_ref.d = 0.0f;

  i_error.d = i_ref.d - i_dq_sen.d; // d-axis current error.
  i_error.q = i_ref.q - i_dq_sen.q; // q-axis current error.

  // PI control.
  v_command.d = control_f32(&id_control, i_error.d);
  v_command.q = control_f32(&iq_control, i_error.q);

  // Cross-coupling compensation.
  v_cross.d =
      -pInst->omega_e * DEG_PER_SEC_TO_RAD_PER_SEC * MOTOR_Ls * i_dq_sen.q;
  v_cross.q =
      pInst->omega_e * DEG_PER_SEC_TO_RAD_PER_SEC * MOTOR_Ls * i_dq_sen.d;

  // Feedforward
  v_command.d = v_command.d + v_cross.d;
  v_command.q = v_command.q + v_cross.q; // TODO : + back-EMF term

  // 3) Inverse Park transform.
  park_inv_f32(&v_command, &v_SVM, s, c);

  // 4) Space vector modulation.
  SVM_f32(&v_SVM, V_DC, duty);
  convertDuty();

  // 5) Profiling.
  uint32_t t_end = htim2.Instance->CNT;
  // alpha_beta_t i_ab_ref;
  // arm_inv_park_f32(i_ref.d, i_ref.q, &i_ab_ref.alpha, &i_ab_ref.beta, s, c);
  // arm_inv_clarke_f32(i_ab_ref.alpha, i_ab_ref.beta, &i_ref_abc[0],
  //                    &i_ref_abc[1]);

  process_time = t_end - t_start; // FOC compute time.
  period_time = t_end - buf_time; // Loop period.
  buf_time = t_end;
}

void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef *hhrtim,
                                       uint32_t TimerIdx) {
  // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0); // Toggle LED for profiling.

  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = data[0];
  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = data[1];
  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR = data[2];

  (void)hhrtim;
  (void)TimerIdx;
  // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0); // Toggle LED for profiling.
}

void flux_control(float32_t deg) {
  float32_t sin_flux, cos_flux;
  arm_sin_cos_f32(deg, &sin_flux, &cos_flux);

  dq_t vdq_comm = {4.0f, 0.0f};

  if (MOTOR_I_RATED * MOTOR_Rs < 4.0f) {
      vdq_comm.d = MOTOR_I_RATED * MOTOR_Rs;
  }
  park_inv_f32(&vdq_comm, &v_SVM, sin_flux, cos_flux);

  // Space vector modulation.
  SVM_f32(&v_SVM, V_DC, duty);
  convertDuty();

  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = data[0];
  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = data[1];
  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR = data[2];
}
