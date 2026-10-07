#pragma once
#include "arm_math_types.h"

// Global timing.
constexpr float32_t DT = 1 / (170000000.0f);

// Control modes.
constexpr uint32_t MODE_CURRENT = 0x01;  // Current control mode.
constexpr uint32_t MODE_VELOCITY = 0x03; // Velocity control mode.
constexpr uint32_t MODE_POSITION = 0x05; // Position control mode.

//Gear Ratio
constexpr float32_t GEAR_RATIO = 1.00f;

// Configuration modes.
constexpr uint32_t MODE_SYSTEM_RESET = 0x00;
constexpr uint32_t MODE_CALIBRATION = 0x06; // Calibration mode.
constexpr uint32_t MODE_SET_CAN_ID = 0x07;  // Set CAN ID.
constexpr uint32_t MODE_SAVE_CTRL_PARAMS = 0x10;
constexpr uint32_t MODE_SET_CURRENT_BANDWITDH = 0x11;
constexpr uint32_t MODE_SET_VELOCITY_KP = 0x20;
constexpr uint32_t MODE_SET_VELOCITY_KI = 0x21;
constexpr uint32_t MODE_COGGING_COMPENSATION = 0x30;
constexpr uint32_t MODE_COGGING_TOGGLE = 0x31;
constexpr uint32_t MODE_CAN_BROADCAST_TOGGLE = 0xF0;
constexpr uint32_t MODE_CAN_BROADCAST_RATE = 0xF1;

// Encoder calibration.
constexpr uint32_t NUM_SAMPLES = 10000;   // LUT size.
constexpr uint32_t WINDOW_SIZE = 256 + 1; // FIR window size.
extern uint32_t FLASH_UPDATE;             // Calibration state flag.

// Current loop ADC buffer.
#define ADC_BUF_SIZE 3

constexpr float32_t ADC_GAIN = 0.6f; // Current sense gain [V/A].

// Motor parameters.
// constexpr float32_t MOTOR_POLE_PAIRS = 7;
constexpr float32_t MOTOR_POLE_PAIRS = 11;

// constexpr float32_t MOTOR_Ls = 0.000053f; // Series inductance [H].
constexpr float32_t MOTOR_Ls = 0.000150f; // Series inductance [H].

// constexpr float32_t MOTOR_Rs = 2.82;        // Series resistance [ohm].
constexpr float32_t MOTOR_Rs = 2.8;        // Series resistance [ohm].

// constexpr float32_t MOTOR_I_RATED = 0.66; // For Encoder Calibration [A].
constexpr float32_t MOTOR_I_RATED = 1.2; // For Encoder Calibration [A].
constexpr float32_t V_DC = 12;              // DC link voltage [V].

// Controller limits.
constexpr float32_t VOLTAGE_MAX = V_DC; // Max voltage command [V].
constexpr float32_t CURRENT_MAX = 0.8f; // Max current magnitude [A]..

// Conversions.
constexpr float32_t RPM_TO_DEG_PER_SEC = 6.0f;
constexpr float32_t DEG_PER_SEC_TO_RPM = 0.1666666666666666f;
constexpr float32_t DEG_PER_SEC_TO_RAD_PER_SEC = 0.017453292519943295f;
/**
 * @brief Mechanical and electrical position state.
 */
typedef struct {
  float32_t theta_m;      // Mechanical angle [deg].
  float32_t omega_m;      // Mechanical speed [deg/s].
  float32_t theta_m_prev; // Previous mechanical angle [deg].

  int32_t rev;          // Revolute

  float32_t theta_e;      // Electrical angle [deg].
  float32_t theta_e_save; // Electrical angle with offset applied [deg].
  float32_t omega_e;      // Electrical speed [deg/s].
  float32_t theta_e_offset[NUM_SAMPLES]; // Offset LUT [deg].

  float32_t direction; // +1 or -1, rotation direction.
  float32_t p;         // Pole pairs as float.
} position_instance_f32_t;

/**
 * @brief Aggregated error flags.
 */
typedef struct {
  bool can_error;
  bool spi_error;
  bool driver_fault;
} flags_t;

/**
 * @brief Global application state.
 */
struct State_t {
  float32_t theta_ref;  // Position reference.
  float32_t omega_ref;  // Speed reference.
  float32_t iq_ref;     // Current reference in mechanical sign.
  uint32_t temperature; // Temperature [C].
  position_instance_f32_t position;
  uint32_t CONTROL_MODE = 0;
  float32_t debug;
  flags_t error_flags;
};
