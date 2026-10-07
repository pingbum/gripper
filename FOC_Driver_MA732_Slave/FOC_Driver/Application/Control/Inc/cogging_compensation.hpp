#pragma once

#include "arm_math_types.h"
#include <cstdint>

/**
 * @brief Learns and compensates position-dependent cogging torque.
 *
 * The table stores an equivalent mechanical q-axis current command [A] over
 * one mechanical revolution.  To reject Coulomb friction, collect data at the
 * same low, constant speed in both directions:
 *
 *   record(theta_deg, iq_ref, +1);  // forward revolution
 *   record(theta_deg, iq_ref, -1);  // reverse revolution
 *
 * After both sweeps, call finalize().  compensation() then returns a linearly
 * interpolated feed-forward current that can be added to the current reference.
 * Measurement must be performed after the velocity has settled; acceleration,
 * load torque, and speed-loop transients otherwise contaminate the table.
 */
class CoggingCompensation {
public:
  // 0.25 mechanical-degree resolution. This is sufficient to represent the
  // theoretical 264 cycles/revolution cogging component of a 22-pole,
  // 24-slot motor without spatial aliasing.
  static constexpr uint16_t BIN_COUNT = 1440U;

  void reset() {
    for (uint16_t i = 0; i < BIN_COUNT; ++i) {
      forward_sum_[i] = 0.0f;
      reverse_sum_[i] = 0.0f;
      table_[i] = 0.0f;
      forward_count_[i] = 0U;
      reverse_count_[i] = 0U;
    }
    ready_ = false;
  }

  /**
   * @param theta_deg Mechanical motor-shaft angle in degrees.
   * @param iq_ref Mechanical-sign q-axis current required at constant speed.
   * @param direction Positive for forward sweep, negative for reverse sweep.
   */
  void record(float32_t theta_deg, float32_t iq_ref, int8_t direction) {
    const uint16_t bin = angle_to_bin(theta_deg);
    if (direction > 0) {
      if (forward_count_[bin] != UINT16_MAX) {
        forward_sum_[bin] += iq_ref;
        ++forward_count_[bin];
      }
    } else if (direction < 0) {
      if (reverse_count_[bin] != UINT16_MAX) {
        reverse_sum_[bin] += iq_ref;
        ++reverse_count_[bin];
      }
    }
    ready_ = false;
  }

  /**
   * @brief Builds the compensation LUT from the forward/reverse measurements.
   * @param max_abs_current Safety clamp for the learned feed-forward current.
   * @return true only when every bin contains samples from both directions.
   */
  bool finalize(float32_t max_abs_current) {
    if (!(max_abs_current > 0.0f))
      return false;

    uint16_t valid_count = 0U;
    for (uint16_t i = 0; i < BIN_COUNT; ++i) {
      valid_[i] = forward_count_[i] != 0U && reverse_count_[i] != 0U;
      if (valid_[i]) {
        const float32_t forward =
            forward_sum_[i] / static_cast<float32_t>(forward_count_[i]);
        const float32_t reverse =
            reverse_sum_[i] / static_cast<float32_t>(reverse_count_[i]);
        // Direction-independent current is the cogging compensation. Direction-
        // dependent Coulomb friction cancels in this average.
        table_[i] = 0.5f * (forward + reverse);
        ++valid_count;
      }
    }

    if (valid_count == 0U) {
      ready_ = false;
      return false;
    }

    // At low speed an angle bin can occasionally be skipped. Fill missing bins
    // by circular linear interpolation instead of rejecting the whole sweep.
    for (uint16_t i = 0; i < BIN_COUNT; ++i) {
      if (valid_[i])
        continue;
      uint16_t prev_distance = 1U;
      while (prev_distance < BIN_COUNT &&
             !valid_[(i + BIN_COUNT - prev_distance) % BIN_COUNT])
        ++prev_distance;
      uint16_t next_distance = 1U;
      while (next_distance < BIN_COUNT &&
             !valid_[(i + next_distance) % BIN_COUNT])
        ++next_distance;
      const uint16_t prev = (i + BIN_COUNT - prev_distance) % BIN_COUNT;
      const uint16_t next = (i + next_distance) % BIN_COUNT;
      const float32_t fraction = static_cast<float32_t>(prev_distance) /
          static_cast<float32_t>(prev_distance + next_distance);
      table_[i] = table_[prev] + fraction * (table_[next] - table_[prev]);
    }

    // Remove constant load/bias; this module compensates only periodic cogging.
    float32_t dc = 0.0f;
    for (uint16_t i = 0; i < BIN_COUNT; ++i)
      dc += table_[i];
    dc /= static_cast<float32_t>(BIN_COUNT);
    for (uint16_t i = 0; i < BIN_COUNT; ++i)
      table_[i] = clamp(table_[i] - dc, max_abs_current);

    // One circular three-point smoothing pass reduces ADC/velocity-loop noise.
    for (uint16_t i = 0; i < BIN_COUNT; ++i) {
      const uint16_t prev = (i == 0U) ? BIN_COUNT - 1U : i - 1U;
      const uint16_t next = (i + 1U == BIN_COUNT) ? 0U : i + 1U;
      filtered_[i] =
          0.25f * table_[prev] + 0.5f * table_[i] + 0.25f * table_[next];
    }
    for (uint16_t i = 0; i < BIN_COUNT; ++i)
      table_[i] = clamp(filtered_[i], max_abs_current);

    ready_ = true;
    return true;
  }

  /** @return Mechanical-sign q-axis feed-forward current [A]. */
  float32_t compensation(float32_t theta_deg) const {
    if (!ready_)
      return 0.0f;

    const float32_t wrapped = wrap_angle(theta_deg);
    const float32_t position =
        wrapped * (static_cast<float32_t>(BIN_COUNT) / 360.0f);
    const uint16_t first = static_cast<uint16_t>(position);
    const uint16_t second = (first + 1U == BIN_COUNT) ? 0U : first + 1U;
    const float32_t fraction = position - static_cast<float32_t>(first);
    return table_[first] + fraction * (table_[second] - table_[first]);
  }

  bool ready() const { return ready_; }

  /** Load a previously validated 1440-value compensation table. */
  bool load_table(const float32_t *values, uint16_t count) {
    if (!values || count != BIN_COUNT) {
      ready_ = false;
      return false;
    }
    for (uint16_t i = 0; i < BIN_COUNT; ++i)
      table_[i] = values[i];
    ready_ = true;
    return true;
  }

  const float32_t *table_data() const { return table_; }

  float32_t table_value(uint16_t bin) const {
    return (bin < BIN_COUNT) ? table_[bin] : 0.0f;
  }

private:
  static float32_t wrap_angle(float32_t angle) {
    while (angle >= 360.0f)
      angle -= 360.0f;
    while (angle < 0.0f)
      angle += 360.0f;
    return angle;
  }

  static uint16_t angle_to_bin(float32_t angle) {
    const float32_t wrapped = wrap_angle(angle);
    uint16_t bin = static_cast<uint16_t>(
        wrapped * (static_cast<float32_t>(BIN_COUNT) / 360.0f));
    if (bin >= BIN_COUNT)
      bin = BIN_COUNT - 1U;
    return bin;
  }

  static float32_t clamp(float32_t value, float32_t magnitude) {
    if (value > magnitude)
      return magnitude;
    if (value < -magnitude)
      return -magnitude;
    return value;
  }

  float32_t forward_sum_[BIN_COUNT]{};
  float32_t reverse_sum_[BIN_COUNT]{};
  float32_t table_[BIN_COUNT]{};
  float32_t filtered_[BIN_COUNT]{};
  uint16_t forward_count_[BIN_COUNT]{};
  uint16_t reverse_count_[BIN_COUNT]{};
  bool valid_[BIN_COUNT]{};
  bool ready_ = false;
};

// Automated measurement sequence. start() uses mechanical RPM and performs
// one settling revolution plus three recorded revolutions in each direction.
bool cogging_measurement_start(float32_t mechanical_rpm);
void cogging_measurement_stop();
bool cogging_measurement_active();
void cogging_compensation_set_enabled(bool enabled);
bool cogging_compensation_enabled();
float32_t cogging_compensation_current(float32_t mechanical_angle_deg);
float32_t cogging_table_value(uint16_t bin);

// Debugger-readable mirror of the finalized LUT. This avoids function-call
// evaluation, which many embedded GDB sessions cannot perform safely.
extern volatile float32_t cogging_lut_debug[CoggingCompensation::BIN_COUNT];
