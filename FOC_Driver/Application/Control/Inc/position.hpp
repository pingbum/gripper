#ifndef POSITION_HPP
#define POSITION_HPP
#include "arm_math.h"
#include "MA732.hpp"
#include "constants.hpp"

#ifdef __cplusplus
extern "C" {
#endif

/** Default slave electrical offset used when flash holds no saved value. */
#define SLAVE_ELEC_OFFSET_DEFAULT_DEG (-82.7f)

/** Slave electrical offset relative to the master [deg]. */
extern volatile float32_t slave_elec_offset_deg;

/**
 * @brief Set the slave electrical offset (RAM only; persist with
 *        save_control_params_to_flash()).
 * @param offset_deg Offset in electrical degrees, accepted range [-360, 360].
 *        Stored normalized to [-180, 180].
 * @return True if the value is finite, in range and applied.
 */
bool slave_elec_offset_set_deg(float32_t offset_deg);

/**
 * @brief Get the slave electrical offset [deg, electrical].
 */
float32_t slave_elec_offset_get_deg(void);

/**
 * @brief Initialize position tracking and calibration.
 * @param s Position instance to initialize.
 * @param polepairs Number of motor pole pairs.
 * @param encoder Encoder instance.
 */
void initializePosition(position_instance_f32_t *s, uint8_t polepairs,
                        MA732_t &encoder);

/**
 * @brief Update mechanical position and speed.
 * @param s Position instance.
 * @param angle_deg Measured mechanical angle in degrees.
 * @param dt Sample period in seconds.
 * @param numSamples Size of the offset table.
 */
void updatePositionMech(position_instance_f32_t *s,
                        float32_t angle_deg,
                        float32_t dt,
                        uint32_t numSamples);

/**
 * @brief Update electrical position using stored offset.
 * @param s Position instance.
 * @param dt Sample period in seconds.
 */
__STATIC_FORCEINLINE void updatePositionElec(position_instance_f32_t *s,
                                             float32_t dt)
{
    s->theta_e = s->theta_e_save + (s->omega_e) * dt;
}

#ifdef __cplusplus
}
#endif
#endif // POSITION_HPP
