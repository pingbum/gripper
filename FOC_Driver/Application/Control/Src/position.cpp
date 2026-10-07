#include "position.hpp"
#include "arm_math.h"
#include <cmath>

// Electrical offset of the slave relative to the master [deg, electrical].
// Positive value leads the master. Editable at runtime via debugger.
volatile float32_t slave_elec_offset_deg = -122.7f;

namespace
{
    static inline float32_t wrap_0_360_f32(float32_t x)
    {
        float32_t temp;
        if (x > 360.0f)
            temp = x - 360;
        else if (x < 0.0f)
            temp = x + 360.0f;
        else
            temp = x;
        return temp;
    }

    static inline void wrap_m360_360_f32(float32_t *x)
    {
        if (*x > 180.0f)
            *x -= 360.0f;
        else if (*x < -180.0f)
            *x += 360.0f;
        return;
    }
}

void initializePosition(position_instance_f32_t *s, uint8_t polepairs,
                        MA732_t &encoder)
{
    // Slave uses the master encoder angle plus slave_elec_offset_deg only.
    // Encoder calibration (open-loop sweep) and the flash LUT are skipped.
    (void)encoder;
    for (uint32_t i = 0; i < NUM_SAMPLES; ++i)
        s->theta_e_offset[i] = 0.0f;
    // Slave phases are wired A-C-B relative to the master (A-B-C), so the
    // electrical angle rotates opposite to the master's.
    s->direction = -1.0f;
    s->p = static_cast<float32_t>(polepairs);
}

void updatePositionMech(position_instance_f32_t *s,
                        float32_t angle_deg,
                        float32_t dt,
                        uint32_t numSamples)
{
    float32_t angle_signed = angle_deg;
    float32_t delta = angle_signed - s->theta_m_prev;
    wrap_m360_360_f32(&delta);


    float32_t raw_diff = angle_signed - s->theta_m_prev;
    if (raw_diff - delta > 180.0f) {
        s->rev--; // 360 -> 0 방향 (정회전)
    } else if (raw_diff - delta < -180.0f) {
        s->rev++; // 0 -> 360 방향 (역회전)
    }

    float32_t omega_inst = delta / dt; // [deg/s]
    
    // First-order LPF: alpha = exp(-dt / tau).
    const float32_t tau = 0.01f;
    float32_t alpha = expf(-dt / tau);
    s->omega_m = alpha * s->omega_m + (1.0f - alpha) * omega_inst;

    // s->omega_m = delta / dt;
    s->theta_m_prev = angle_signed;
    s->theta_m = wrap_0_360_f32(angle_signed);

    // Slave commutation: master encoder angle + constant electrical offset.
    // The calibration LUT is not used.
    (void)numSamples;
    s->theta_e_save =
        s->direction * (s->p * s->theta_m) + slave_elec_offset_deg;
    s->theta_e = s->theta_e_save;
    s->omega_e = s->direction * s->p * s->omega_m;
}
