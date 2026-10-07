#include "position.hpp"
#include "encoder_calibration.hpp" // Use the new refactored calibration class
#include "current_loop.hpp"        // For flux_control
#include "arm_math.h"
#include "flash_memory.hpp"
#include "memory_constants.hpp"
#include <cmath>

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
    // The motor control function is passed to the calibrator to decouple modules.
    auto motor_control_func = [](float32_t angle)
    {
        flux_control(angle);
    };

    EncoderCalibrator_t calibrator(encoder, motor_control_func);
    calibrator.set_offset_buffer(s->theta_e_offset, NUM_SAMPLES);

    bool calibration_available = false;

    if (FLASH_UPDATE == ENCODER_CALIBRATION_MARKER)
    {
        calibration_available = calibrator.load_from_flash();
    }
    else if (FLASH_UPDATE == 0U)
    {
        // A zero marker is written only by the explicit CAN calibration
        // command. Blank or legacy flash data must not start calibration.
        calibrator.run(polepairs, NUM_SAMPLES, WINDOW_SIZE);
        calibration_available = true;
        if (calibrator.save_to_flash())
            FLASH_UPDATE = ENCODER_CALIBRATION_MARKER;
    }

    if (calibration_available)
    {
        calibrator.get_offset_table(s->theta_e_offset, NUM_SAMPLES);
        s->direction = calibrator.get_direction();
    }
    else
    {
        // No MA732 calibration data: use raw mechanical angle without an
        // electrical offset until calibration is explicitly requested.
        for (uint32_t i = 0; i < NUM_SAMPLES; ++i)
            s->theta_e_offset[i] = 0.0f;
        s->direction = 1.0f;
    }

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

    float32_t theta_e_raw = s->p * s->theta_m;
    uint32_t index = static_cast<uint32_t>(theta_e_raw / (360.0f * s->p / static_cast<float32_t>(numSamples)));
    if (index >= numSamples)
        index = numSamples - 1;

    s->theta_e_save =
        s->direction * (s->p * s->theta_m - s->theta_e_offset[index]);
    s->theta_e = s->theta_e_save;
    s->omega_e = s->direction * s->p * s->omega_m;
}
