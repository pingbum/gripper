#ifndef ENCODER_CALIBRATION_HPP
#define ENCODER_CALIBRATION_HPP

#include "MA732.hpp"
#include "arm_math.h"
#include "memory_constants.hpp"
#include "flash_memory.hpp"
#include <functional>

// Forward declaration to avoid circular dependency.
void flux_control(float32_t deg);

class EncoderCalibrator_t
{
public:
    using MotorControlFunc_t = std::function<void(float32_t)>;

    EncoderCalibrator_t(MA732_t &encoder,
                        MotorControlFunc_t motor_control_func);

    /**
     * @brief Provide an external buffer to store the offset table.
     * @param buffer Destination buffer.
     * @param num_samples Length of the buffer.
     */
    void set_offset_buffer(float32_t *buffer, uint32_t num_samples);

    /**
     * @brief Runs the full encoder calibration sequence.
     * @param pole_pairs Number of motor pole pairs.
     * @param num_samples Number of samples to collect for the offset table.
     * @param window_size Size of the filter window.
     */
    void run(uint8_t pole_pairs, uint32_t num_samples, uint32_t window_size);

    /**
     * @brief Saves the calibrated data to flash memory.
     * @return True on success, false on failure.
     */
    bool save_to_flash();

    /**
     * @brief Loads calibration data from flash memory.
     * @return True on success, false on failure.
     */
    bool load_from_flash();

    /**
     * @brief Gets the calibrated offset table.
     * @param offset_table Pointer to the destination array.
     * @param num_samples The size of the destination array.
     */
    void get_offset_table(float32_t *offset_table, uint32_t num_samples);

    /**
     * @brief Gets the detected motor rotation direction.
     * @return 1.0f for normal, -1.0f for reversed.
     */
    float32_t get_direction() const;

private:
    /**
     * @brief Measures the raw encoder values by rotating the motor.
     */
    void measure_raw_data(uint8_t pole_pairs, uint32_t num_samples);

    /**
     * @brief Fills in any gaps in the measured data using linear interpolation.
     */
    void interpolate_data(uint32_t num_samples);

    /**
     * @brief Applies a non-causal zero-phase filter to smooth the offset data.
     */
    void filter_data(uint32_t num_samples, uint32_t window_size);

    /**
     * @brief Measure and merge a sweep into the offset table.
     * @param pole_pairs Number of motor pole pairs.
     * @param num_samples Number of samples in the table.
     * @param dir_sign +1 for forward sweep, -1 for reverse.
     */
    void measure_and_merge(uint8_t pole_pairs, uint32_t num_samples, int8_t dir_sign);

    /**
     * @brief Detect the electrical-to-mechanical direction sign.
     */
    float32_t detect_direction(uint8_t pole_pairs, uint32_t num_samples);

    MA732_t &encoder_;
    MotorControlFunc_t motor_control_func_;

    float32_t *offset_data_ = nullptr;
    float32_t *external_buffer_ = nullptr;
    uint32_t external_len_ = 0;
    bool owns_offset_data_ = false;
    float32_t direction_ = 1.0f;
    uint32_t num_samples_ = 0;
};

#endif // ENCODER_CALIBRATION_HPP
