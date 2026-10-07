#include "encoder_calibration.hpp"
#include "math_tools.hpp"
#include "constants.hpp"
#include <cstdlib> // for calloc/free

// Anonymous namespace for helper functions
namespace
{

    // Sentinel outside the valid offset range [-180, 180].
    static constexpr float32_t OFFSET_EMPTY_SENTINEL = 1000.0f;

    float32_t wrap_deg(float32_t a)
    {
        while (a > 180.0f)
            a -= 360.0f;
        while (a < -180.0f)
            a += 360.0f;
        return a;
    }

    // Circular mean between a and b (deg) using the shortest path.
    inline float32_t circ_avg2(float32_t a, float32_t b)
    {
        return wrap_deg(a + 0.5f * wrap_deg(b - a));
    }

} // namespace

EncoderCalibrator_t::EncoderCalibrator_t(MA732_t &encoder,
                                         MotorControlFunc_t motor_control_func)
    : encoder_(encoder), motor_control_func_(motor_control_func) {}

void EncoderCalibrator_t::set_offset_buffer(float32_t *buffer,
                                            uint32_t num_samples)
{
    external_buffer_ = buffer;
    external_len_ = num_samples;
}

void EncoderCalibrator_t::run(uint8_t pole_pairs, uint32_t num_samples,
                              uint32_t window_size)
{
    if (owns_offset_data_ && offset_data_)
        free(offset_data_);
    offset_data_ = nullptr;
    owns_offset_data_ = false;

    if (num_samples == 0)
        return;

    if (external_buffer_ && external_len_ > 0)
    {
        if (num_samples > external_len_)
            num_samples = external_len_;
        offset_data_ = external_buffer_;
    }
    else
    {
        offset_data_ = static_cast<float32_t *>(calloc(num_samples, sizeof(float32_t)));
        if (!offset_data_)
            return;
        owns_offset_data_ = true;
    }

    // calloc initializes to 0; use a sentinel to avoid "0 = unmeasured".
    for (uint32_t i = 0; i < num_samples; ++i)
        offset_data_[i] = OFFSET_EMPTY_SENTINEL;

    num_samples_ = num_samples;

    // Detect wiring direction before building the offset table.
    direction_ = detect_direction(pole_pairs, num_samples);

    // 1) Forward sweep: accumulate into the buffer.
    measure_and_merge(pole_pairs, num_samples, /*dir_sign=*/+1);

    // 2) Reverse sweep: merge using circular mean.
    measure_and_merge(pole_pairs, num_samples, /*dir_sign=*/-1);

    interpolate_data(num_samples);
    filter_data(num_samples, window_size); // Filter as configured.
}

bool EncoderCalibrator_t::save_to_flash()
{
    if (!offset_data_)
        return false;

    uint32_t flash_status = 0;

    // 1) Write the offset table first (separate flash region).
    flash_status |= write_flash_buffer(
        ENCODER_BUFFER_BASE_ADDR,
        reinterpret_cast<uint8_t *>(offset_data_),
        num_samples_,
        sizeof(float32_t));

    // 2) Write the state block (16 bytes):
    //    [0] direction (float32)
    //    [1] buffer size
    //    [2] reserved filler
    //    [3] MA732 calibration marker
    struct
    {
        float32_t direction;
        uint32_t buffer_size;
        uint32_t reserved;
        uint32_t flash_update;
    } __attribute__((packed, aligned(8))) state;

    state.direction = direction_;
    state.buffer_size = static_cast<uint32_t>(num_samples_); // ENCODER_BUFFER_SIZE_OFFSET
    state.reserved = 0xFFFFFFFFu;                            // Reserved filler.
    state.flash_update = ENCODER_CALIBRATION_MARKER;         // ENCODER_FLASH_UPDATE_OFFSET

    // Program 4 words (16 bytes) starting at ENCODER_STATE_ADDR.
    flash_status |= write_flash_buffer(
        ENCODER_STATE_ADDR,
        reinterpret_cast<uint8_t *>(&state),
        /*length=*/4,
        /*size=*/sizeof(uint32_t));

    return flash_status == HAL_OK;
}

bool EncoderCalibrator_t::load_from_flash()
{
    if (owns_offset_data_ && offset_data_)
        free(offset_data_);
    offset_data_ = nullptr;
    owns_offset_data_ = false;

    uint32_t load_samples = NUM_SAMPLES;
    if (external_buffer_ && external_len_ > 0)
    {
        if (load_samples > external_len_)
            load_samples = external_len_;
        offset_data_ = external_buffer_;
    }
    else
    {
        offset_data_ = static_cast<float32_t *>(calloc(load_samples, sizeof(float32_t)));
        if (!offset_data_)
            return false;
        owns_offset_data_ = true;
    }

    num_samples_ = load_samples;

    uint32_t flash_status = 0;
    flash_status |= read_flash_buffer_f32(ENCODER_BUFFER_BASE_ADDR, offset_data_, num_samples_, sizeof(float32_t));
    flash_status |= read_flash_buffer_f32(ENCODER_STATE_ADDR + ENCODER_DIRECTION_OFFSET * 4, &direction_, 1, sizeof(float32_t));

    return flash_status == HAL_OK;
}

void EncoderCalibrator_t::get_offset_table(float32_t *offset_table,
                                           uint32_t num_samples)
{
    if (offset_data_ && offset_table && num_samples <= num_samples_)
    {
        if (offset_table != offset_data_)
            memcpy(offset_table, offset_data_, num_samples * sizeof(float32_t));
    }
}

float32_t EncoderCalibrator_t::get_direction() const
{
    return direction_;
}

float32_t EncoderCalibrator_t::detect_direction(uint8_t pole_pairs,
                                                uint32_t num_samples)
{
    (void)pole_pairs;
    (void)num_samples;

    const float32_t elec_span = 360.0f; // One electrical revolution.
    const uint32_t steps = 60U;
    const float32_t step = elec_span / static_cast<float32_t>(steps);

    motor_control_func_(0.0f);
    HAL_Delay(1000);

    float32_t angle_prev = encoder_.readAngleDeg();
    float32_t angle_accum = 0.0f;

    for (uint32_t i = 1; i <= steps; ++i)
    {
        const float32_t target_elec = i * step;
        motor_control_func_(target_elec);
        HAL_Delay(5);

        float32_t angle = encoder_.readAngleDeg();
        angle_accum += wrap_deg(angle - angle_prev);
        angle_prev = angle;
    }

    motor_control_func_(0.0f);
    HAL_Delay(200);

    if (angle_accum > -3.0f && angle_accum < 3.0f)
        return 1.0f;

    if (angle_accum >= 0.0f)
        return 1.0f;
    return -1.0f;
}

void EncoderCalibrator_t::measure_and_merge(uint8_t pole_pairs,
                                            uint32_t num_samples,
                                            int8_t dir_sign)
{
    const float32_t resolution = 360.0f * pole_pairs / static_cast<float32_t>(num_samples);

    // Hold the start position.
    motor_control_func_(0.0f);
    HAL_Delay(1000);

    int32_t last_idx = -1; // Avoid consecutive duplicate indices.

    for (uint32_t i = 0; i < num_samples; ++i)
    {
        // dir_sign = +1 forward, -1 reverse.
        const float32_t target_elec = dir_sign * (i * resolution);
        motor_control_func_(target_elec);
        HAL_Delay(2);

        float32_t angle = encoder_.readAngleDeg();
        while (angle >= 360.0f)
            angle -= 360.0f;
        while (angle < 0.0f)
            angle += 360.0f;

        uint32_t idx = static_cast<uint32_t>(floorf(angle * static_cast<float32_t>(num_samples) / 360.0f));
        if (idx >= num_samples)
            idx = 0;

        if (static_cast<int32_t>(idx) != last_idx) // Skip consecutive duplicates.
        {
            // Measured electrical offset = (mech angle * pole pairs) - direction*command.
            float32_t meas = wrap_deg(angle * pole_pairs - direction_ * target_elec);

            if (offset_data_[idx] == OFFSET_EMPTY_SENTINEL)
            {
                // First contribution (either direction).
                offset_data_[idx] = meas;
            }
            else
            {
                // Merge with existing value using circular mean.
                offset_data_[idx] = circ_avg2(offset_data_[idx], meas);
            }
        }

        last_idx = static_cast<int32_t>(idx);
    }
}

void EncoderCalibrator_t::measure_raw_data(uint8_t pole_pairs,
                                           uint32_t num_samples)
{
    float32_t angle_prev = encoder_.readAngleDeg();
    float32_t angle_accum = 0.0f;

    const float32_t resolution = 360.0f * pole_pairs / static_cast<float32_t>(num_samples);

    motor_control_func_(0.0f);
    HAL_Delay(1000);

    for (uint32_t i = 0; i < num_samples; ++i)
    {
        motor_control_func_(i * resolution);
        HAL_Delay(2);

        float32_t angle = encoder_.readAngleDeg();
        // Normalize mechanical angle to [0, 360).
        while (angle >= 360.0f)
            angle -= 360.0f;
        while (angle < 0.0f)
            angle += 360.0f;

        // Map mechanical angle to table index.
        uint32_t idx = static_cast<uint32_t>(floorf(angle * static_cast<float32_t>(num_samples) / 360.0f));
        if (idx >= num_samples)
            idx = 0; // Guard against 360.0.

        // Only write when the slot is empty.
        if (offset_data_[idx] == OFFSET_EMPTY_SENTINEL)
        {
            offset_data_[idx] = wrap_deg(angle * pole_pairs - i * resolution);
        }

        angle_accum += wrap_deg(angle - angle_prev);
        angle_prev = angle;
    }
    direction_ = (angle_accum >= 0.0f) ? 1.0f : -1.0f;
}

void EncoderCalibrator_t::interpolate_data(uint32_t n)
{
    if (!offset_data_ || n == 0)
        return;

    // Find the first valid sample.
    uint32_t start = 0;
    bool has_data = false;
    for (; start < n; ++start)
    {
        if (offset_data_[start] != OFFSET_EMPTY_SENTINEL)
        {
            has_data = true;
            break;
        }
    }
    if (!has_data)
        return; // All empty: nothing to do.

    auto next_idx = [n](uint32_t i)
    { return (i + 1 < n) ? (i + 1) : 0U; };

    uint32_t prev = start;
    uint32_t i = next_idx(start);

    while (i != start)
    {
        if (offset_data_[i] != OFFSET_EMPTY_SENTINEL)
        {
            prev = i;
            i = next_idx(i);
            continue;
        }

        uint32_t len = 0;
        while (offset_data_[i] == OFFSET_EMPTY_SENTINEL)
        {
            ++len;
            i = next_idx(i);
            if (i == start)
                break; // Guard against full wrap.
        }
        uint32_t next = i; // i is next valid sample (or start).

        // If next == prev, only one valid sample exists; cannot interpolate.
        if (next == prev)
            break;

        float32_t v_prev = offset_data_[prev];
        float32_t v_next = offset_data_[next];

        // Wrap angle delta to the shortest path.
        float32_t delta = wrap_deg(v_next - v_prev);

        for (uint32_t k = 1; k <= len; ++k)
        {
            uint32_t idx = (prev + k) % n;
            float32_t t = static_cast<float32_t>(k) / static_cast<float32_t>(len + 1);
            offset_data_[idx] = wrap_deg(v_prev + delta * t);
        }

        prev = next;
        if (i == start)
            break;
    }
}

void EncoderCalibrator_t::filter_data(uint32_t num_samples,
                                      uint32_t window_size)
{
    if (!offset_data_ || num_samples == 0 || window_size == 0)
        return;

    // Sanitize window length (limit to data length, enforce odd).
    if (window_size > num_samples)
        window_size = num_samples;
    if ((window_size & 1U) == 0U)
        window_size -= 1U; // Force odd length.
    if (window_size == 0)
        return;

    float32_t *window = static_cast<float32_t *>(calloc(window_size, sizeof(float32_t)));
    float32_t *filter_coeff = static_cast<float32_t *>(calloc(window_size, sizeof(float32_t)));
    if (!window || !filter_coeff)
    {
        if (window)
            free(window);
        if (filter_coeff)
            free(filter_coeff);
        return;
    }

    // Build filter coefficients.
    hamming_f32(window, window_size);
    LPF_f32(window, filter_coeff, 1.0f / 90.09f, window_size);

    // Zero-phase circular convolution is not in-place; use a temp buffer.
    float32_t *temp_filtered_data =
        static_cast<float32_t *>(malloc(num_samples * sizeof(float32_t)));
    if (!temp_filtered_data)
    {
        free(filter_coeff);
        free(window);
        return;
    }

    apply_filter_non_causal_f32(offset_data_, temp_filtered_data,
                                filter_coeff, num_samples, window_size);

    // Copy result back.
    memcpy(offset_data_, temp_filtered_data, num_samples * sizeof(float32_t));

    free(temp_filtered_data);
    free(filter_coeff);
    free(window);
}
