#pragma once
#include "math.h"
#include "basic_function.hpp"

// Reciprocal of pi.
constexpr float32_t _1_PI = 1.0f / PI;

/**
 * @brief Normalized sinc function.
 * @param omega Normalized cutoff (rad/sample).
 * @param index Sample index.
 * @return sinc(omega * index) scaled by omega/pi.
 */
inline float32_t sinc_f32(float32_t omega, int index)
{
    float32_t angle = (float32_t)omega * index;
    if (index == 0)
        return (float32_t)omega * _1_PI;
    else
        return (float32_t)omega * _1_PI * sinf(angle) / angle;
}

/**
 * @brief Generate a Hamming window.
 * @param w Output array of length block_size.
 * @param block_size Number of taps.
 */
inline void hamming_f32(float32_t *w, uint32_t block_size)
{
    if (!w || block_size == 0)
        return;
    if (block_size == 1)
    {
        w[0] = 1.0f;
        return;
    }

    for (uint32_t n = 0; n < block_size; ++n)
        w[n] = 0.54f - 0.46f * cosf(2.0f * PI * n / (block_size - 1));
}

// void lpf_windowed_sinc(float32_t *coef, uint32_t N, float32_t wc) // rad/sample, 0 < wc < pi
// {
//     float32_t mid = (N - 1) * 0.5f;
//     float32_t acc = 0.0f;
//     for (uint32_t n = 0; n < N; ++n)
//     {
//         float32_t x = (float32_t)n - mid;
//         float32_t sinc = (x == 0.0f) ? wc / PI : sinf(wc * x) / (PI * x);
//         coef[n] = sinc;
//     }
//     float32_t coef_window[N];
//     hamming_f32(coef_window, N);
//     for (uint32_t n = 0; n < N; ++n)
//     {
//         float32_t x = (float32_t)n - mid;
//         coef[n] *= 0.54f - 0.46f * cosf(2.0f * PI * x / (N - 1));
//         acc += coef[n];
//     }

//     for (uint32_t n = 0; n < N; ++n)
//         coef[n] /= acc;
// }

/**
 * @brief Apply a windowed sinc low-pass filter kernel.
 * @param pSrc_window Window values (length block_size).
 * @param pDest Output coefficients (length block_size).
 * @param cutoff_freq Cutoff frequency in rad/sample (0, pi).
 * @param block_size Number of taps (must be odd).
 */
inline void LPF_f32(float32_t *pSrc_window, float32_t *pDest, float32_t cutoff_freq, uint32_t block_size)
{
    if (!pSrc_window || !pDest || block_size == 0)
        return;
    if (!(cutoff_freq > 0.0f && cutoff_freq < PI))
        return;
    if ((block_size & 1U) == 0U)
        return;
    const int mid = static_cast<int>((block_size - 1U) / 2U);
    for (uint32_t n = 0; n < block_size; ++n)
    {
        const int k = static_cast<int>(n) - mid;
        pDest[n] = pSrc_window[n] * sinc_f32(cutoff_freq, k);
    }
}

/**
 * @brief Apply a zero-phase non-causal FIR filter using circular indexing.
 * @param pSrc Input signal.
 * @param pDest Output signal.
 * @param filter FIR coefficients.
 * @param Data_length Number of samples in the input/output.
 * @param block_size Number of filter taps.
 */
inline bool apply_filter_non_causal_f32_noalloc(float32_t *pSrc,
                                                float32_t *pDest,
                                                float32_t *filter,
                                                uint32_t Data_length,
                                                uint32_t block_size,
                                                float32_t *zero_phase_filter,
                                                uint32_t zero_phase_len)
{
    if (block_size == 0 || Data_length == 0)
        return false;
    if (!pSrc || !pDest || !filter || !zero_phase_filter)
        return false;

    float32_t sum_coeff = 0;
    for (uint32_t index_filter = 0; index_filter < block_size; ++index_filter)
        sum_coeff += filter[index_filter];

    if (fabsf(sum_coeff) < 1e-12f)
        return false;

    for (uint32_t index_filter = 0; index_filter < block_size; ++index_filter)
        filter[index_filter] = filter[index_filter] / sum_coeff;

    const uint32_t Lz = 2U * block_size - 1U;
    if (zero_phase_len < Lz)
        return false;

    for (uint32_t i = 0; i < Lz; ++i)
        zero_phase_filter[i] = 0.0f;

    for (uint32_t n = 0; n < block_size; ++n)
    {
        for (uint32_t k = 0; k < block_size; ++k)
        {
            uint32_t idx = Index_circular(static_cast<int>(n) - static_cast<int>(k), Lz);
            zero_phase_filter[idx] += filter[k] * filter[n];
        }
    }

    for (uint32_t i = 0; i < Data_length; ++i)
    {
        float32_t acc = 0.0f;
        for (int m = -static_cast<int>(block_size) + 1; m < static_cast<int>(block_size); ++m)
        {
            uint32_t xi = Index_circular(static_cast<int>(i) - m, Data_length);
            uint32_t hm = Index_circular(m, Lz);
            acc += pSrc[xi] * zero_phase_filter[hm];
        }
        pDest[i] = acc;
    }
    return true;
}

inline void apply_filter_non_causal_f32(float32_t *pSrc, float32_t *pDest, float32_t *filter, uint32_t Data_length, uint32_t block_size)
{
    if (block_size == 0 || Data_length == 0)
        return;

    const uint32_t Lz = 2U * block_size - 1U;
    float32_t *zero_phase_filter =
        static_cast<float32_t *>(calloc(Lz, sizeof(float32_t)));

    if (!zero_phase_filter)
        return;

    (void)apply_filter_non_causal_f32_noalloc(pSrc, pDest, filter, Data_length,
                                              block_size, zero_phase_filter,
                                              Lz);
    free(zero_phase_filter);
}
