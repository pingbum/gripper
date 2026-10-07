
#ifndef INC_CLAMP_FUNCTION_HPP_
#define INC_CLAMP_FUNCTION_HPP_

#include <arm_math.h>

/**
 * @brief Reinterpret a float as raw bits.
 */
__STATIC_FORCEINLINE uint32_t float_to_bits(float f)
{
    union
    {
        float f;
        uint32_t u;
    } fu;
    fu.f = f;
    return fu.u;
}
/**
 * @brief Reinterpret raw bits as a float.
 */
__STATIC_FORCEINLINE float32_t bits_to_float(uint32_t u)
{
    union
    {
        uint32_t u;
        float32_t f;
    } uf;
    uf.u = u;
    return uf.f;
}

/**
 * @brief Clamp a value to the provided bounds.
 * @param x Input value.
 * @param lower_bound Minimum allowed value.
 * @param upper_bound Maximum allowed value.
 * @return Clamped value.
 */
__STATIC_FORCEINLINE float32_t clamp_f32(float32_t x,
                                         float32_t lower_bound,
                                         float32_t upper_bound)
{

    uint32_t xi = float_to_bits(x);
    uint32_t li = float_to_bits(lower_bound);

    float32_t diff_low = x - lower_bound;
    uint32_t mask_low = (uint32_t)((int32_t)float_to_bits(diff_low) >> 31);

    xi = (xi & ~mask_low) | (li & mask_low);

    uint32_t ui = float_to_bits(upper_bound);

    float32_t diff_up = bits_to_float(xi) - upper_bound;
    uint32_t mask_up = (uint32_t)((int32_t)float_to_bits(diff_up) >> 31);

    uint32_t ri = (xi & mask_up) | (ui & ~mask_up);
    return bits_to_float(ri);
}

/**
 * @brief Clamp a value to the range [lo, hi].
 * @tparam T Arithmetic type.
 * @param v Input value.
 * @param lo Minimum allowed value.
 * @param hi Maximum allowed value.
 * @return v < lo ? lo : (v > hi ? hi : v).
 */
template <typename T>
__attribute__((always_inline)) inline constexpr T limit(const T v, const T lo, const T hi) noexcept
{
    static_assert(std::is_arithmetic<T>::value, "T must be arithmetic");
    return v < lo ? lo : (v > hi ? hi : v);
}

#endif /*INC_CLAMP_FUNCTION_HPP_*/
