#ifndef INC_BASIC_FUNCTION_HPP_
#define INC_BASIC_FUNCTION_HPP_

#include "arm_math.h"

/**
 * @brief Wrap a signed index into the range [0, bufsize).
 * @param index Signed index to wrap.
 * @param bufsize Buffer length.
 * @return Wrapped index as an unsigned value.
 */
inline uint32_t Index_circular(int index, uint32_t bufsize)
{
    if (index < 0)
        return (uint32_t)bufsize + index;
    else if (index >= (int)bufsize)
        return (uint32_t)index - bufsize;
    else
        return (uint32_t)index;
}
#endif /*INC_BASIC_FUNCTION_HPP_*/
