#pragma once

#include <stdbool.h>
#include <stdint.h>

// Error flag bit masks.
// NOTE: ERROR_SPI covers both sensor SPI faults and HAL SPI init failures.
#define ERROR_CAN 0x01u
#define ERROR_SPI 0x02u
#define ERROR_DRIVER_FAULT 0x04u
#define ERROR_HAL_RCC 0x08u
#define ERROR_HAL_ADC 0x10u
#define ERROR_HAL_TIM 0x20u
#define ERROR_HAL_HRTIM 0x40u
#define ERROR_HAL_FMAC 0x80u

#ifdef __cplusplus
#include "constants.hpp"

/**
 * @brief Compose error flags from the current state.
 * @param state Application state pointer.
 * @return Bitmask of error flags.
 */
uint8_t GET_ERROR_FLAGS(State_t *state);
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Error handler entry point.
 */
void _Error_Handler(void);

/**
 * @brief Latch error flags and enter error state.
 * @param flags Error bitmask.
 */
void Error_SetFlags(uint8_t flags);

/**
 * @brief Latch error flags and stop control loops.
 * @param flags Error bitmask.
 */
void Error_Raise(uint8_t flags);

/**
 * @brief Check whether the system is in an error state.
 * @return 1 if error state is active, 0 otherwise.
 */
uint8_t Error_IsActive(void);

#ifdef __cplusplus
}
#endif
