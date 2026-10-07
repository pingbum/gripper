#pragma once

#include <cstdint>

/**
 * @brief Load control parameters from flash.
 * @return True on success.
 */
bool load_control_params_from_flash(void);

/**
 * @brief Save control parameters to flash.
 * @return True on success.
 */
bool save_control_params_to_flash(void);

/**
 * @brief Set CAN broadcast rate (1..1000 Hz).
 * @return True if the rate is valid and applied.
 */
bool set_can_broadcast_rate_hz(uint16_t rate_hz);

/**
 * @brief Get CAN broadcast rate (Hz).
 */
uint16_t get_can_broadcast_rate_hz(void);
