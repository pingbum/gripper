/**
 * @file MA732.hpp
 * @brief MA732 SPI angle and configuration register interface.
 */
#ifndef INC_MA732_HPP_
#define INC_MA732_HPP_

#include "SPI_handler.hpp"
#include <arm_math_types.h>
#include <cstdint>

class MA732_t {
public:
  explicit MA732_t(SPIHandler_t &spi) : m_spi{spi} {}

  /** Read the 16-bit angle word returned by the sensor. */
  HAL_StatusTypeDef readAngle(uint16_t &angle);

  /** Read the 14-bit angle value (0..16383). */
  uint16_t readAngleRaw();

  /** Read the mechanical angle in degrees (0 <= angle < 360). */
  float readAngleDeg();

  /**
   * Read an 8-bit register at address 0x00..0x1F.
   * Call from thread/main context with interrupts enabled, never from an ISR.
   * The caller must suspend motor operation and any shared-SPI Slave control.
   * On failure, value is unchanged. HAL_OK does not prove sensor presence.
   */
  HAL_StatusTypeDef readRegister(uint8_t address, uint8_t &value);

  /**
   * Write a documented configuration register, then read it back.
   * This updates sensor NVM; it waits at least 20ms after the write command.
   * Reserved bits and the read-only magnetic-status register cannot be written.
   * An unchanged value is not programmed again. On failure, readback is unchanged.
   * Same calling-context/disabled-motor requirements as readRegister().
   */
  HAL_StatusTypeDef writeRegister(uint8_t address, uint8_t value,
                                  uint8_t &readback);

  /** TIM6 must skip angle polling while a register transaction owns SPI3. */
  bool registerAccessInProgress() const { return m_register_active; }

  /** Last register response frame including its low byte; zero if unavailable. */
  uint16_t lastRegisterResponse() const { return m_last_register_response; }

  static bool isWritableRegister(uint8_t address);

private:
  SPIHandler_t &m_spi;
  volatile bool m_register_active = false;
  bool m_response_pending = false;
  uint16_t m_last_register_response = 0U;

  bool beginRegisterAccess();
  void endRegisterAccess();
  HAL_StatusTypeDef transferWord(uint16_t tx, uint16_t &rx);
  HAL_StatusTypeDef drainRegisterResponse();
  HAL_StatusTypeDef readRegisterUnlocked(uint8_t address, uint8_t &value);
  static uint8_t writableMask(uint8_t address);
};

#endif /* INC_MA732_HPP_ */
