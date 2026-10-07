/**
 * @file MA732.hpp
 * @brief Minimal MA732 SPI interface.
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

private:
  SPIHandler_t &m_spi;
};

#endif /* INC_MA732_HPP_ */
