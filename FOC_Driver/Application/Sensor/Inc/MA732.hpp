#ifndef INC_MA732_HPP_
#define INC_MA732_HPP_
#include "stm32g4xx_hal.h"
#include <cstdint>

// Passive MA732 tap: PB3=SCK, PB5=sensor MISO, PB6=CS input.
class MA732_t {
public:
  struct Sample { uint16_t word; uint32_t cycles; uint32_t sequence; };
  HAL_StatusTypeDef start();
  HAL_StatusTypeDef readSample(Sample &sample);
  HAL_StatusTypeDef readAngle(uint16_t &angle);
  uint16_t readAngleRaw();
  float readAngleDeg();
};
extern "C" void MA732_SlaveWatchdog(void);
#endif
