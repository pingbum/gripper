/**
 * @file DRV8316C_SPI.hpp
 * @brief DRV8316C gate driver SPI interface.
 */

#ifndef INC_DRV8316C_SPI_HPP_
#define INC_DRV8316C_SPI_HPP_

#include "DRV8316C_Register.hpp"
#include "SPI_handler.hpp"
#include <cstdint>
#include <math.h>

#define READ 1
#define WRITE 0

static constexpr float EPSILON = 0.000001f;

class DRV8316C_t {
public:
  explicit DRV8316C_t(SPIHandler_t &spi) : m_spi{spi} {}

  /**
   * @brief Transfer a command to the driver.
   * @param isread True for read, false for write.
   * @param addr Register address.
   * @param data Data byte.
   * @param rx_buf Receive buffer (2 bytes).
   * @return HAL status code.
   */
  HAL_StatusTypeDef ReadWrite(bool isread, uint8_t addr, uint8_t data,
                              uint8_t *rx_buf);

  /**
   * @brief Read a register.
   */
  HAL_StatusTypeDef Read(uint8_t addr, uint8_t *rx_data);

  /**
   * @brief Update a bitfield within a register.
   */
  HAL_StatusTypeDef changeField(uint8_t addr, uint8_t data, uint8_t bit,
                                uint8_t bit_length);

  /**
   * @brief Lock or unlock the register write access.
   */
  HAL_StatusTypeDef lockRegister(bool lock);

  /**
   * @brief Set buck converter output voltage.
   */
  HAL_StatusTypeDef setBuckVoltage(float voltage);

  /**
   * @brief Configure slew rate.
   */
  HAL_StatusTypeDef setSLEW(uint8_t rate);

  /**
   * @brief Select PWM frequency at 100% duty.
   */
  HAL_StatusTypeDef setPWMDUTY(uint8_t duty);

  /**
   * @brief Clear Fault
   */
  HAL_StatusTypeDef clearFAULT();

  /**
   * @brief Set current sense amplifier gain.
   */
  HAL_StatusTypeDef setCurrentGain(uint8_t gain);

  /**
   * @brief Enable or disable active asynchronous rectification (AAR).
   */
  HAL_StatusTypeDef enableAAR(bool enable);

  /**
   * @brief Enable or disable active synchronous rectification (ASR).
   */
  HAL_StatusTypeDef enableASR(bool enable);

  /**
   * @brief Configure driver delay compensation.
   * @param enable True to enable, false to disable.
   * @param target Delay target code (DLY_TARGET_*).
   */
  HAL_StatusTypeDef setDelayCompensation(bool enable, uint8_t target);

  /**
   * @brief Enable or disable the buck converter.
   */
  HAL_StatusTypeDef enableBuck(bool enable);

private:
  static inline uint8_t parity(uint8_t hi, uint8_t lo) {
    uint16_t w = (uint16_t(hi) << 8) | lo;
    return (__builtin_popcount(w) & 1U) ? 1U : 0U;
  }

  SPIHandler_t &m_spi;
};

#endif /* INC_DRV8316C_SPI_HPP_ */
