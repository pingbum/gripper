/**
 * @file DRV8316C_SPI.cpp
 * @brief DRV8316C SPI register access.
 */

#include "DRV8316C_SPI.hpp"

/**
 * @brief Transfer a raw SPI command.
 * @param isread True for read, false for write.
 * @param addr Register address.
 * @param data Data byte to write.
 * @param rx_buf Receive buffer (2 bytes).
 * @return HAL status code.
 */
HAL_StatusTypeDef DRV8316C_t::ReadWrite(bool isread, uint8_t addr, uint8_t data,
                                        uint8_t *rx_buf) {
  buffer_t txbuf;
  txbuf.packet[1] = addr;
  txbuf.packet[0] = data;
  txbuf.packet[1] = (txbuf.packet[1] << 1) | (isread << 7);
  uint8_t parityBit = parity(txbuf.packet[0], txbuf.packet[1]);
  txbuf.packet[1] |= (parityBit);

  if (!isread)
    lockRegister(false);
  HAL_StatusTypeDef result = m_spi.transfer(txbuf.packet, rx_buf, 1);
  if (!isread)
    lockRegister(true);

  return result;
}

/**
 * @brief Read a register value.
 * @param addr Register address.
 * @param rx_data Receive buffer (2 bytes).
 * @return HAL status code.
 */
HAL_StatusTypeDef DRV8316C_t::Read(uint8_t addr, uint8_t *rx_data) {
  uint8_t dummy{};
  HAL_StatusTypeDef result = DRV8316C_t::ReadWrite(READ, addr, dummy, rx_data);
  return result;
}

/**
 * @brief Update a single bitfield within a register.
 * @param addr Register address.
 * @param data Field value to write.
 * @param bit Bit position.
 * @param bit_length Field width in bits.
 * @return HAL status code.
 */
HAL_StatusTypeDef DRV8316C_t::changeField(uint8_t addr, uint8_t data,
                                          uint8_t bit, uint8_t bit_length) {
  uint8_t positive_mask =
      0x00; // Example: length=1 -> 0b00000001, length=2 -> 0b00000011

  for (int i = 0; i < bit_length; i++)
    positive_mask |= 0x01 << i;

  uint8_t mask = ~(positive_mask << bit);
  uint8_t rx[2]{};
  DRV8316C_t::ReadWrite(READ, addr, 0x00, rx);

  uint8_t data_temp = (rx[0] & mask) | (data << bit);

  return DRV8316C_t::ReadWrite(WRITE, addr, data_temp, rx);
}

/**
 * @brief Lock or unlock register write access.
 * @param lock True to lock, false to unlock.
 * @return HAL status code.
 */
HAL_StatusTypeDef DRV8316C_t::lockRegister(bool lock) {
  uint8_t txbuf[2];
  txbuf[1] = REGISTER_LOCK_ADDR;
  txbuf[0] = (lock) ? 0x06 : 0x03;
  txbuf[1] = (txbuf[1] << 1) | (0 << 7);

  uint8_t parityBit = parity(txbuf[0], txbuf[1]);

  txbuf[1] |= parityBit;
  uint8_t dummy[2];
  return m_spi.transfer(txbuf, dummy, 1);
}

/**
 * @brief Set buck converter output voltage.
 * @param voltage 3.3, 4.0, 5.0, or 5.7 volts.
 * @return HAL status code.
 */
HAL_StatusTypeDef DRV8316C_t::setBuckVoltage(float voltage) {
  uint8_t data = 0x00;

  if (fabs(voltage - 3.3) < EPSILON)
    data = 0x00;
  else if (fabs(voltage - 5) < EPSILON)
    data |= 0x01;
  else if (fabs(voltage - 4) < EPSILON)
    data |= 0x02;
  else if (fabs(voltage - 5.7) < EPSILON)
    data |= 0x03;
  else
    data |= 0x00;

  return DRV8316C_t::changeField(BUCK_CONF_ADDR, data, BUCK_SEL,
                                 BUCK_SEL_LENGTH);
}

/**
 * @brief Enable or disable the buck converter.
 * @param enable True to enable, false to disable.
 * @return HAL status code.
 */
HAL_StatusTypeDef DRV8316C_t::enableBuck(bool enable) {
  uint8_t data = 0x00;

  if (enable)
    data = 0x00;
  else
    data = 0x01;

  return DRV8316C_t::changeField(BUCK_CONF_ADDR, data, BUCK_DIS,
                                 BUCK_DIS_LENGTH);
}

/**
 * @brief Set slew rate.
 * @param rate SLEW_25V, SLEW_50V, SLEW_125V, or SLEW_200V.
 * @return HAL status code.
 */
HAL_StatusTypeDef DRV8316C_t::setSLEW(uint8_t rate) {
  return DRV8316C_t::changeField(CTRL_REG2_ADDR, rate, SLEW, SLEW_LENGTH);
}

/**
 * @brief Select PWM frequency at 100% duty cycle.
 * @param duty 0h for 20 kHz, 1h for 40 kHz.
 * @return HAL status code.
 */
HAL_StatusTypeDef DRV8316C_t::setPWMDUTY(uint8_t duty) {
  return DRV8316C_t::changeField(CTRL_REG3_ADDR, duty, PWM_100_DUTY_SEL,
                                 PWM_100_DUTY_SEL_LENGTH);
}

/**
 * @brief Clear Fault
 * @return HAL status code.
 */
HAL_StatusTypeDef DRV8316C_t::clearFAULT() {
  return DRV8316C_t::changeField(CTRL_REG2_ADDR, 0x01, CLR_FLT, CLR_FLT_LENGTH);
}

/**
 * @brief Set current sense amplifier gain.
 * @param gain 0h..3h for 0.15/0.3/0.6/1.2 V/A.
 * @return HAL status code.
 */
HAL_StatusTypeDef DRV8316C_t::setCurrentGain(uint8_t gain) {
  return DRV8316C_t::changeField(CTRL_REG5_ADDR, gain, CSA_GAIN,
                                 CSA_GAIN_LENGTH);
}

/**
 * @brief Enable or disable active asynchronous rectification (AAR).
 * @param enable True to enable, false to disable.
 * @return HAL status code.
 */
HAL_StatusTypeDef DRV8316C_t::enableAAR(bool enable) {
  uint8_t data = enable ? 0x01 : 0x00;
  return DRV8316C_t::changeField(CTRL_REG5_ADDR, data, EN_AAR, EN_AAR_LENGTH);
}

/**
 * @brief Enable or disable active synchronous rectification (ASR).
 * @param enable True to enable, false to disable.
 * @return HAL status code.
 */
HAL_StatusTypeDef DRV8316C_t::enableASR(bool enable) {
  uint8_t data = enable ? 0x01 : 0x00;
  return DRV8316C_t::changeField(CTRL_REG5_ADDR, data, EN_ASR, EN_ASR_LENGTH);
}

/**
 * @brief Configure driver delay compensation.
 * @param enable True to enable, false to disable.
 * @param target Delay target code (DLY_TARGET_*).
 * @return HAL status code.
 */
HAL_StatusTypeDef DRV8316C_t::setDelayCompensation(bool enable,
                                                   uint8_t target) {
  uint8_t data = target & ((1U << DLY_TARGET_LENGTH) - 1U);
  HAL_StatusTypeDef result = DRV8316C_t::changeField(
      CTRL_REG10_ADDR, data, DLY_TARGET, DLY_TARGET_LENGTH);
  if (result != HAL_OK)
    return result;

  data = enable ? 0x01 : 0x00;
  return DRV8316C_t::changeField(CTRL_REG10_ADDR, data, DLYCMP_EN,
                                 DLYCMP_EN_LENGTH);
}
