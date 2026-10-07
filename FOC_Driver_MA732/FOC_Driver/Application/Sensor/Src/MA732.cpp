/**
 * @file MA732.cpp
 * @brief MA732 SPI angle reads and two-frame register transactions.
 */
#include "MA732.hpp"
#include "error_handler.hpp"

namespace {
constexpr float32_t DEGREES_PER_COUNT = 360.0f / 16384.0f;

// MA732 Rev. 1.1, tables 3 and 7, SPI read/write sections (pp. 11, 13-14, 17).
// Register responses need >=750ns of CS-high idle before/after the frame.
// A conservative HAL millisecond delay keeps this debug path independent of
// CPU instruction timing. DMA/SysTick interrupts must remain enabled.
constexpr uint32_t REGISTER_IDLE_MS = 1U;
constexpr uint32_t NVM_WRITE_MS = 20U;
constexpr uint16_t READ_REGISTER = 0x4000U;
constexpr uint16_t WRITE_REGISTER = 0x8000U;
}

HAL_StatusTypeDef MA732_t::readAngle(uint16_t &angle) {
  if (m_register_active)
    return HAL_BUSY;

  // A failed register response must never become a mechanical-angle sample.
  // Retry a register read (with polling stopped) or reset/reinitialize first.
  if (m_response_pending)
    return HAL_ERROR;

  return transferWord(0U, angle);
}

HAL_StatusTypeDef MA732_t::transferWord(uint16_t word, uint16_t &received) {
  buffer_t tx{};
  buffer_t rx{};
  tx.data = word;

  const HAL_StatusTypeDef status = m_spi.transfer(tx.packet, rx.packet, 1U);
  if (status == HAL_OK)
    received = rx.data;
  return status;
}

uint16_t MA732_t::readAngleRaw() {
  uint16_t angle = 0U;
  if (readAngle(angle) != HAL_OK) {
    Error_Raise(ERROR_SPI);
    return 0U;
  }

  // MA732 supplies its 14 useful bits in bits 15:2 of the 16-bit frame.
  return angle >> 2U;
}

float MA732_t::readAngleDeg() {
  return static_cast<float32_t>(readAngleRaw()) * DEGREES_PER_COUNT;
}

bool MA732_t::beginRegisterAccess() {
  if (__get_IPSR() != 0U || __get_PRIMASK() != 0U)
    return false;
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const bool available = !m_register_active;
  if (available)
    m_register_active = true;
  __DMB();
  __set_PRIMASK(primask);
  return available;
}

void MA732_t::endRegisterAccess() {
  __DMB();
  m_register_active = false;
}

HAL_StatusTypeDef MA732_t::drainRegisterResponse() {
  if (!m_response_pending)
    return HAL_OK;
  uint16_t discarded = 0U;
  const HAL_StatusTypeDef status = transferWord(0U, discarded);
  if (status == HAL_OK) {
    m_response_pending = false;
  }
  return status;
}

HAL_StatusTypeDef MA732_t::readRegisterUnlocked(uint8_t address,
                                              uint8_t &value) {
  HAL_StatusTypeDef status = drainRegisterResponse();
  if (status != HAL_OK)
    return status;

  HAL_Delay(REGISTER_IDLE_MS);
  uint16_t received = 0U;
  m_last_register_response = 0U;
  // Even an errored DMA transfer may have put the command on the wire.
  m_response_pending = true;
  status = transferWord(READ_REGISTER | (static_cast<uint16_t>(address) << 8U),
                        received);
  if (status != HAL_OK)
    return status;

  HAL_Delay(REGISTER_IDLE_MS);
  status = transferWord(0U, received);
  if (status == HAL_OK) {
    m_response_pending = false;
    m_last_register_response = received;
    // The response is V[7:0] followed by eight zero bits. Reject an obvious
    // floating/stuck-high MISO frame instead of reporting 0xFF as a valid value.
    if ((received & 0x00FFU) != 0U)
      status = HAL_ERROR;
  }
  HAL_Delay(REGISTER_IDLE_MS);
  if (status == HAL_OK)
    value = static_cast<uint8_t>(received >> 8U);
  return status;
}

HAL_StatusTypeDef MA732_t::readRegister(uint8_t address, uint8_t &value) {
  if (address > 0x1FU)
    return HAL_ERROR;
  if (!beginRegisterAccess())
    return HAL_BUSY;
  const HAL_StatusTypeDef status = readRegisterUnlocked(address, value);
  endRegisterAccess();
  return status;
}

uint8_t MA732_t::writableMask(uint8_t address) {
  switch (address) {
  case 0x00U: // Z low
  case 0x01U: // Z high
  case 0x02U: // BCT
  case 0x05U: // PPT high
  case 0x0EU: // FW
  case 0x10U: // HYS
    return 0xFFU;
  case 0x03U: // ETY, ETX
    return 0x03U;
  case 0x04U: // PPT low, ILIP
  case 0x06U: // MGLT, MGHT
    return 0xFCU;
  case 0x09U: // RD
    return 0x80U;
  default:
    return 0U;
  }
}

bool MA732_t::isWritableRegister(uint8_t address) {
  return writableMask(address) != 0U;
}

HAL_StatusTypeDef MA732_t::writeRegister(uint8_t address, uint8_t value,
                                        uint8_t &readback) {
  const uint8_t mask = writableMask(address);
  if (mask == 0U || (value & static_cast<uint8_t>(~mask)) != 0U)
    return HAL_ERROR;
  if (!beginRegisterAccess())
    return HAL_BUSY;

  uint8_t previous = 0U;
  HAL_StatusTypeDef status = readRegisterUnlocked(address, previous);
  if (status == HAL_OK && (previous & mask) != value) {
    // Preserve reserved bits read from the device.
    const uint8_t programmed = (previous & static_cast<uint8_t>(~mask)) | value;
    uint16_t received = 0U;
    m_response_pending = true;
    status = transferWord(WRITE_REGISTER |
                              (static_cast<uint16_t>(address) << 8U) | programmed,
                          received);
    // A reported SPI failure may still have transmitted a complete command.
    // Honor NVM busy time even then, before a caller can retry/recover.
    HAL_Delay(NVM_WRITE_MS);
    if (status == HAL_OK) {
      status = transferWord(0U, received);
      if (status == HAL_OK) {
        m_response_pending = false;
        m_last_register_response = received;
        if ((received & 0x00FFU) != 0U ||
            (static_cast<uint8_t>(received >> 8U) & mask) != value)
          status = HAL_ERROR;
      }
      HAL_Delay(REGISTER_IDLE_MS);
    }
    // Verify with a fresh read request, rather than only the write echo.
    if (status == HAL_OK) {
      status = readRegisterUnlocked(address, previous);
      if (status == HAL_OK && (previous & mask) != value)
        status = HAL_ERROR;
    }
  }
  if (status == HAL_OK)
    readback = previous;
  endRegisterAccess();
  return status;
}
