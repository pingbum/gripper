#include "can.hpp"
#include "constants.hpp"
#include "control_params.hpp"
#include "cogging_compensation.hpp"
#include "current_loop.hpp"
#include "error_handler.hpp"
#include "flash_memory.hpp"
#include "low_speed_loop.hpp"
#include "memory_constants.hpp"
#include <cstring>

extern State_t state;

// Debugger counters. Queued TX means accepted by FDCAN, not bus ACK.
volatile uint32_t can_status_tx_queued = 0;
volatile uint32_t can_status_tx_errors = 0;
volatile uint32_t can_rx_frames = 0;
volatile uint32_t can_rx_fd_brs_frames = 0;
volatile uint32_t can_rx_rejected_frames = 0;

namespace {
constexpr uint8_t DEFAULT_DRIVER_ID = 0x01; // Default motor/driver ID.
constexpr uint32_t CAN_ID_CONFIG_MAGIC =
    0xA5A5A5A5u;                          // Magic for CAN ID config.
constexpr uint8_t MODE_READ_FLAG = 0x80u; // Read-back request bit.
constexpr uint8_t MAX_DRIVER_ID = 0xFFu;

struct __attribute__((packed, aligned(8))) CanIdConfig_t {
  uint32_t magic;
  uint32_t id;
};
static_assert(sizeof(CanIdConfig_t) == 8, "CanIdConfig_t must be 8 bytes");

uint8_t s_driver_id = DEFAULT_DRIVER_ID;
bool s_can_broadcast_enabled = true;
uint16_t s_broadcast_accumulator = 800;

static inline uint32_t be32_to_u32(const uint8_t *d) {
  return ((uint32_t)d[0] << 24) | ((uint32_t)d[1] << 16) |
         ((uint32_t)d[2] << 8) | (uint32_t)d[3];
}

static inline int32_t be32_to_s32(const uint8_t *d) {
  return static_cast<int32_t>(be32_to_u32(d));
}

static inline float32_t be32_to_f32(const uint8_t *d) {
  uint32_t raw = be32_to_u32(d);
  float32_t value = 0.0f;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

static inline void u32_to_be32(uint32_t value, uint8_t *out) {
  out[0] = (value >> 24) & 0xFFu;
  out[1] = (value >> 16) & 0xFFu;
  out[2] = (value >> 8) & 0xFFu;
  out[3] = value & 0xFFu;
}

static inline void f32_to_be32(float32_t value, uint8_t *out) {
  uint32_t raw = 0;
  std::memcpy(&raw, &value, sizeof(raw));
  u32_to_be32(raw, out);
}

static uint32_t dlc_from_len(uint8_t length) {
  switch (length) {
  case 1:
    return FDCAN_DLC_BYTES_1;
  case 4:
    return FDCAN_DLC_BYTES_4;
  case 8:
    return FDCAN_DLC_BYTES_8;
  default:
    return FDCAN_DLC_BYTES_1;
  }
}

static void send_param_response(uint8_t driver_id, uint8_t mode_id,
                                const uint8_t *payload, uint8_t length) {
  uint8_t tx_data[8] = {};
  if (!payload || length == 0 || length > sizeof(tx_data))
    return;
  std::memcpy(tx_data, payload, length);

  FDCAN_TxHeaderTypeDef txHeader = {.Identifier =
                                        (static_cast<uint32_t>(mode_id) << 8) |
                                        static_cast<uint32_t>(driver_id),
                                    .IdType = FDCAN_EXTENDED_ID,
                                    .TxFrameType = FDCAN_DATA_FRAME,
                                    .DataLength = dlc_from_len(length),
                                    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
                                    .BitRateSwitch = FDCAN_BRS_ON,
                                    .FDFormat = FDCAN_FD_CAN,
                                    .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
                                    .MessageMarker = 0};

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, tx_data) != HAL_OK) {
    Error_Raise(ERROR_CAN);
  }
}

static bool load_driver_id_from_flash() {
  const CanIdConfig_t *cfg =
      reinterpret_cast<const CanIdConfig_t *>(CAN_ID_FLASH_ADDR);
  if (cfg->magic != CAN_ID_CONFIG_MAGIC)
    return false;
  if (cfg->id > MAX_DRIVER_ID)
    return false;
  s_driver_id = static_cast<uint8_t>(cfg->id);
  return true;
}

static bool save_driver_id_to_flash(uint8_t new_id) {
  CanIdConfig_t cfg{CAN_ID_CONFIG_MAGIC, static_cast<uint32_t>(new_id)};
  uint32_t status = write_flash_buffer(
      CAN_ID_FLASH_ADDR, reinterpret_cast<uint8_t *>(&cfg), 1, sizeof(cfg));
  return (status == HAL_OK);
}

static bool parse_driver_id_payload(const FDCAN_RxHeaderTypeDef &rxh,
                                    const uint8_t *data, uint8_t *out_id) {
  if (!out_id)
    return false;
  uint32_t value = 0;
  if (rxh.DataLength == FDCAN_DLC_BYTES_1) {
    value = data[0];
  } else if (rxh.DataLength == FDCAN_DLC_BYTES_4 ||
             rxh.DataLength == FDCAN_DLC_BYTES_8) {
    value = be32_to_u32(&data[0]);
  } else {
    return false;
  }
  if (value > MAX_DRIVER_ID)
    return false;
  *out_id = static_cast<uint8_t>(value);
  return true;
}

static bool parse_u32_payload(const FDCAN_RxHeaderTypeDef &rxh,
                              const uint8_t *data, uint32_t *out_value) {
  if (!out_value)
    return false;
  if (rxh.DataLength == FDCAN_DLC_BYTES_1) {
    *out_value = data[0];
    return true;
  }
  if (rxh.DataLength == FDCAN_DLC_BYTES_4 ||
      rxh.DataLength == FDCAN_DLC_BYTES_8) {
    *out_value = be32_to_u32(&data[0]);
    return true;
  }
  return false;
}

static bool parse_be_f32_payload(const FDCAN_RxHeaderTypeDef &rxh,
                                 const uint8_t *data, float32_t *out_value) {
  if (!out_value)
    return false;
  if (rxh.DataLength != FDCAN_DLC_BYTES_4 &&
      rxh.DataLength != FDCAN_DLC_BYTES_8)
    return false;
  *out_value = be32_to_f32(&data[0]);
  return true;
}

static void handle_driver_id_update(const FDCAN_RxHeaderTypeDef &rxh,
                                    const uint8_t *data) {
  uint8_t new_id = 0;
  if (parse_driver_id_payload(rxh, data, &new_id)) {
    if (save_driver_id_to_flash(new_id)) {
      s_driver_id = new_id;
    }
  }
}

static void handle_rx_message(const FDCAN_RxHeaderTypeDef &rxh,
                              const uint8_t *data) {
  const uint32_t eid = rxh.Identifier;

  if (rxh.IdType == FDCAN_STANDARD_ID) {
    if (eid == MODE_SET_CAN_ID) {
      handle_driver_id_update(rxh, data);
    }
    return;
  }

  if (rxh.IdType != FDCAN_EXTENDED_ID)
    return;

  const uint8_t driver_id = (uint8_t)(eid & 0xFFu);
  const uint8_t mode_id = (uint8_t)((eid >> 8) & 0xFFu);

  if (eid == MODE_SET_CAN_ID ||
      (mode_id == MODE_SET_CAN_ID && driver_id == s_driver_id)) {
    handle_driver_id_update(rxh, data);
    return;
  }

  if (driver_id != s_driver_id)
    return; // Ignore frames for other drivers.

  if (mode_id == MODE_SYSTEM_RESET) {
    NVIC_SystemReset();
    return;
  }

  if (mode_id == MODE_CAN_BROADCAST_TOGGLE) {
    if (rxh.DataLength == FDCAN_DLC_BYTES_1 ||
        rxh.DataLength == FDCAN_DLC_BYTES_4 ||
        rxh.DataLength == FDCAN_DLC_BYTES_8) {
      s_can_broadcast_enabled = (data[0] != 0u);
    } else {
      s_can_broadcast_enabled = !s_can_broadcast_enabled;
    }
    if (!s_can_broadcast_enabled)
      s_broadcast_accumulator = 0;
    return;
  }

  if (mode_id == MODE_CAN_BROADCAST_RATE) {
    uint32_t rate_hz = 0;
    if (!parse_u32_payload(rxh, data, &rate_hz))
      return;
    if (!set_can_broadcast_rate_hz(static_cast<uint16_t>(rate_hz)))
      return;
    s_broadcast_accumulator = 0;
    if (!save_control_params_to_flash()) {
      Error_Raise(ERROR_CAN);
    }
    return;
  }

  if ((mode_id & MODE_READ_FLAG) != 0u) {
    const uint8_t base_mode = (uint8_t)(mode_id & ~MODE_READ_FLAG);
    uint8_t payload[8] = {};
    uint8_t length = 0;

    if (base_mode == MODE_SET_CAN_ID) {
      payload[0] = s_driver_id;
      length = 1;
    } else if (base_mode == MODE_SET_CURRENT_BANDWITDH) {
      f32_to_be32(current_loop_get_cutoff_freq(), payload);
      length = 4;
    } else if (base_mode == MODE_SET_VELOCITY_KP ||
               base_mode == MODE_SET_VELOCITY_KI) {
      float32_t kp = 0.0f;
      float32_t ki = 0.0f;
      velocity_loop_get_gains(&kp, &ki);
      f32_to_be32(base_mode == MODE_SET_VELOCITY_KP ? kp : ki, payload);
      length = 4;
    } else {
      return;
    }

    send_param_response(driver_id, mode_id, payload, length);
    return;
  }

  if (mode_id == MODE_SAVE_CTRL_PARAMS) {
    save_control_params_to_flash();
    return;
  }

  if (mode_id == MODE_COGGING_COMPENSATION) {
    if (!(rxh.DataLength == FDCAN_DLC_BYTES_8 ||
          rxh.DataLength == FDCAN_DLC_BYTES_4))
      return;
    const int32_t rpm_x100 = be32_to_s32(&data[0]);
    if (rpm_x100 == 0)
      cogging_measurement_stop();
    else
      cogging_measurement_start(static_cast<float32_t>(rpm_x100) * 0.01f);
    return;
  }

  if (mode_id == MODE_COGGING_TOGGLE) {
    if (!(rxh.DataLength == FDCAN_DLC_BYTES_8 ||
          rxh.DataLength == FDCAN_DLC_BYTES_4 ||
          rxh.DataLength == FDCAN_DLC_BYTES_1))
      return;
    cogging_compensation_set_enabled(data[0] != 0U);
    return;
  }

  if (mode_id == MODE_SET_CURRENT_BANDWITDH ||
      mode_id == MODE_SET_VELOCITY_KP || mode_id == MODE_SET_VELOCITY_KI) {
    float32_t value = 0.0f;
    if (!parse_be_f32_payload(rxh, data, &value))
      return;
    if (mode_id == MODE_SET_CURRENT_BANDWITDH) {
      current_loop_set_cutoff_freq(value);
    } else if (mode_id == MODE_SET_VELOCITY_KP) {
      velocity_loop_set_kp(value);
    } else {
      velocity_loop_set_ki(value);
    }
    return;
  }

  // Accept 4 bytes or more (8 bytes recommended by spec).
  if (!(rxh.DataLength == FDCAN_DLC_BYTES_8 ||
        rxh.DataLength == FDCAN_DLC_BYTES_4))
    return;

  // First 4 bytes BE -> s32.
  const int32_t raw = be32_to_s32(&data[0]);

  if (mode_id == MODE_CURRENT) {
    // Units: mA. Range -60000..60000 -> -6..6 A.
    int32_t mA = raw;
    float A = (float)mA * 0.001f;
    // Apply current loop command (q-axis only).
    state.iq_ref = A;
    state.CONTROL_MODE = MODE_CURRENT;
    // Add driver mode switching here if required.
  } else if (mode_id == MODE_VELOCITY) {
    // Units: eRPM. Range -100000..100000.
    int32_t erpm_i32 = raw;
    float erpm = (float)erpm_i32;
    // Apply speed command.
    state.omega_ref = erpm / (state.position.p); // erpm -> rpm
    state.CONTROL_MODE = MODE_VELOCITY;
  } else if (mode_id == MODE_POSITION) {
    state.theta_ref = (float32_t)raw * 0.01f;
    state.CONTROL_MODE = MODE_POSITION;
  } else if (mode_id == MODE_CALIBRATION) {
    // Doubleword start address aligned to 8 bytes.
    const uint32_t dw_addr =
        ENCODER_FLASH_UPDATE & ~0x7UL; // = ENCODER_STATE_ADDR + 8

    // 8-byte payload: [0..3] = 0xFF..FF (reserved)
    //                 [4..7] = flag 0
    uint8_t dw_payload[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00};
    write_flash_buffer(dw_addr, dw_payload, 1, 8);

    NVIC_SystemReset();
  }
}
} // namespace

void CAN_Handler::FDCAN1_SetupFiltersAndStart(void) {
  load_driver_id_from_flash();
  // Configure protected registers while HAL is READY, before starting FDCAN.
  // ST's G4 example recommends DataTimeSeg1 * DataPrescaler, in kernel clocks.
  // At 170 MHz, data prescaler 2 / seg1 12 gives TDCO = 24; TDCF = 0.
  const uint32_t tdc_offset =
      hfdcan1.Init.DataTimeSeg1 * hfdcan1.Init.DataPrescaler;
  if (tdc_offset > 127U ||
      HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, tdc_offset, 0) != HAL_OK ||
      HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK) {
    Error_Raise(ERROR_CAN);
    return;
  }

  // Standard ID filter: receive CAN ID set command.
  FDCAN_FilterTypeDef stdFilter{};
  stdFilter.IdType = FDCAN_STANDARD_ID;
  stdFilter.FilterIndex = 0;
  stdFilter.FilterType = FDCAN_FILTER_MASK;
  stdFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  stdFilter.FilterID1 = MODE_SET_CAN_ID;
  stdFilter.FilterID2 = 0x7FFu; // Allow only the exact ID.
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &stdFilter) != HAL_OK) {
    Error_Raise(ERROR_CAN);
    return;
  }

  // Extended ID filter: accept all, then filter in software.
  FDCAN_FilterTypeDef extFilter{};
  extFilter.IdType = FDCAN_EXTENDED_ID;
  extFilter.FilterIndex = 0;
  extFilter.FilterType = FDCAN_FILTER_MASK;
  extFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  extFilter.FilterID1 = 0x00000000u;
  extFilter.FilterID2 = 0x00000000u; // Mask: allow all extended IDs.
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &extFilter) != HAL_OK) {
    Error_Raise(ERROR_CAN);
    return;
  }

  // No unhandled fallback into FIFO0; remote frames are not commands.
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK) {
    Error_Raise(ERROR_CAN);
    return;
  }

  // Arm RX before starting, so no traffic is accepted before setup completes.
  if (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                     FDCAN_IT_RX_FIFO1_NEW_MESSAGE,
                                     0) != HAL_OK) {
    Error_Raise(ERROR_CAN);
    return;
  }
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    Error_Raise(ERROR_CAN);
  }
}

/**
 * @brief FDCAN RX FIFO1 callback.
 * @param hfdcan FDCAN handle pointer.
 * @param itFlags Interrupt flags.
 */
extern "C" void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan,
                                          uint32_t itFlags) {
  if ((itFlags & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == 0)
    return;

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1) > 0) {
    FDCAN_RxHeaderTypeDef rxh;
    // HAL copies the complete DLC payload before software can inspect it.
    // This is an RX scratch buffer, not a change to the 8-byte wire protocol.
    uint8_t data[64] = {};

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rxh, data) != HAL_OK)
      break;

    can_rx_frames = can_rx_frames + 1U;
    if (rxh.FDFormat == FDCAN_FD_CAN && rxh.BitRateSwitch == FDCAN_BRS_ON)
      can_rx_fd_brs_frames = can_rx_fd_brs_frames + 1U;

    // Keep existing 0-byte read/reset requests and 1/4/8-byte payloads.
    // Reject invalid lengths before any command (including reset/flash writes).
    if (rxh.RxFrameType != FDCAN_DATA_FRAME ||
        (rxh.DataLength != FDCAN_DLC_BYTES_0 &&
         rxh.DataLength != FDCAN_DLC_BYTES_1 &&
         rxh.DataLength != FDCAN_DLC_BYTES_4 &&
         rxh.DataLength != FDCAN_DLC_BYTES_8)) {
      can_rx_rejected_frames = can_rx_rejected_frames + 1U;
      continue;
    }
    handle_rx_message(rxh, data);
  }

  return;
}

/**
 * @brief Broadcast motor status over CAN.
 * @param position_deg Position in degrees.
 * @param speed_erpm Speed in eRPM.
 * @param current_A Current in amperes.
 * @param temp_C Temperature in Celsius.
 * @param error_code Error code.
 */
void CAN_Handler::broadcast_motor_status(float position_deg, float speed_erpm,
                                         float current_A, int8_t rev,
                                         int8_t error_code) {
  if (!s_can_broadcast_enabled) {
    s_broadcast_accumulator = 0;
    return;
  }

  const uint16_t rate_hz = get_can_broadcast_rate_hz();
  if (rate_hz == 0 || rate_hz > 1000U)
    return;

  s_broadcast_accumulator =
      static_cast<uint16_t>(s_broadcast_accumulator + rate_hz);
  if (s_broadcast_accumulator < 1000U)
    return;
  s_broadcast_accumulator =
      static_cast<uint16_t>(s_broadcast_accumulator - 1000U);

  FDCAN_TxHeaderTypeDef txHeader = {
      .Identifier = static_cast<uint32_t>(s_driver_id),
      .IdType = FDCAN_EXTENDED_ID,
      .TxFrameType = FDCAN_DATA_FRAME,
      .DataLength = FDCAN_DLC_BYTES_8, // 8 bytes.
      .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
      .BitRateSwitch = FDCAN_BRS_ON,
      .FDFormat = FDCAN_FD_CAN,
      .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
      .MessageMarker = 0};
  uint8_t tx_data[8];

  // Position: scale from deg to int16
  // -3200.0 to 3200.0 -> -32000 to 32000
  int16_t pos_scaled = static_cast<int16_t>(position_deg * 10.0f);

  // Speed: eRPM with encoder sign (absolute mechanical direction).
  // -320000 to 320000 -> -32000 to 32000
  int16_t speed_scaled = static_cast<int16_t>(speed_erpm / 10.0f);

  // Current: scale from A to int16
  // -6.00 to 6.00 -> -6000 to 6000
  int16_t current_scaled = static_cast<int16_t>(current_A * 1000.0f);

  tx_data[0] = (pos_scaled >> 8) & 0xFF;
  tx_data[1] = pos_scaled & 0xFF;
  tx_data[2] = (speed_scaled >> 8) & 0xFF;
  tx_data[3] = speed_scaled & 0xFF;
  tx_data[4] = (current_scaled >> 8) & 0xFF;
  tx_data[5] = current_scaled & 0xFF;
  tx_data[6] = rev;
  tx_data[7] = error_code; // Error code

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, tx_data) != HAL_OK) {
    can_status_tx_errors = can_status_tx_errors + 1U;
    Error_Raise(ERROR_CAN);
  } else {
    can_status_tx_queued = can_status_tx_queued + 1U;
  }
}
