#include "control_params.hpp"
#include "current_loop.hpp"
#include "flash_memory.hpp"
#include "low_speed_loop.hpp"
#include "memory_constants.hpp"
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace {
constexpr uint32_t CONTROL_PARAM_MAGIC = 0x4350524Du;
constexpr uint32_t POSITION_PARAM_MAGIC = 0x50504431u; // "PPD1" extension.
constexpr uint16_t BROADCAST_RATE_MIN_HZ = 1;
constexpr uint16_t BROADCAST_RATE_MAX_HZ = 1000;

struct __attribute__((packed, aligned(8))) ControlParamBlock_t {
  uint32_t magic;
  float32_t cutoff_freq_cur;
  float32_t kp_vel;
  float32_t ki_vel;
  uint32_t can_broadcast_rate_hz;
  // Preserve the complete legacy 24-byte prefix, including its tail padding.
  uint32_t legacy_reserved;
  uint32_t position_magic;
  float32_t kp_pos;
  float32_t kd_pos;
  uint32_t reserved;
};
static_assert(offsetof(ControlParamBlock_t, position_magic) == 24,
              "Legacy control parameter layout must not change");
static_assert(sizeof(ControlParamBlock_t) == 40,
              "ControlParamBlock_t must be 40 bytes");

uint16_t s_can_broadcast_rate_hz = BROADCAST_RATE_MAX_HZ;

bool read_control_param_block(ControlParamBlock_t *out) {
  if (!out)
    return false;
  std::memcpy(out, reinterpret_cast<const void *>(CONTROL_PARAM_FLASH_ADDR),
              sizeof(ControlParamBlock_t));
  return out->magic == CONTROL_PARAM_MAGIC;
}

bool verify_control_param_block(const ControlParamBlock_t &block) {
  ControlParamBlock_t verify{};
  std::memcpy(&verify, reinterpret_cast<const void *>(CONTROL_PARAM_FLASH_ADDR),
              sizeof(ControlParamBlock_t));
  return std::memcmp(&verify, &block, sizeof(ControlParamBlock_t)) == 0;
}

bool is_valid_broadcast_rate(uint32_t rate_hz) {
  return rate_hz >= BROADCAST_RATE_MIN_HZ && rate_hz <= BROADCAST_RATE_MAX_HZ;
}
} // namespace

bool load_control_params_from_flash(void) {
  ControlParamBlock_t block{};
  if (!read_control_param_block(&block))
    return false;

  current_loop_set_cutoff_freq(block.cutoff_freq_cur);
  velocity_loop_set_kp(block.kp_vel);
  velocity_loop_set_ki(block.ki_vel);
  if (is_valid_broadcast_rate(block.can_broadcast_rate_hz)) {
    s_can_broadcast_rate_hz =
        static_cast<uint16_t>(block.can_broadcast_rate_hz);
  }
  // Old firmware wrote only the legacy prefix. Leave position defaults alone
  // unless this explicitly tagged extension exists; setters reject bad gains.
  if (block.position_magic == POSITION_PARAM_MAGIC) {
    position_loop_set_kp(block.kp_pos);
    position_loop_set_kd(block.kd_pos);
  }
  return true;
}

bool save_control_params_to_flash(void) {
  float32_t kp_vel = 0.0f;
  float32_t ki_vel = 0.0f;
  velocity_loop_get_gains(&kp_vel, &ki_vel);
  float32_t kp_pos = 0.0f;
  float32_t kd_pos = 0.0f;
  position_loop_get_gains(&kp_pos, &kd_pos);

  ControlParamBlock_t block = {CONTROL_PARAM_MAGIC,
                               current_loop_get_cutoff_freq(), kp_vel, ki_vel,
                               s_can_broadcast_rate_hz, 0xFFFFFFFFu,
                               POSITION_PARAM_MAGIC, kp_pos, kd_pos,
                               0xFFFFFFFFu};

  uint32_t status =
      write_flash_buffer(CONTROL_PARAM_FLASH_ADDR,
                         reinterpret_cast<uint8_t *>(&block), 1, sizeof(block));
  if (status != HAL_OK)
    return false;
  return verify_control_param_block(block);
}

bool set_can_broadcast_rate_hz(uint16_t rate_hz) {
  if (!is_valid_broadcast_rate(rate_hz))
    return false;
  s_can_broadcast_rate_hz = rate_hz;
  return true;
}

uint16_t get_can_broadcast_rate_hz(void) {
  return s_can_broadcast_rate_hz;
}
