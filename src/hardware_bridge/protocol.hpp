#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace open_mower_next::hardware_bridge {

constexpr uint8_t PACKET_ID_LL_STATUS = 1;
constexpr uint8_t PACKET_ID_LL_IMU = 2;
constexpr uint8_t PACKET_ID_LL_UI_EVENT = 3;
constexpr uint8_t PACKET_ID_LL_HEARTBEAT = 0x42;

constexpr uint8_t LL_EMERGENCY_BIT_LATCH = 1 << 0;
constexpr uint8_t LL_EMERGENCY_BIT_STOP = 1 << 1;
constexpr uint8_t LL_EMERGENCY_BIT_LIFT = 1 << 2;

constexpr uint8_t LL_STATUS_BIT_CHARGING = 1 << 2;
constexpr uint8_t LL_STATUS_BIT_RAIN = 1 << 4;

#pragma pack(push, 1)
struct LlStatus {
  uint8_t type;
  uint8_t status_bitmask;
  float uss_ranges_m[5];
  uint8_t emergency_bitmask;
  float v_charge;
  float v_system;
  float charging_current;
  uint8_t batt_percentage;
  uint16_t crc;
};

struct LlImu {
  uint8_t type;
  uint16_t dt_millis;
  float acceleration_mss[3];
  float gyro_rads[3];
  float mag_uT[3];
  uint16_t crc;
};

struct LlHeartbeat {
  uint8_t type;
  uint8_t emergency_requested;
  uint8_t emergency_release_requested;
  uint16_t crc;
};

struct LlUiEvent {
  uint8_t type;
  uint8_t button_id;
  uint8_t press_duration;
  uint16_t crc;
};
#pragma pack(pop)

static_assert(sizeof(LlStatus) == 38);
static_assert(sizeof(LlImu) == 41);
static_assert(sizeof(LlHeartbeat) == 5);
static_assert(sizeof(LlUiEvent) == 5);

inline uint16_t crc_ccitt(const uint8_t * data, size_t size)
{
  uint16_t crc = 0xffff;
  for (size_t i = 0; i < size; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000) != 0) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

inline uint16_t read_le_u16(const uint8_t * data)
{
  return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
}

inline void write_le_u16(uint8_t * data, uint16_t value)
{
  data[0] = static_cast<uint8_t>(value & 0xff);
  data[1] = static_cast<uint8_t>((value >> 8) & 0xff);
}

inline bool has_valid_crc(const std::vector<uint8_t> & packet)
{
  if (packet.size() < 3) {
    return false;
  }
  const auto expected = crc_ccitt(packet.data(), packet.size() - sizeof(uint16_t));
  const auto received = read_le_u16(packet.data() + packet.size() - sizeof(uint16_t));
  return expected == received;
}

inline std::vector<uint8_t> cobs_encode(const uint8_t * input, size_t size)
{
  std::vector<uint8_t> output;
  output.reserve(size + 2);

  size_t code_index = 0;
  uint8_t code = 1;
  output.push_back(0);

  for (size_t i = 0; i < size; ++i) {
    if (input[i] == 0) {
      output[code_index] = code;
      code_index = output.size();
      output.push_back(0);
      code = 1;
    } else {
      output.push_back(input[i]);
      ++code;
      if (code == 0xff) {
        output[code_index] = code;
        code_index = output.size();
        output.push_back(0);
        code = 1;
      }
    }
  }

  output[code_index] = code;
  output.push_back(0);
  return output;
}

inline std::optional<std::vector<uint8_t>> cobs_decode(const std::vector<uint8_t> & input)
{
  std::vector<uint8_t> output;
  output.reserve(input.size());

  size_t index = 0;
  while (index < input.size()) {
    const uint8_t code = input[index];
    if (code == 0) {
      return std::nullopt;
    }
    ++index;

    const size_t next_index = index + code - 1;
    if (next_index > input.size()) {
      return std::nullopt;
    }

    while (index < next_index) {
      output.push_back(input[index]);
      ++index;
    }

    if (code != 0xff && index < input.size()) {
      output.push_back(0);
    }
  }

  return output;
}

}  // namespace open_mower_next::hardware_bridge
