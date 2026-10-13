#pragma once

#include <cstddef>
#include <cstdint>

// UDP packets from the phone. Layout (little-endian), see docs/PLAN.md:
//   0  char[4] "HMD1"
//   4  u8      type
//   5  u8[3]   reserved
//   8  u32     seq
//  12  i64     timestamp_ns (Android SensorEvent.timestamp)
//  20  f32[3]  gyro  (rad/s)    -- sensor packets only
//  32  f32[3]  accel (m/s^2)    -- sensor packets only
//  44  f32[3]  mag   (uT)       -- sensor packets only
namespace protocol {

constexpr uint16_t kSensorPort = 9944;
constexpr size_t kHeaderSize = 20;
constexpr size_t kSensorPacketSize = 56;

enum class PacketType : uint8_t {
    Sensors = 1,
    Recenter = 2,
    KeyframeRequest = 3,
};

struct Packet {
    PacketType type = PacketType::Sensors;
    uint32_t seq = 0;
    int64_t timestamp_ns = 0;
    float gyro[3] = {};
    float accel[3] = {};
    float mag[3] = {};
};

// Returns false if the bytes are not a valid packet.
bool Parse(const uint8_t* data, size_t size, Packet& out);

}  // namespace protocol
