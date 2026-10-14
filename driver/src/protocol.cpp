#include "protocol.h"

#include <cstring>

namespace protocol {
namespace {

uint32_t ReadU32(const uint8_t* p)
{
    return static_cast<uint32_t>(p[0]) | static_cast<uint32_t>(p[1]) << 8 | static_cast<uint32_t>(p[2]) << 16 |
           static_cast<uint32_t>(p[3]) << 24;
}

int64_t ReadI64(const uint8_t* p)
{
    const uint64_t low = ReadU32(p);
    const uint64_t high = ReadU32(p + 4);
    return static_cast<int64_t>(low | high << 32);
}

float ReadF32(const uint8_t* p)
{
    const uint32_t bits = ReadU32(p);
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

void ReadVec(const uint8_t* p, float out[3])
{
    for (int i = 0; i < 3; ++i) {
        out[i] = ReadF32(p + 4 * i);
    }
}

}  // namespace

bool Parse(const uint8_t* data, size_t size, Packet& out)
{
    if (size < kHeaderSize || std::memcmp(data, "HMD1", 4) != 0) {
        return false;
    }
    const uint8_t type = data[4];
    if (type < 1 || type > 6 || type == static_cast<uint8_t>(PacketType::PcHere)) {
        return false;
    }
    out.type = static_cast<PacketType>(type);
    out.seq = ReadU32(data + 8);
    out.timestamp_ns = ReadI64(data + 12);
    if (out.type == PacketType::Sensors) {
        if (size < kSensorPacketSize) {
            return false;
        }
        ReadVec(data + 20, out.gyro);
        ReadVec(data + 32, out.accel);
        ReadVec(data + 44, out.mag);
    } else if (out.type == PacketType::Latency) {
        if (size < kLatencyPacketSize) {
            return false;
        }
        out.latency_ms = ReadF32(data + 20);
    }
    return true;
}

size_t WritePcHere(uint8_t* out)
{
    std::memset(out, 0, kHeaderSize);
    std::memcpy(out, "HMD1", 4);
    out[4] = static_cast<uint8_t>(PacketType::PcHere);
    return kHeaderSize;
}

}  // namespace protocol
