#include "body_protocol.h"

#include <cmath>
#include <cstring>

namespace body_protocol {
namespace {

uint32_t ReadU32(const uint8_t* p)
{
    return static_cast<uint32_t>(p[0]) | static_cast<uint32_t>(p[1]) << 8 | static_cast<uint32_t>(p[2]) << 16 |
           static_cast<uint32_t>(p[3]) << 24;
}

uint16_t ReadU16(const uint8_t* p)
{
    return static_cast<uint16_t>(p[0] | p[1] << 8);
}

float ReadF32(const uint8_t* p)
{
    const uint32_t bits = ReadU32(p);
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

}  // namespace

bool Parse(const uint8_t* data, size_t size, Frame* out)
{
    if (size < kHeaderSize || std::memcmp(data, "HMD1", 4) != 0 || data[4] != kTypeBodyFrame) {
        return false;
    }
    Frame frame;
    frame.seq = ReadU32(data + 8);
    frame.camera = ReadU32(data + 20);
    frame.width = ReadU16(data + 24);
    frame.height = ReadU16(data + 26);
    frame.age_seconds = ReadF32(data + 28) / 1000.0;
    frame.flags = data[32];
    frame.points = data[33];
    if (frame.points > kPoints || size < kHeaderSize + kPointSize * static_cast<size_t>(frame.points) ||
        frame.width == 0 || frame.height == 0 || !std::isfinite(frame.age_seconds) || frame.age_seconds < 0.0 ||
        frame.age_seconds > 5.0) {
        return false;
    }
    for (int i = 0; i < 3; ++i) {
        frame.ball[i] = ReadF32(data + 36 + 4 * i);
    }
    for (int p = 0; p < frame.points; ++p)
        for (int i = 0; i < 5; ++i) frame.point[p][i] = ReadF32(data + kHeaderSize + kPointSize * p + 4 * i);
    if (frame.flags & kFlagBulbs) {
        const size_t start = kHeaderSize + kPointSize * static_cast<size_t>(frame.points);
        if (size < start + 4 || data[start] > kMaxBulbs || size < start + 4 + 12 * static_cast<size_t>(data[start])) {
            return false;
        }
        frame.bulbs = data[start];
        for (int b = 0; b < frame.bulbs; ++b)
            for (int i = 0; i < 3; ++i) frame.bulb[b][i] = ReadF32(data + start + 4 + 12 * b + 4 * i);
    }
    *out = frame;
    return true;
}

}  // namespace body_protocol
