#pragma once

#include <cstddef>
#include <cstdint>

// Packets from the pose program on this PC (tools/body_tracker.py) to the driver: one camera
// frame's pose. Little-endian, with the phones' 20-byte header ("HMD1", type, 3 bytes padding,
// seq u32, timestamp i64 ns), then the same fields as the pose packet the program sends the
// camera phone:
//   20 u32 camera id   24 u16 width   26 u16 height (the upright picture, pixels)
//   28 f32 age: ms from the camera taking the frame to this packet leaving the program, so the
//      driver puts the frame on its own clock without sharing the program's
//   32 u8 flags (1 a person, 4 a Move's bulb)   33 u8 point count   34 u16 padding
//   36 f32 bulb x, 40 y (0-1 across the picture), 44 radius (over its width), when flagged
//   48 per point: x, y (0-1), z, visibility, presence (f32), MediaPipe's 33 pose points
//   then, flagged 32: u8 bulb count, 3 bytes padding, per bulb x, y, radius (f32) as the bulb
//      above: every lit bulb by a hand, for the controllers
namespace body_protocol {

constexpr uint16_t kPort = 9947;
constexpr uint8_t kTypeBodyFrame = 12;
constexpr uint8_t kFlagPose = 1;
constexpr uint8_t kFlagBall = 4;
constexpr uint8_t kFlagBulbs = 32;
constexpr int kMaxBulbs = 2;
constexpr int kPoints = 33;
constexpr size_t kHeaderSize = 48;
constexpr size_t kPointSize = 20;

struct Frame {
    uint32_t seq = 0;
    uint32_t camera = 0;
    int width = 0;
    int height = 0;
    double age_seconds = 0.0;
    uint8_t flags = 0;
    int points = 0;  // 0 when no one is in view
    float ball[3] = {};
    float point[kPoints][5] = {};
    int bulbs = 0;
    float bulb[kMaxBulbs][3] = {};
};

// False if the bytes are not a whole body frame.
bool Parse(const uint8_t* data, size_t size, Frame* out);

}  // namespace body_protocol
