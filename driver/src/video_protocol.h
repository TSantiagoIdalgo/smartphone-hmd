#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

// UDP video packets from the PC to the phone. Each H.264 frame is split into chunks of at most
// kMaxPayload bytes, each with this 44-byte little-endian header:
//   0  char[4] "HMV3"
//   4  u32     frame_index
//   8  u16     chunk_index
//  10  u16     chunk_count
//  12  u8      flags (bit 0: keyframe; bit 1: render_turn is known)
//  13  u24     sensor_seq: low 24 bits of the newest phone packet's seq when SteamVR presented
//              this frame, so the phone can time the whole round trip (0 = unknown)
//  16  u16     width
//  18  u16     height
//  20  f32[4]  render_turn (w, x, y, z): the head orientation the frame was drawn for, from the
//              one the PC had from packet sensor_seq, in head axes (+x right, +y up, +z back).
//              With its own record of that packet's moment, the phone turns the frame by however
//              much the head turned since (timewarp).
//  36  f32     tan_drawn: each eye's frame spans this tangent of half its angle, across and up...
//  40  f32     tan_shown: ...of which the phone shows this much; the rest is a margin for the
//              timewarp to turn into
// Every chunk except the last one carries exactly kMaxPayload bytes.
namespace video {

constexpr uint16_t kVideoPort = 9945;
constexpr size_t kHeaderSize = 44;
constexpr size_t kMaxPayload = 1400;
constexpr uint8_t kFlagKeyframe = 1;
constexpr uint8_t kFlagRenderTurn = 2;

struct FrameInfo {
    uint32_t frame_index = 0;
    uint32_t sensor_seq = 0;
    bool keyframe = false;
    uint16_t width = 0;
    uint16_t height = 0;
    bool has_render_turn = false;
    float render_turn[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // w, x, y, z
    float tan_drawn = 1.0f;
    float tan_shown = 1.0f;
};

using SendFunction = std::function<void(const uint8_t* data, size_t size)>;

// Splits one encoded frame into packets and hands each one to `send`.
void Packetize(const FrameInfo& info, const uint8_t* data, size_t size, const SendFunction& send);

// True if the Annex-B stream starts with a sequence parameter set (NAL type 7).
bool StartsWithSps(const uint8_t* data, size_t size);

}  // namespace video
