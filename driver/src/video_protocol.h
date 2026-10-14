#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

// UDP video packets from the PC to the phone. Each H.264 frame is split into chunks of at most
// kMaxPayload bytes, each with this 20-byte little-endian header:
//   0  char[4] "HMV1"
//   4  u32     frame_index
//   8  u16     chunk_index
//  10  u16     chunk_count
//  12  u8      flags (bit 0: keyframe)
//  13  u24     sensor_seq: low 24 bits of the newest phone packet's seq when SteamVR presented
//              this frame, so the phone can time the whole round trip (0 = unknown)
//  16  u16     width
//  18  u16     height
// Every chunk except the last one carries exactly kMaxPayload bytes.
namespace video {

constexpr uint16_t kVideoPort = 9945;
constexpr size_t kHeaderSize = 20;
constexpr size_t kMaxPayload = 1400;
constexpr uint8_t kFlagKeyframe = 1;

struct FrameInfo {
    uint32_t frame_index = 0;
    uint32_t sensor_seq = 0;
    bool keyframe = false;
    uint16_t width = 0;
    uint16_t height = 0;
};

using SendFunction = std::function<void(const uint8_t* data, size_t size)>;

// Splits one encoded frame into packets and hands each one to `send`.
void Packetize(const FrameInfo& info, const uint8_t* data, size_t size, const SendFunction& send);

// True if the Annex-B stream starts with a sequence parameter set (NAL type 7).
bool StartsWithSps(const uint8_t* data, size_t size);

}  // namespace video
