#include "video_protocol.h"

#include <cstring>

namespace video {
namespace {

void PutU16(uint8_t* p, uint16_t v)
{
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
}

void PutU32(uint8_t* p, uint32_t v)
{
    for (int i = 0; i < 4; ++i) {
        p[i] = static_cast<uint8_t>(v >> (8 * i));
    }
}

void PutF32(uint8_t* p, float f)
{
    uint32_t v;
    std::memcpy(&v, &f, 4);
    PutU32(p, v);
}

}  // namespace

void Packetize(const FrameInfo& info, const uint8_t* data, size_t size, const SendFunction& send)
{
    const size_t chunk_count = size == 0 ? 1 : (size + kMaxPayload - 1) / kMaxPayload;
    if (chunk_count > 0xFFFF) {
        return;  // a frame this large means something is badly wrong; drop it
    }
    uint8_t packet[kHeaderSize + kMaxPayload];
    std::memcpy(packet, "HMV3", 4);
    PutU32(packet + 4, info.frame_index);
    PutU16(packet + 10, static_cast<uint16_t>(chunk_count));
    packet[12] = (info.keyframe ? kFlagKeyframe : 0) | (info.has_render_turn ? kFlagRenderTurn : 0);
    packet[13] = static_cast<uint8_t>(info.sensor_seq);
    packet[14] = static_cast<uint8_t>(info.sensor_seq >> 8);
    packet[15] = static_cast<uint8_t>(info.sensor_seq >> 16);
    PutU16(packet + 16, info.width);
    PutU16(packet + 18, info.height);
    for (int i = 0; i < 4; ++i) {
        PutF32(packet + 20 + 4 * i, info.render_turn[i]);
    }
    PutF32(packet + 36, info.tan_drawn);
    PutF32(packet + 40, info.tan_shown);

    for (size_t chunk = 0; chunk < chunk_count; ++chunk) {
        const size_t offset = chunk * kMaxPayload;
        const size_t length = size - offset < kMaxPayload ? size - offset : kMaxPayload;
        PutU16(packet + 8, static_cast<uint16_t>(chunk));
        if (length > 0) {
            std::memcpy(packet + kHeaderSize, data + offset, length);
        }
        send(packet, kHeaderSize + length);
    }
}

bool StartsWithSps(const uint8_t* data, size_t size)
{
    // Skip the start code (00 00 01 or 00 00 00 01).
    size_t i = 0;
    while (i < size && data[i] == 0) {
        ++i;
    }
    if (i < 2 || i >= size || data[i] != 1 || i + 1 >= size) {
        return false;
    }
    return (data[i + 1] & 0x1F) == 7;
}

}  // namespace video
