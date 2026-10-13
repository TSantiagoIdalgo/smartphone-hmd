#pragma once

#include <cstdint>

// Converts 8-bit RGBA or BGRA pixels to NV12 (BT.601, limited range), the input format of
// H.264 encoders. Width and height must be even. `y` holds width*height bytes; `uv` holds
// width*height/2 bytes of interleaved U and V at half resolution.
void RgbaToNv12(const uint8_t* src, int src_pitch, int width, int height, bool bgra, uint8_t* y, uint8_t* uv);
