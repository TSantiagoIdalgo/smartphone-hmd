#include "color_convert.h"

#include <thread>
#include <vector>

namespace {

inline uint8_t LumaOf(int r, int g, int b)
{
    return static_cast<uint8_t>(((66 * r + 129 * g + 25 * b + 128) >> 8) + 16);
}

}  // namespace

void RgbaToNv12(const uint8_t* src, int src_pitch, int width, int height, bool bgra, uint8_t* y, uint8_t* uv)
{
    const int r_index = bgra ? 2 : 0;
    const int b_index = bgra ? 0 : 2;

    for (int row = 0; row < height; row += 2) {
        const uint8_t* top = src + static_cast<int64_t>(row) * src_pitch;
        const uint8_t* bottom = top + src_pitch;
        uint8_t* y_top = y + static_cast<int64_t>(row) * width;
        uint8_t* y_bottom = y_top + width;
        uint8_t* uv_row = uv + static_cast<int64_t>(row / 2) * width;

        for (int col = 0; col < width; col += 2) {
            int r_sum = 0, g_sum = 0, b_sum = 0;
            const uint8_t* pixels[4] = {top + col * 4, top + col * 4 + 4, bottom + col * 4, bottom + col * 4 + 4};
            uint8_t* lumas[4] = {y_top + col, y_top + col + 1, y_bottom + col, y_bottom + col + 1};
            for (int i = 0; i < 4; ++i) {
                const int r = pixels[i][r_index];
                const int g = pixels[i][1];
                const int b = pixels[i][b_index];
                *lumas[i] = LumaOf(r, g, b);
                r_sum += r;
                g_sum += g;
                b_sum += b;
            }
            const int r = r_sum / 4, g = g_sum / 4, b = b_sum / 4;
            uv_row[col] = static_cast<uint8_t>(((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128);
            uv_row[col + 1] = static_cast<uint8_t>(((112 * r - 94 * g - 18 * b + 128) >> 8) + 128);
        }
    }
}

void RgbaToNv12Parallel(const uint8_t* src, int src_pitch, int width, int height, bool bgra, uint8_t* y, uint8_t* uv,
                        int threads)
{
    if (threads < 1) {
        threads = 1;
    }
    // Bands start on even rows, since each pass converts two rows.
    const int band = ((height / 2 + threads - 1) / threads) * 2;
    std::vector<std::thread> workers;
    for (int first = band; first < height; first += band) {
        const int rows = height - first < band ? height - first : band;
        workers.emplace_back(RgbaToNv12, src + static_cast<int64_t>(first) * src_pitch, src_pitch, width, rows, bgra,
                             y + static_cast<int64_t>(first) * width, uv + static_cast<int64_t>(first / 2) * width);
    }
    RgbaToNv12(src, src_pitch, width, band < height ? band : height, bgra, y, uv);
    for (std::thread& worker : workers) {
        worker.join();
    }
}

int MeanLuma(const uint8_t* y, int width, int height)
{
    const int step = 16;
    int64_t sum = 0;
    int64_t count = 0;
    for (int row = step / 2; row < height; row += step) {
        for (int col = step / 2; col < width; col += step) {
            sum += y[static_cast<int64_t>(row) * width + col];
            ++count;
        }
    }
    return count > 0 ? static_cast<int>(sum / count) : 0;
}
