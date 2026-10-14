#include "virtual_display.h"

#include <cmath>

#include "video_streamer.h"

VirtualDisplay::VirtualDisplay(VideoStreamer* streamer, float frequency)
    : streamer_(streamer), frame_seconds_(1.0 / frequency), start_(std::chrono::steady_clock::now())
{
}

void VirtualDisplay::Present(const vr::PresentInfo_t* present_info, uint32_t)
{
    streamer_->OnPresent(present_info->backbufferTextureHandle);
}

void VirtualDisplay::WaitForPresent()
{
    streamer_->OnWaitForPresent();
}

bool VirtualDisplay::GetTimeSinceLastVsync(float* seconds_since_last_vsync, uint64_t* frame_counter)
{
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
    const double frames = std::floor(elapsed / frame_seconds_);
    *seconds_since_last_vsync = static_cast<float>(elapsed - frames * frame_seconds_);
    *frame_counter = static_cast<uint64_t>(frames);
    return true;
}
