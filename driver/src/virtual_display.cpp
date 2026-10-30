#include "virtual_display.h"

#include "video_streamer.h"

VirtualDisplay::VirtualDisplay(VideoStreamer* streamer, float frequency)
    : streamer_(streamer), start_(std::chrono::steady_clock::now()), vsync_(1.0 / frequency)
{
}

double VirtualDisplay::Now() const
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
}

void VirtualDisplay::Present(const vr::PresentInfo_t* present_info, uint32_t)
{
    streamer_->OnPresent(present_info->backbufferTextureHandle, present_info->nFrameId);
}

void VirtualDisplay::WaitForPresent()
{
    streamer_->OnWaitForPresent();
    std::lock_guard<std::mutex> lock(vsync_mutex_);
    vsync_.FramePresented(Now());
}

bool VirtualDisplay::GetTimeSinceLastVsync(float* seconds_since_last_vsync, uint64_t* frame_counter)
{
    double seconds = 0.0;
    std::lock_guard<std::mutex> lock(vsync_mutex_);
    vsync_.SinceLastVsync(Now(), &seconds, frame_counter);
    *seconds_since_last_vsync = static_cast<float>(seconds);
    return true;
}
