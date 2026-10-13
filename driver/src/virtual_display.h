#pragma once

#include <chrono>

#include "openvr_driver.h"

class VideoStreamer;

// Receives the compositor's final frames instead of showing them in a desktop window, and gives
// SteamVR a steady virtual vsync to pace itself against.
class VirtualDisplay : public vr::IVRVirtualDisplay {
public:
    VirtualDisplay(VideoStreamer* streamer, float frequency);

    void Present(const vr::PresentInfo_t* present_info, uint32_t present_info_size) override;
    void WaitForPresent() override;
    bool GetTimeSinceLastVsync(float* seconds_since_last_vsync, uint64_t* frame_counter) override;

private:
    VideoStreamer* streamer_;
    double frame_seconds_;
    std::chrono::steady_clock::time_point start_;
};
