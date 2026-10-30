#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "math3d.h"
#include "openvr_driver.h"

class FrameCapture;

// Sends what SteamVR renders to the phone: captures each presented frame, encodes it as H.264 on a
// background thread and sends it over UDP to the address the phone's sensor packets come from.
// Only does work while the phone is sending sensor data.
//
// Each frame also carries the head orientation SteamVR drew it for, from the one the phone's newest
// packet gave, so the phone can turn it by however much the head turned since (timewarp).
class VideoStreamer {
public:
    VideoStreamer();
    ~VideoStreamer();

    // Creates the GPU device. SteamVR must render on the GPU whose id ends up in `adapter_luid`.
    bool Start(int fps, int bitrate, uint64_t* adapter_luid);
    void Stop();

    // Each eye's frame spans this tangent of half its angle.
    void SetFieldOfView(float tan_half_fov) { tan_half_fov_ = tan_half_fov; }
    // From the tracking thread, for each sensor packet: the head orientation it gave, unpredicted,
    // in SteamVR's raw space.
    void OnPhonePacket(uint32_t phone_ipv4, uint32_t seq, Quat orientation);
    void RequestKeyframe() { keyframe_requested_ = true; }

    // From the compositor (IVRVirtualDisplay). `frame_id`: SteamVR's for this frame.
    void OnPresent(vr::SharedTextureHandle_t texture, uint64_t frame_id);
    void OnWaitForPresent();

private:
    bool PhoneActive() const;
    void EncodeThread();
    void LogStats();

    std::unique_ptr<FrameCapture> capture_;
    int fps_ = 60;
    int bitrate_ = 20000000;

    std::atomic<uint32_t> phone_ipv4_{0};
    std::atomic<int64_t> phone_seen_ns_{0};
    std::atomic<float> tan_half_fov_{1.0f};
    // The newest sensor packet and the orientation it gave.
    std::mutex phone_mutex_;
    uint32_t phone_seq_ = 0;
    Quat phone_orientation_;
    // The frame being presented: compositor thread.
    uint32_t presented_seq_ = 0;
    bool presented_has_turn_ = false;
    Quat presented_turn_;
    std::atomic<bool> keyframe_requested_{true};

    // Latest captured frame, handed from the compositor thread to the encoder thread.
    std::mutex frame_mutex_;
    std::condition_variable frame_ready_;
    std::vector<uint8_t> frame_;
    int frame_width_ = 0;
    int frame_height_ = 0;
    uint32_t frame_seq_ = 0;
    bool frame_has_turn_ = false;
    Quat frame_turn_;
    bool frame_bgra_ = false;
    bool has_frame_ = false;

    std::atomic<bool> running_{false};
    std::thread encode_thread_;

    // Counters for the "Video:" log line, written from the tracking thread every few seconds so it
    // also shows up when the compositor or the encoder stop.
    std::atomic<int64_t> next_stats_ns_{0};
    std::atomic<int> presented_{0};
    std::atomic<int> copied_{0};
    std::atomic<int> encoded_{0};
    std::atomic<int> encoded_out_{0};
    std::atomic<int> keyframes_out_{0};
    std::atomic<int> packets_sent_{0};
    std::atomic<int> send_errors_{0};
    std::atomic<int64_t> bytes_sent_{0};
    std::atomic<int> last_luma_{-1};
    std::atomic<int> turns_found_{0};    // frames whose render pose SteamVR gave...
    std::atomic<int> turns_matched_{0};  // ...for this very frame (not the newest one it had)
    int logged_acquire_timeouts_ = 0;
};
