#pragma once

#include <atomic>
#include <climits>
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
// packet gave, so the phone can turn it by however much the head turned since (timewarp). SteamVR
// keeps that orientation in its frame timings, which it writes once a frame is presented: it is
// looked up as late as possible, when the frame leaves the encoder.
class VideoStreamer {
public:
    VideoStreamer();
    ~VideoStreamer();

    // Creates the GPU device. SteamVR must render on the GPU whose id ends up in `adapter_luid`.
    bool Start(int fps, int bitrate, uint64_t* adapter_luid);
    void Stop();

    // Each eye's frame spans `tan_drawn` (the tangent of half its angle), of which the phone shows
    // `tan_shown`: a margin for the timewarp to turn into.
    void SetFieldOfView(float tan_drawn, float tan_shown)
    {
        tan_drawn_ = tan_drawn;
        tan_shown_ = tan_shown;
    }
    // From the tracking thread, for each sensor packet: the head orientation it gave, unpredicted,
    // in SteamVR's raw space.
    void OnPhonePacket(uint32_t phone_ipv4, uint32_t seq, Quat orientation);
    void RequestKeyframe() { keyframe_requested_ = true; }

    // From the compositor (IVRVirtualDisplay). `frame_id`: SteamVR's for this frame.
    void OnPresent(vr::SharedTextureHandle_t texture, uint64_t frame_id);
    void OnWaitForPresent();

private:
    // A frame as SteamVR presented it, until its render pose is known.
    struct Presented {
        uint64_t id = 0;            // SteamVR's for the frame
        bool have_newest = false;   // the newest frame timing when it was presented
        uint32_t newest = 0;
        uint32_t seq = 0;           // the newest phone packet then...
        Quat orientation;           // ...and the head orientation it gave
        bool have_guess = false;    // the render pose, should no better be found later: the newest
        Quat guess;                 // timing's when it was presented
    };

    // The render pose of `frame` as a turn from its packet's orientation; false if not known.
    bool RenderTurn(const Presented& frame, Quat* turn);
    bool PhoneActive() const;
    void EncodeThread();
    void LogStats();

    std::unique_ptr<FrameCapture> capture_;
    int fps_ = 60;
    int bitrate_ = 20000000;

    std::atomic<uint32_t> phone_ipv4_{0};
    std::atomic<int64_t> phone_seen_ns_{0};
    std::atomic<float> tan_drawn_{1.0f};
    std::atomic<float> tan_shown_{1.0f};
    // The newest sensor packet and the orientation it gave.
    std::mutex phone_mutex_;
    uint32_t phone_seq_ = 0;
    Quat phone_orientation_;
    // The frame being presented: compositor thread.
    Presented presented_;
    std::atomic<bool> keyframe_requested_{true};

    // Latest captured frame, handed from the compositor thread to the encoder thread.
    std::mutex frame_mutex_;
    std::condition_variable frame_ready_;
    std::vector<uint8_t> frame_;
    int frame_width_ = 0;
    int frame_height_ = 0;
    Presented frame_presented_;
    bool frame_bgra_ = false;
    bool has_frame_ = false;

    std::atomic<bool> running_{false};
    std::thread encode_thread_;

    // Counters for the "Video:" log line, written from the tracking thread every few seconds so it
    // also shows up when the compositor or the encoder stop.
    std::atomic<int64_t> next_stats_ns_{0};
    std::atomic<int> presented_count_{0};
    std::atomic<int> copied_{0};
    std::atomic<int> encoded_{0};
    std::atomic<int> encoded_out_{0};
    std::atomic<int> keyframes_out_{0};
    std::atomic<int> packets_sent_{0};
    std::atomic<int> send_errors_{0};
    std::atomic<int64_t> bytes_sent_{0};
    std::atomic<int> last_luma_{-1};
    // How each frame's render pose was found: the timing with its index, the first timing written
    // after it was presented, the newest when it was presented, or none.
    std::atomic<int> poses_by_index_{0};
    std::atomic<int> poses_after_{0};
    std::atomic<int> poses_guessed_{0};
    std::atomic<int> poses_missing_{0};
    std::atomic<int> poses_reprojected_{0};  // of the found, those SteamVR turned itself
    std::atomic<int64_t> index_lead_min_{INT64_MAX};  // the newest timing's index minus the frame's,
    std::atomic<int64_t> index_lead_max_{INT64_MIN};  // when it was presented
    int logged_acquire_timeouts_ = 0;
};
