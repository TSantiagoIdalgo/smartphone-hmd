#include "video_streamer.h"

#include <algorithm>
#include <array>
#include <climits>
#include <chrono>
#include <cstring>
#include <thread>

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <mmsystem.h>
#include <objbase.h>
#include <mfapi.h>

#include "color_convert.h"
#include "driver_log.h"
#include "frame_capture.h"
#include "h264_encoder.h"
#include "udp_socket.h"
#include "video_protocol.h"

namespace {

int64_t NowNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// Packets of a frame are spread out to this rate. A keyframe sent in one burst from a wired PC can
// overflow the Wi-Fi access point's queue, and losing one packet loses the whole frame.
constexpr int64_t kMaxSendBitsPerSecond = 200000000;

constexpr int64_t kStatsIntervalNs = 5000000000LL;

// Converting a phone-sized frame to NV12 takes several milliseconds on one core.
constexpr int kConvertThreads = 4;

}  // namespace

VideoStreamer::VideoStreamer() : capture_(std::make_unique<FrameCapture>()) {}

VideoStreamer::~VideoStreamer()
{
    Stop();
}

bool VideoStreamer::Start(int fps, int bitrate, uint64_t* adapter_luid)
{
    fps_ = fps;
    bitrate_ = bitrate;
    LUID luid{};
    if (!capture_->Init(&luid)) {
        return false;
    }
    *adapter_luid = (static_cast<uint64_t>(static_cast<uint32_t>(luid.HighPart)) << 32) | luid.LowPart;
    running_ = true;
    encode_thread_ = std::thread(&VideoStreamer::EncodeThread, this);
    return true;
}

void VideoStreamer::Stop()
{
    if (running_.exchange(false)) {
        frame_ready_.notify_all();
        if (encode_thread_.joinable()) {
            encode_thread_.join();
        }
    }
}

void VideoStreamer::OnPhonePacket(uint32_t phone_ipv4, uint32_t seq, Quat orientation)
{
    {
        std::lock_guard<std::mutex> lock(phone_mutex_);
        phone_seq_ = seq;
        phone_orientation_ = orientation;
    }
    const bool was_active = PhoneActive();
    if (phone_ipv4_.exchange(phone_ipv4) != phone_ipv4 || !was_active) {
        keyframe_requested_ = true;  // a phone that (re)connects needs a keyframe to start decoding
    }
    const int64_t now = NowNs();
    phone_seen_ns_ = now;
    if (now >= next_stats_ns_) {
        next_stats_ns_ = now + kStatsIntervalNs;
        LogStats();
    }
}

bool VideoStreamer::PhoneActive() const
{
    return NowNs() - phone_seen_ns_.load() < 1000000000LL;
}

namespace {

constexpr uint32_t kTimings = 8;
constexpr uint32_t kReprojected = vr::VRCompositor_ReprojectionReason_Cpu | vr::VRCompositor_ReprojectionReason_Gpu |
                                  vr::VRCompositor_ReprojectionAsync | vr::VRCompositor_ReprojectionMotion;

uint32_t ReadTimings(vr::Compositor_FrameTiming (&timings)[kTimings])
{
    timings[0].m_nSize = sizeof(vr::Compositor_FrameTiming);
    return std::min(vr::VRServerDriverHost()->GetFrameTimings(timings, kTimings), kTimings);
}

Quat PoseRotation(const vr::Compositor_FrameTiming& timing)
{
    const vr::HmdMatrix34_t& m = timing.m_HmdPose.mDeviceToAbsoluteTracking;
    Mat3 rotation;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            rotation.m[r][c] = m.m[r][c];
        }
    }
    return Mat3ToQuat(rotation);
}

}  // namespace

void VideoStreamer::OnPresent(vr::SharedTextureHandle_t texture, uint64_t frame_id)
{
    if (!PhoneActive()) {
        return;
    }
    ++presented_count_;
    if (!capture_->Copy(texture)) {
        return;
    }
    ++copied_;
    Presented frame;
    frame.id = frame_id;
    {
        std::lock_guard<std::mutex> lock(phone_mutex_);
        frame.seq = phone_seq_;
        frame.orientation = phone_orientation_;
    }
    vr::Compositor_FrameTiming timings[kTimings] = {};
    const uint32_t count = ReadTimings(timings);
    for (uint32_t i = 0; i < count; ++i) {
        if (!frame.have_newest || timings[i].m_nFrameIndex > frame.newest) {
            frame.have_newest = true;
            frame.newest = timings[i].m_nFrameIndex;
        }
        if (timings[i].m_HmdPose.bPoseIsValid) {
            frame.have_guess = true;
            frame.guess = PoseRotation(timings[i]);  // the newest valid one, oldest first
        }
    }
    if (frame.have_newest) {
        const int64_t lead = static_cast<int64_t>(frame.newest) - static_cast<int64_t>(frame_id);
        int64_t low = index_lead_min_.load(), high = index_lead_max_.load();
        while (lead < low && !index_lead_min_.compare_exchange_weak(low, lead)) {
        }
        while (lead > high && !index_lead_max_.compare_exchange_weak(high, lead)) {
        }
    }
    presented_ = frame;
}

// The timing with the frame's own index if there is one; else the first written after it was
// presented; else the newest when it was presented.
bool VideoStreamer::RenderTurn(const Presented& frame, Quat* turn)
{
    vr::Compositor_FrameTiming timings[kTimings] = {};
    const uint32_t count = ReadTimings(timings);
    const vr::Compositor_FrameTiming* found = nullptr;
    for (uint32_t i = 0; i < count && !found; ++i) {
        if (timings[i].m_HmdPose.bPoseIsValid && timings[i].m_nFrameIndex == frame.id) {
            found = &timings[i];
            ++poses_by_index_;
        }
    }
    for (uint32_t i = 0; i < count && !found && frame.have_newest; ++i) {
        if (timings[i].m_HmdPose.bPoseIsValid && timings[i].m_nFrameIndex == frame.newest + 1) {
            found = &timings[i];
            ++poses_after_;
        }
    }
    if (found) {
        poses_reprojected_ += (found->m_nReprojectionFlags & kReprojected) != 0;
        *turn = Normalize(Conjugate(frame.orientation) * PoseRotation(*found));
        return true;
    }
    if (frame.have_guess) {
        ++poses_guessed_;
        *turn = Normalize(Conjugate(frame.orientation) * frame.guess);
        return true;
    }
    ++poses_missing_;
    return false;
}

void VideoStreamer::OnWaitForPresent()
{
    capture_->Read([this](const uint8_t* pixels, int pitch, int width, int height, bool bgra) {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        const size_t row_bytes = static_cast<size_t>(width) * 4;
        frame_.resize(row_bytes * height);
        for (int row = 0; row < height; ++row) {
            std::memcpy(frame_.data() + row_bytes * row, pixels + static_cast<size_t>(pitch) * row, row_bytes);
        }
        frame_width_ = width;
        frame_height_ = height;
        frame_presented_ = presented_;
        frame_bgra_ = bgra;
        has_frame_ = true;
        frame_ready_.notify_one();
    });
}

void VideoStreamer::EncodeThread()
{
    CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    MFStartup(MF_VERSION, MFSTARTUP_NOSOCKET);
    // 1 ms sleeps instead of Windows' default 15.6 ms, for the short waits on the encoder.
    timeBeginPeriod(1);

    UdpSocket socket;
    socket.OpenForSending();
    H264Encoder encoder;
    bool encoder_ready = false;
    std::vector<uint8_t> rgba;
    std::vector<uint8_t> nv12;
    uint32_t frame_index = 0;
    int64_t frame_count = 0;
    int64_t next_send_ns = 0;
    // Recent frames as SteamVR presented them, by the time stamp the encoder hands back.
    std::array<std::pair<int64_t, Presented>, 8> sent{};

    while (running_) {
        int width = 0, height = 0, source_pitch = 0;
        Presented presented;
        bool bgra = false;
        {
            std::unique_lock<std::mutex> lock(frame_mutex_);
            frame_ready_.wait_for(lock, std::chrono::milliseconds(100), [this] { return has_frame_ || !running_; });
            if (!has_frame_) {
                continue;
            }
            rgba.swap(frame_);
            width = frame_width_;
            height = frame_height_;
            source_pitch = width * 4;
            presented = frame_presented_;
            bgra = frame_bgra_;
            has_frame_ = false;
        }

        // Encoders need even sizes; drop the last row or column if needed.
        width &= ~1;
        height &= ~1;
        if (width == 0 || height == 0) {
            continue;
        }
        if (!encoder_ready || encoder.Width() != width || encoder.Height() != height) {
            encoder_ready = encoder.Init(width, height, fps_, bitrate_);
            keyframe_requested_ = true;
            if (!encoder_ready) {
                continue;
            }
        }

        nv12.resize(static_cast<size_t>(width) * height * 3 / 2);
        RgbaToNv12Parallel(rgba.data(), source_pitch, width, height, bgra, nv12.data(),
                           nv12.data() + static_cast<size_t>(width) * height, kConvertThreads);
        last_luma_ = MeanLuma(nv12.data(), width, height);

        const uint32_t phone = phone_ipv4_.load();
        const int64_t time_100ns = frame_count * 10000000LL / fps_;
        sent[frame_count % sent.size()] = {time_100ns, presented};
        ++frame_count;
        ++encoded_;
        encoder.Encode(nv12.data(), time_100ns, keyframe_requested_.exchange(false),
                       [&](const uint8_t* data, size_t size, bool keyframe, int64_t output_time_100ns) {
                           ++encoded_out_;
                           if (keyframe) {
                               ++keyframes_out_;
                           }
                           video::FrameInfo info;
                           info.frame_index = frame_index++;
                           for (const auto& entry : sent) {
                               Quat turn;
                               if (entry.first == output_time_100ns) {
                                   info.sensor_seq = entry.second.seq;
                                   info.has_render_turn = RenderTurn(entry.second, &turn);
                                   info.render_turn[0] = static_cast<float>(turn.w);
                                   info.render_turn[1] = static_cast<float>(turn.x);
                                   info.render_turn[2] = static_cast<float>(turn.y);
                                   info.render_turn[3] = static_cast<float>(turn.z);
                               }
                           }
                           info.tan_drawn = tan_drawn_;
                           info.tan_shown = tan_shown_;
                           info.keyframe = keyframe;
                           info.width = static_cast<uint16_t>(width);
                           info.height = static_cast<uint16_t>(height);
                           video::Packetize(info, data, size, [&](const uint8_t* packet, size_t packet_size) {
                               int64_t now = NowNs();
                               while (now < next_send_ns) {
                                   std::this_thread::yield();
                                   now = NowNs();
                               }
                               next_send_ns = now + static_cast<int64_t>(packet_size) * 8 * 1000000000LL /
                                                        kMaxSendBitsPerSecond;
                               if (socket.SendTo(phone, video::kVideoPort, packet, packet_size)) {
                                   ++packets_sent_;
                                   bytes_sent_ += static_cast<int64_t>(packet_size);
                               } else {
                                   ++send_errors_;
                               }
                           });
                       });
    }

    encoder.Shutdown();
    timeEndPeriod(1);
    MFShutdown();
    CoUninitialize();
}

// One line every few seconds while the phone is connected, so vrserver.txt shows which stage of
// the video path stops: capture (presented/copied), encoder (in/out) or network (sent/errors).
// A brightness near 16 means the captured frames are black.
void VideoStreamer::LogStats()
{
    const int acquire_timeouts = capture_->AcquireTimeouts() - logged_acquire_timeouts_;
    logged_acquire_timeouts_ += acquire_timeouts;
    const uint32_t phone = phone_ipv4_.load();
    const uint8_t* ip = reinterpret_cast<const uint8_t*>(&phone);
    DriverLog("Video: phone %u.%u.%u.%u, %d presented, %d copied, %d texture busy, %d encoded, %d out (%d key), "
              "%d packets, %lld KB sent, %d send errors, brightness %d",
              ip[0], ip[1], ip[2], ip[3], presented_count_.exchange(0), copied_.exchange(0), acquire_timeouts,
              encoded_.exchange(0), encoded_out_.exchange(0), keyframes_out_.exchange(0), packets_sent_.exchange(0),
              static_cast<long long>(bytes_sent_.exchange(0) / 1024), send_errors_.exchange(0), last_luma_.load());
    const int64_t lead_min = index_lead_min_.exchange(INT64_MAX), lead_max = index_lead_max_.exchange(INT64_MIN);
    DriverLog("Video: the pose each frame was drawn for, from SteamVR's timings: %d by the frame's index, %d the first "
              "written after it was presented, %d the newest then (a guess), %d none; %d of those found reprojected "
              "by SteamVR; when presented, the newest timing's index was the frame's %lld to %lld",
              poses_by_index_.exchange(0), poses_after_.exchange(0), poses_guessed_.exchange(0),
              poses_missing_.exchange(0), poses_reprojected_.exchange(0),
              static_cast<long long>(lead_min == INT64_MAX ? 0 : lead_min),
              static_cast<long long>(lead_max == INT64_MIN ? 0 : lead_max));
}
