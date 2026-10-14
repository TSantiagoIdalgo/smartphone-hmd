#include "video_streamer.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <thread>

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

void VideoStreamer::OnPhonePacket(uint32_t phone_ipv4)
{
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

void VideoStreamer::OnPresent(vr::SharedTextureHandle_t texture)
{
    if (PhoneActive()) {
        ++presented_;
        if (capture_->Copy(texture)) {
            ++copied_;
        }
    }
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
        frame_bgra_ = bgra;
        has_frame_ = true;
        frame_ready_.notify_one();
    });
}

void VideoStreamer::EncodeThread()
{
    CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    MFStartup(MF_VERSION, MFSTARTUP_NOSOCKET);

    UdpSocket socket;
    socket.OpenForSending();
    H264Encoder encoder;
    bool encoder_ready = false;
    std::vector<uint8_t> rgba;
    std::vector<uint8_t> nv12;
    uint32_t frame_index = 0;
    int64_t frame_count = 0;
    int64_t next_send_ns = 0;

    while (running_) {
        int width = 0, height = 0, source_pitch = 0;
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
        RgbaToNv12(rgba.data(), source_pitch, width, height, bgra, nv12.data(),
                   nv12.data() + static_cast<size_t>(width) * height);
        last_luma_ = MeanLuma(nv12.data(), width, height);

        const uint32_t phone = phone_ipv4_.load();
        const int64_t time_100ns = frame_count++ * 10000000LL / fps_;
        ++encoded_;
        encoder.Encode(nv12.data(), time_100ns, keyframe_requested_.exchange(false),
                       [&](const uint8_t* data, size_t size, bool keyframe) {
                           ++encoded_out_;
                           if (keyframe) {
                               ++keyframes_out_;
                           }
                           video::FrameInfo info;
                           info.frame_index = frame_index++;
                           info.keyframe = keyframe;
                           info.width = static_cast<uint16_t>(width);
                           info.height = static_cast<uint16_t>(height);
                           video::Packetize(info, data, size, [&](const uint8_t* packet, size_t packet_size) {
                               while (NowNs() < next_send_ns) {
                                   std::this_thread::yield();
                               }
                               next_send_ns = std::max(next_send_ns, NowNs()) +
                                              static_cast<int64_t>(packet_size) * 8 * 1000000000LL / kMaxSendBitsPerSecond;
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
              ip[0], ip[1], ip[2], ip[3], presented_.exchange(0), copied_.exchange(0), acquire_timeouts,
              encoded_.exchange(0), encoded_out_.exchange(0), keyframes_out_.exchange(0), packets_sent_.exchange(0),
              static_cast<long long>(bytes_sent_.exchange(0) / 1024), send_errors_.exchange(0), last_luma_.load());
}
