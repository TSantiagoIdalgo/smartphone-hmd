// Small self-contained tests for the parser and the head tracker. Run with ctest.
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "chaperone.h"
#include "color_convert.h"
#include "head_position.h"
#include "head_tracker.h"
#include "protocol.h"
#include "video_protocol.h"
#include "vsync_clock.h"

namespace {

int g_failures = 0;

#define CHECK(cond)                                                     \
    do {                                                                \
        if (!(cond)) {                                                  \
            std::printf("%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
            ++g_failures;                                               \
        }                                                               \
    } while (0)

#define CHECK_NEAR(a, b, tol) CHECK(std::fabs((a) - (b)) <= (tol))

constexpr double kPi = 3.14159265358979;
constexpr int kRateHz = 500;
constexpr int64_t kStepNs = 1000000000 / kRateHz;

void PutU32(std::vector<uint8_t>& b, uint32_t v)
{
    for (int i = 0; i < 4; ++i) b.push_back(static_cast<uint8_t>(v >> (8 * i)));
}

void PutF32(std::vector<uint8_t>& b, float f)
{
    uint32_t v;
    std::memcpy(&v, &f, 4);
    PutU32(b, v);
}

std::vector<uint8_t> SensorBytes(uint32_t seq, int64_t ts, const float g[3], const float a[3], const float m[3])
{
    std::vector<uint8_t> b = {'H', 'M', 'D', '1', 1, 0, 0, 0};
    PutU32(b, seq);
    PutU32(b, static_cast<uint32_t>(ts));
    PutU32(b, static_cast<uint32_t>(static_cast<uint64_t>(ts) >> 32));
    for (int i = 0; i < 3; ++i) PutF32(b, g[i]);
    for (int i = 0; i < 3; ++i) PutF32(b, a[i]);
    for (int i = 0; i < 3; ++i) PutF32(b, m[i]);
    return b;
}

// Feeds `seconds` of samples with constant phone-frame gyro and accel.
struct Feeder {
    HeadTracker tracker;
    uint32_t seq = 0;
    int64_t ts = 1000000000;

    void Run(double seconds, float gx, float gy, float gz, float ax, float ay, float az)
    {
        const int n = static_cast<int>(seconds * kRateHz);
        for (int i = 0; i < n; ++i) {
            protocol::Packet p;
            p.seq = ++seq;
            p.timestamp_ns = (ts += kStepNs);
            p.gyro[0] = gx; p.gyro[1] = gy; p.gyro[2] = gz;
            p.accel[0] = ax; p.accel[1] = ay; p.accel[2] = az;
            tracker.AddSample(p);
        }
    }

    Vec3 Forward() const { return Rotate(tracker.Orientation(), Vec3{0, 0, -1}); }
};

// Upright in the headset, gravity reads +g along the phone's x axis.
constexpr float kG = 9.81f;

void TestParse()
{
    const float g[3] = {1.5f, -2.0f, 0.25f}, a[3] = {9.81f, 0, 0}, m[3] = {10, 20, 30};
    const int64_t ts = 123456789012345LL;
    std::vector<uint8_t> bytes = SensorBytes(42, ts, g, a, m);
    CHECK(bytes.size() == protocol::kSensorPacketSize);

    protocol::Packet p;
    CHECK(protocol::Parse(bytes.data(), bytes.size(), p));
    CHECK(p.type == protocol::PacketType::Sensors);
    CHECK(p.seq == 42);
    CHECK(p.timestamp_ns == ts);
    CHECK(p.gyro[0] == 1.5f && p.gyro[1] == -2.0f && p.gyro[2] == 0.25f);
    CHECK(p.accel[0] == 9.81f);
    CHECK(p.mag[2] == 30.0f);

    CHECK(!protocol::Parse(bytes.data(), bytes.size() - 1, p));  // truncated
    bytes[0] = 'X';
    CHECK(!protocol::Parse(bytes.data(), bytes.size(), p));      // wrong magic

    const uint8_t recenter[20] = {'H', 'M', 'D', '1', 2};
    CHECK(protocol::Parse(recenter, sizeof(recenter), p));
    CHECK(p.type == protocol::PacketType::Recenter);

    const uint8_t unknown[20] = {'H', 'M', 'D', '1', 9};
    CHECK(!protocol::Parse(unknown, sizeof(unknown), p));
}

void TestDiscoveryAndLatencyPackets()
{
    protocol::Packet p;
    const uint8_t discover[20] = {'H', 'M', 'D', '1', 4};
    CHECK(protocol::Parse(discover, sizeof(discover), p));
    CHECK(p.type == protocol::PacketType::Discover);

    // The PC's own answer is not a phone packet.
    uint8_t reply[protocol::kHeaderSize];
    CHECK(protocol::WritePcHere(reply) == protocol::kHeaderSize);
    CHECK(std::memcmp(reply, "HMD1", 4) == 0 && reply[4] == 5);
    CHECK(!protocol::Parse(reply, sizeof(reply), p));

    std::vector<uint8_t> latency = {'H', 'M', 'D', '1', 6, 0, 0, 0};
    PutU32(latency, 7);
    PutU32(latency, 0);
    PutU32(latency, 0);
    PutF32(latency, 42.5f);
    CHECK(latency.size() == protocol::kLatencyPacketSize);
    CHECK(protocol::Parse(latency.data(), latency.size(), p));
    CHECK(p.type == protocol::PacketType::Latency);
    CHECK(p.latency_ms == 42.5f);
    CHECK(!protocol::Parse(latency.data(), latency.size() - 1, p));  // truncated
}

void TestPredict()
{
    // Turning left (counter-clockwise seen from above, +y in OpenVR) at 90 degrees per second.
    const Quat start;
    const Vec3 omega{0, kPi / 2, 0};
    const Vec3 forward = Rotate(Predict(start, omega, 1.0), Vec3{0, 0, -1});
    CHECK_NEAR(forward.x, -1.0, 1e-9);  // a quarter turn later, looking left
    CHECK_NEAR(forward.z, 0.0, 1e-9);

    // Starting from a turned head, the prediction adds to it.
    const Quat turned = AxisAngle(Vec3{0, 1, 0}, kPi / 4);
    const Vec3 later = Rotate(Predict(turned, omega, 0.5), Vec3{0, 0, -1});
    CHECK_NEAR(later.x, -1.0, 1e-9);

    // No time or no motion: unchanged.
    const Quat same = Predict(turned, omega, 0.0);
    CHECK_NEAR(same.w, turned.w, 1e-12);
    const Quat still = Predict(turned, Vec3{}, 0.05);
    CHECK_NEAR(still.y, turned.y, 1e-12);
}

void TestStillUpright()
{
    Feeder f;
    f.Run(2.0, 0, 0, 0, kG, 0, 0);
    const Vec3 fwd = f.Forward();
    CHECK_NEAR(fwd.x, 0.0, 1e-3);
    CHECK_NEAR(fwd.y, 0.0, 1e-3);
    CHECK_NEAR(fwd.z, -1.0, 1e-3);
}

void TestTurnLeft()
{
    // Head turning left is a positive rotation around head +y, which is phone +x.
    Feeder f;
    f.Run(1.0, 1.0f, 0, 0, kG, 0, 0);
    const Vec3 fwd = f.Forward();
    CHECK_NEAR(fwd.x, -std::sin(1.0), 0.02);
    CHECK_NEAR(fwd.y, 0.0, 0.02);
    CHECK_NEAR(fwd.z, -std::cos(1.0), 0.02);
    // SteamVR gets the angular velocity in world axes.
    CHECK_NEAR(f.tracker.AngularVelocity().y, 1.0, 1e-3);
}

void TestRecenter()
{
    Feeder f;
    f.Run(1.0, 1.0f, 0, 0, kG, 0, 0);
    f.Run(0.2, 0, 0, 0, kG, 0, 0);
    f.tracker.Recenter();
    const Vec3 fwd = f.Forward();
    CHECK_NEAR(fwd.x, 0.0, 1e-6);
    CHECK_NEAR(fwd.z, -1.0, 1e-3);
}

void TestTiltFollowsGravity()
{
    // Looking 30 degrees down with no gyro: the accelerometer alone must bring the view down.
    const double alpha = 30.0 * kPi / 180.0;
    Feeder f;
    f.Run(10.0, 0, 0, 0, static_cast<float>(kG * std::cos(alpha)), 0, static_cast<float>(kG * std::sin(alpha)));
    const Vec3 fwd = f.Forward();
    CHECK_NEAR(fwd.y, -std::sin(alpha), 0.01);
    CHECK_NEAR(fwd.z, -std::cos(alpha), 0.01);
}

void TestGyroBiasIsLearned()
{
    // A constant 0.01 rad/s offset would drift 0.2 rad in 20 s if it were not removed.
    Feeder f;
    f.Run(20.0, 0.01f, 0, 0, kG, 0, 0);
    const Vec3 fwd = f.Forward();
    CHECK(std::fabs(std::atan2(-fwd.x, -fwd.z)) < 0.05);
}

void TestRejectsBadSamples()
{
    HeadTracker t;
    protocol::Packet p;
    p.seq = 10;
    p.timestamp_ns = 1000;
    p.accel[0] = kG;
    CHECK(t.AddSample(p));
    CHECK(!t.AddSample(p));  // duplicate

    p.seq = 9;
    p.timestamp_ns = 2000;
    CHECK(!t.AddSample(p));  // out of order

    p.seq = 11;
    p.gyro[0] = NAN;
    CHECK(!t.AddSample(p));  // not a number

    p.gyro[0] = 100.0f;
    CHECK(!t.AddSample(p));  // impossible rotation speed

    p.gyro[0] = 0.0f;
    CHECK(t.AddSample(p));

    // The phone app restarted and its sequence began again from a much lower number.
    p.seq = static_cast<uint32_t>(11 - 5000);
    p.timestamp_ns = 5000;
    CHECK(t.AddSample(p));
}

void TestPacketize()
{
    std::vector<uint8_t> frame(3000);
    for (size_t i = 0; i < frame.size(); ++i) frame[i] = static_cast<uint8_t>(i * 7);

    video::FrameInfo info;
    info.frame_index = 0x01020304;
    info.sensor_seq = 0xAB123456;
    info.keyframe = true;
    info.width = 1920;
    info.height = 1080;

    std::vector<std::vector<uint8_t>> packets;
    video::Packetize(info, frame.data(), frame.size(),
                     [&](const uint8_t* data, size_t size) { packets.emplace_back(data, data + size); });

    CHECK(packets.size() == 3);  // 1400 + 1400 + 200
    std::vector<uint8_t> rebuilt;
    for (size_t i = 0; i < packets.size(); ++i) {
        const std::vector<uint8_t>& p = packets[i];
        CHECK(std::memcmp(p.data(), "HMV1", 4) == 0);
        CHECK(p[4] == 0x04 && p[5] == 0x03 && p[6] == 0x02 && p[7] == 0x01);
        CHECK(p[8] == i && p[9] == 0);
        CHECK(p[10] == 3 && p[11] == 0);
        CHECK(p[12] == video::kFlagKeyframe);
        CHECK(p[13] == 0x56 && p[14] == 0x34 && p[15] == 0x12);  // low 24 bits of the sensor seq
        CHECK((p[16] | p[17] << 8) == 1920);
        CHECK((p[18] | p[19] << 8) == 1080);
        rebuilt.insert(rebuilt.end(), p.begin() + video::kHeaderSize, p.end());
    }
    CHECK(packets[0].size() == video::kHeaderSize + video::kMaxPayload);
    CHECK(packets[2].size() == video::kHeaderSize + 200);
    CHECK(rebuilt == frame);
}

void TestStartsWithSps()
{
    const uint8_t sps[] = {0, 0, 0, 1, 0x67, 0x42};
    const uint8_t sps_short_code[] = {0, 0, 1, 0x67};
    const uint8_t idr[] = {0, 0, 0, 1, 0x65, 0x88};
    CHECK(video::StartsWithSps(sps, sizeof(sps)));
    CHECK(video::StartsWithSps(sps_short_code, sizeof(sps_short_code)));
    CHECK(!video::StartsWithSps(idr, sizeof(idr)));
    CHECK(!video::StartsWithSps(sps, 3));
}

void TestRgbaToNv12()
{
    // 2x2 blocks: white, black, red, blue side by side (8x2 pixels), as RGBA and as BGRA.
    const uint8_t colors[4][3] = {{255, 255, 255}, {0, 0, 0}, {255, 0, 0}, {0, 0, 255}};
    const int width = 8, height = 2, pitch = width * 4 + 8;  // padded rows, like a mapped texture
    for (int bgra = 0; bgra < 2; ++bgra) {
        std::vector<uint8_t> src(pitch * height, 0xAB);
        for (int row = 0; row < height; ++row) {
            for (int col = 0; col < width; ++col) {
                const uint8_t* c = colors[col / 2];
                uint8_t* px = &src[row * pitch + col * 4];
                px[0] = bgra ? c[2] : c[0];
                px[1] = c[1];
                px[2] = bgra ? c[0] : c[2];
                px[3] = 255;
            }
        }
        std::vector<uint8_t> y(width * height), uv(width * height / 2);
        RgbaToNv12(src.data(), pitch, width, height, bgra != 0, y.data(), uv.data());

        CHECK(y[0] == 235 && y[width] == 235);  // white
        CHECK(uv[0] == 128 && uv[1] == 128);
        CHECK(y[2] == 16);                      // black
        CHECK(uv[2] == 128 && uv[3] == 128);
        CHECK(y[4] == 82);                      // red
        CHECK(uv[4] == 90 && uv[5] == 240);
        CHECK(y[6] == 41);                      // blue
        CHECK(uv[6] == 240 && uv[7] == 110);
    }
}

}  // namespace

void TestRgbaToNv12Parallel()
{
    // Odd band boundaries (66 rows on 4 threads) must give exactly the single-threaded result.
    const int width = 10, height = 66, pitch = width * 4;
    std::vector<uint8_t> src(pitch * height);
    for (size_t i = 0; i < src.size(); ++i) src[i] = static_cast<uint8_t>(i * 31 + 7);
    std::vector<uint8_t> y1(width * height), uv1(width * height / 2), y2(y1.size()), uv2(uv1.size());
    RgbaToNv12(src.data(), pitch, width, height, false, y1.data(), uv1.data());
    for (int threads : {1, 3, 4, 40}) {
        std::fill(y2.begin(), y2.end(), 0);
        std::fill(uv2.begin(), uv2.end(), 0);
        RgbaToNv12Parallel(src.data(), pitch, width, height, false, y2.data(), uv2.data(), threads);
        CHECK(y1 == y2);
        CHECK(uv1 == uv2);
    }
}

void TestMeanLuma()
{
    // Left half black (16), right half white (235): the sparse grid sees both halves equally.
    const int width = 64, height = 32;
    std::vector<uint8_t> y(width * height);
    for (int row = 0; row < height; ++row) {
        for (int col = 0; col < width; ++col) {
            y[row * width + col] = col < width / 2 ? 16 : 235;
        }
    }
    CHECK(MeanLuma(y.data(), width, height) == (16 + 235) / 2);
    CHECK(MeanLuma(y.data(), 8, 8) == 0);  // smaller than one grid step: nothing sampled
}

void TestVsyncClock()
{
    const double t = 1.0 / 60.0;
    VsyncClock clock(t);
    double seconds = 0.0;
    uint64_t counter = 0;

    // With nothing presented it is a plain 60 Hz clock.
    clock.SinceLastVsync(2.25 * t, &seconds, &counter);
    CHECK(counter == 2);
    CHECK_NEAR(seconds, 0.25 * t, 1e-9);

    // A frame presented mid-interval reports its vsync at once, still in the future.
    clock.FramePresented(2.5 * t);
    clock.SinceLastVsync(2.5 * t, &seconds, &counter);
    CHECK(counter == 3);
    CHECK_NEAR(seconds, -0.5 * t, 1e-9);

    // A second frame before that vsync waits for the next one; a third does not push further.
    clock.FramePresented(2.6 * t);
    clock.SinceLastVsync(2.6 * t, &seconds, &counter);
    CHECK(counter == 4);
    clock.FramePresented(2.7 * t);
    clock.SinceLastVsync(2.7 * t, &seconds, &counter);
    CHECK(counter == 4);
    CHECK_NEAR(seconds, -1.3 * t, 1e-9);

    // Without presents the count keeps up with time.
    clock.SinceLastVsync(7.1 * t, &seconds, &counter);
    CHECK(counter == 7);
    CHECK_NEAR(seconds, 0.1 * t, 1e-9);

    // At one present per interval the reported vsync moves by exactly one each time.
    for (int i = 10; i < 20; ++i) {
        clock.FramePresented((i + 0.8) * t);
        clock.SinceLastVsync((i + 0.8) * t, &seconds, &counter);
        CHECK(counter == static_cast<uint64_t>(i + 1));
    }
}

void TestChaperoneJson()
{
    std::string json = ChaperoneJson(kUniverseId, 1.7f);
    CHECK(json.find("\"universeID\": \"1397247300\"") != std::string::npos);
    CHECK(json.find("\"seated\": {\"translation\": [0, -1.700, 0]") != std::string::npos);
    CHECK(json.find("\"standing\": {\"translation\": [0, 0, 0]") != std::string::npos);
    // SteamVR rejects trailing commas.
    json.erase(std::remove_if(json.begin(), json.end(), [](char c) { return std::isspace(static_cast<unsigned char>(c)); }),
               json.end());
    CHECK(json.find(",]") == std::string::npos);
    CHECK(json.find(",}") == std::string::npos);
    CHECK(std::count(json.begin(), json.end(), '{') == std::count(json.begin(), json.end(), '}'));
    CHECK(std::count(json.begin(), json.end(), '[') == std::count(json.begin(), json.end(), ']'));
}

void TestRecenterTowards()
{
    Feeder f;
    f.Run(1.0, 1.0f, 0, 0, kG, 0, 0);  // turned 1 rad to the left
    const Quat before = f.tracker.Orientation();
    // Towards a target 30 degrees to the left of -z.
    const double target = 30.0 * kPi / 180.0;
    const Quat delta = f.tracker.RecenterTowards(Vec3{-std::sin(target), 0, -std::cos(target)});
    const Vec3 fwd = f.Forward();
    CHECK_NEAR(std::atan2(-fwd.x, -fwd.z), target, 1e-6);
    // The returned rotation is what was added on top of the old orientation.
    const Quat rebuilt = delta * before;
    const Quat after = f.tracker.Orientation();
    CHECK_NEAR(std::fabs(rebuilt.w * after.w + rebuilt.x * after.x + rebuilt.y * after.y + rebuilt.z * after.z), 1.0,
               1e-9);
    // Towards -z is the plain recenter.
    f.tracker.RecenterTowards(Vec3{0, 0, -1});
    CHECK_NEAR(f.Forward().x, 0.0, 1e-6);
}

void TestNeckModel()
{
    const Vec3 eyes{0.0, 1.7, 0.0};
    const Vec3 eyes_from_neck{0.0, 0.075, -0.08};
    // Looking straight ahead, the eyes are where they were asked to be.
    HeadPoseOut head = NeckModel(eyes, eyes_from_neck, Quat{}, Vec3{});
    CHECK_NEAR(Length(head.position - eyes), 0.0, 1e-12);
    // Turned around, they are on the other side of the neck.
    head = NeckModel(eyes, eyes_from_neck, AxisAngle(Vec3{0, 1, 0}, kPi), Vec3{});
    CHECK_NEAR(Length(head.position - Vec3{0.0, 1.7, 0.16}), 0.0, 1e-9);
    // Looking down, they drop below the neck and come back a little.
    head = NeckModel(eyes, eyes_from_neck, AxisAngle(Vec3{1, 0, 0}, -kPi / 2), Vec3{});
    CHECK_NEAR(Length(head.position - Vec3{0.0, 1.7 - 0.155, 0.005}), 0.0, 1e-9);
    // Turning left moves the eyes, in front of the neck, to the left.
    head = NeckModel(eyes, eyes_from_neck, Quat{}, Vec3{0, 1, 0});
    CHECK_NEAR(Length(head.velocity - Vec3{-0.08, 0.0, 0.0}), 0.0, 1e-12);
}

void TestMat3()
{
    Mat3 a;
    const double values[3][3] = {{4, 1, 0.5}, {1, 3, 0.2}, {0.5, 0.2, 2}};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) a.m[i][j] = values[i][j];
    Mat3 inverse;
    CHECK(Inverse(a, &inverse));
    const Mat3 identity = a * inverse;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) CHECK_NEAR(identity.m[i][j], i == j ? 1.0 : 0.0, 1e-12);
    CHECK(!Inverse(Outer(Vec3{1, 2, 3}, Vec3{1, 2, 3}), &inverse));  // rank one

    const Quat a_rot = AxisAngle(Vec3{0, 1, 0}, 0.2);
    const Quat b_rot = AxisAngle(Vec3{0, 1, 0}, 1.0);
    const Quat mid = Slerp(a_rot, b_rot, 0.5);
    CHECK_NEAR(mid.y, std::sin(0.3), 1e-9);
    CHECK_NEAR(Slerp(a_rot, b_rot, 1.0).w, b_rot.w, 1e-9);
}

// A simulated head: the neck pivot and the rotation over time. The phone reports the rotation
// as it happens; the camera sees the light at 60 frames/s and delivers each frame 40 ms later.
struct HeadMotion {
    Vec3 neck;
    Quat rotation;
    Vec3 angular_velocity;  // world
};

struct HeadRun {
    double rms = 0.0;        // eye error after the first second
    double max_error = 0.0;  // same
    double max_step = 0.0;   // largest change of the output between two 2 ms steps
    Vec3 last_eyes;
    Vec3 last_truth;
};

HeadPositionConfig TestHeadConfig()
{
    HeadPositionConfig config;
    config.eyes_from_light = Vec3{0.0, -0.10, 0.03};  // light 10 cm above, 3 cm behind the eyes
    config.eyes_from_neck = Vec3{0.0, 0.075, -0.08};
    return config;
}

struct HeadSimulation {
    HeadPositionConfig config = TestHeadConfig();
    HeadPositionEstimator estimator{TestHeadConfig()};
    Vec3 camera{0.0, 1.9, -1.8};
    double camera_delay = 0.040;
    // Standard deviation per frame along the camera ray and across it, m. Never zero: the estimator
    // treats a light that repeats exactly as lost.
    double depth_noise = 0.0001;
    double lateral_noise = 0.0001;
    bool interpolated = false;   // PSMoveServiceEx's 200 Hz stream between frames
    double time = 100.0;
    double last_frame = 100.0;
    std::mt19937 random{1234};
    Vec3 previous_frame_light, frame_light;
    double frame_arrival = 0.0;
    bool have_frame = false;
    bool frame_seen = false;

    // Box-Muller on the raw generator, so every compiler draws the same numbers.
    double Gaussian()
    {
        const double u = (static_cast<double>(random()) + 0.5) / 4294967296.0;
        const double v = static_cast<double>(random()) / 4294967296.0;
        return std::sqrt(-2.0 * std::log(u)) * std::cos(2.0 * kPi * v);
    }

    Vec3 Eyes(const HeadMotion& m) const { return m.neck + Rotate(m.rotation, config.eyes_from_neck); }

    Vec3 Light(const HeadMotion& m)
    {
        const Vec3 light = m.neck + Rotate(m.rotation, config.eyes_from_neck - config.eyes_from_light);
        Vec3 along = light - camera;
        along = along * (1.0 / Length(along));
        Vec3 side = Cross(along, Vec3{0, 1, 0});
        side = side * (1.0 / Length(side));
        const Vec3 up = Cross(side, along);
        return light + along * (depth_noise * Gaussian()) + side * (lateral_noise * Gaussian()) +
               up * (lateral_noise * Gaussian());
    }

    // Runs `seconds` of `motion(t)`, t from 0 at the start of this run; `visible(t)` false hides the
    // light. Errors count after `settle`. Camera frames keep one schedule across runs.
    HeadRun Run(double seconds, const std::function<HeadMotion(double)>& motion,
                const std::function<bool(double)>& visible = nullptr, double settle = 1.0,
                const std::function<void(double)>& each_step = nullptr)
    {
        estimator.SetCamera(true, camera, 1);
        HeadRun run;
        double sum = 0.0;
        int count = 0;
        bool have_output = false;
        Vec3 previous_output;
        const double start = time;
        for (int step = 0; step * 0.002 < seconds; ++step) {
            time += 0.002;
            const double t = time - start;
            const HeadMotion now = motion(t);
            estimator.AddOrientation(time, now.rotation);
            // Frames are exposed at multiples of 1/60 s and arrive camera_delay later.
            const double frame = std::floor((time - camera_delay) * 60.0) / 60.0;
            if (frame > last_frame) {
                last_frame = frame;
                const double frame_t = frame - start;
                frame_seen = !visible || visible(frame_t);
                previous_frame_light = have_frame ? frame_light : Light(motion(frame_t));
                frame_light = Light(motion(frame_t));
                frame_arrival = time;
                have_frame = true;
                if (!interpolated || !frame_seen) {
                    estimator.AddLight(time, frame_light, frame_seen, now.angular_velocity);
                }
            }
            // PSMoveServiceEx publishes every 5 ms, interpolating from the previous frame to the
            // newest one over one frame.
            if (interpolated && have_frame && frame_seen && std::floor(time / 0.005) != std::floor((time - 0.002) / 0.005)) {
                const double f = std::min((time - frame_arrival) * 60.0, 1.0);
                estimator.AddLight(time, previous_frame_light + (frame_light - previous_frame_light) * f, true,
                                   now.angular_velocity);
            }
            estimator.Tick(time, true);
            if (each_step) {
                each_step(time);
            }
            const Vec3 eyes = estimator.Output(time, now.rotation, now.angular_velocity).position;
            const Vec3 truth = Eyes(now);
            if (have_output) {
                run.max_step = std::max(run.max_step, Length(eyes - previous_output));
            }
            previous_output = eyes;
            have_output = true;
            if (t >= settle) {
                const double error = Length(eyes - truth);
                sum += error * error;
                ++count;
                run.max_error = std::max(run.max_error, error);
            }
            run.last_eyes = eyes;
            run.last_truth = truth;
        }
        run.rms = count > 0 ? std::sqrt(sum / count) : 0.0;
        return run;
    }
};

HeadMotion StandStill(double)
{
    return {Vec3{0.1, 1.55, 0.2}, Quat{}, Vec3{}};
}

void TestHeadNeverSeen()
{
    HeadPositionConfig config = TestHeadConfig();
    config.default_eyes = Vec3{0, 1.6, 0};
    HeadPositionEstimator estimator(config);
    CHECK(estimator.state() == HeadPositionEstimator::State::NeverSeen);
    const HeadPoseOut ahead = estimator.Output(1.0, Quat{}, Vec3{});
    CHECK_NEAR(ahead.position.y, 1.6, 1e-12);
    CHECK_NEAR(ahead.position.z, 0.0, 1e-12);
    // Looking down moves the eyes around our neck model.
    const Quat down = AxisAngle(Vec3{1, 0, 0}, -0.5);
    const Vec3 eyes = estimator.Output(1.0, down, Vec3{}).position;
    const Vec3 expected = config.default_eyes - config.eyes_from_neck + Rotate(down, config.eyes_from_neck);
    CHECK_NEAR(Length(eyes - expected), 0.0, 1e-12);

    // An impossible sample (eyes 3 m up) is ignored.
    estimator.AddLight(1.0, Vec3{0, 3.1, 0}, true, Vec3{});
    CHECK(estimator.state() == HeadPositionEstimator::State::NeverSeen);
}

void TestHeadStillLight()
{
    // Without noise the eyes end up exactly where the light says, for any head rotation.
    for (const Quat rotation : {Quat{}, AxisAngle(Vec3{0, 1, 0}, kPi / 2), AxisAngle(Vec3{1, 0, 0}, -0.5)}) {
        HeadSimulation sim;
        const HeadRun run = sim.Run(2.0, [&](double) { return HeadMotion{Vec3{0.1, 1.55, 0.2}, rotation, Vec3{}}; });
        CHECK(run.max_error < 0.001);
        CHECK(sim.estimator.state() == HeadPositionEstimator::State::Live);
    }
}

HeadMotion ShakeHead(double t)
{
    // "No" with the whole head: +-70 degrees at 0.9 Hz around a still neck.
    const double w = 2.0 * kPi * 0.9;
    const double amplitude = 70.0 * kPi / 180.0;
    return {Vec3{0.0, 1.55, 0.3}, AxisAngle(Vec3{0, 1, 0}, amplitude * std::sin(w * t)),
            Vec3{0, amplitude * w * std::cos(w * t), 0}};
}

void TestHeadTurnUsesPhone()
{
    HeadSimulation sim;
    const HeadRun run = sim.Run(4.0, ShakeHead);
    CHECK(run.rms < 0.003);

    // Putting the newest rotation on the light as the camera last saw it is far worse.
    HeadSimulation naive;
    double sum = 0.0;
    int count = 0;
    for (double t = 1.0; t < 4.0; t += 0.002) {
        const double seen = std::floor((t - naive.camera_delay) * 60.0) / 60.0;
        const HeadMotion now = ShakeHead(t);
        const Vec3 eyes = naive.Light(ShakeHead(seen)) + Rotate(now.rotation, naive.config.eyes_from_light);
        const double error = Length(eyes - naive.Eyes(now));
        sum += error * error;
        ++count;
    }
    CHECK(run.rms < 0.5 * std::sqrt(sum / count));
}

void TestHeadDelayMismatch()
{
    // The setting says 20 ms; the camera is really 40 ms late. The error grows with the head speed
    // times the 20 ms mismatch times the 11 cm the light sits in front of the neck, and no more.
    HeadPositionConfig config = TestHeadConfig();
    config.latency_seconds = 0.020;
    HeadSimulation sim;
    sim.estimator = HeadPositionEstimator(config);
    const HeadRun run = sim.Run(4.0, ShakeHead);
    CHECK(run.rms < 0.020);
}

HeadMotion Crouch(double t)
{
    // Stand, then crouch 40 cm in 0.8 s, then stay down.
    const double s = std::clamp((t - 1.5) / 0.8, 0.0, 1.0);
    return {Vec3{0.0, 1.55 - 0.40 * s * s * (3.0 - 2.0 * s), 0.2}, Quat{}, Vec3{}};
}

void TestHeadCrouch()
{
    HeadSimulation sim;
    CHECK(sim.Run(2.3, Crouch).max_error < 0.04);
    // Settled 0.3 s after the crouch ends.
    CHECK(sim.Run(0.5, [](double t) { return Crouch(t + 2.3); }, nullptr, 0.3).max_error < 0.01);
}

void TestHeadSmoothsDepthNoise()
{
    // One camera: 3 cm of noise along the ray, 2 mm across. The output must be much steadier along
    // the ray without getting noisier across it.
    HeadSimulation sim;
    sim.interpolated = true;
    sim.depth_noise = 0.03;
    sim.lateral_noise = 0.002;
    const Vec3 truth = sim.Eyes(StandStill(0));
    Vec3 along = truth - sim.camera;
    along = along * (1.0 / Length(along));
    double depth_sum = 0.0;
    double lateral_sum = 0.0;
    int count = 0;
    sim.Run(32.0, StandStill, nullptr, 1.0, [&](double time) {
        if (time < 102.0) {
            return;
        }
        const Vec3 error = sim.estimator.Output(time, Quat{}, Vec3{}).position - truth;
        const double depth = Dot(error, along);
        depth_sum += depth * depth;
        lateral_sum += Dot(error, error) - depth * depth;
        ++count;
    });
    // Predicting 40-50 ms ahead amplifies the small sideways noise a little; it must stay small.
    CHECK(std::sqrt(depth_sum / count) < 0.65 * 0.03);
    CHECK(std::sqrt(lateral_sum / count) < 2.0 * 0.002);
}

void TestHeadInterpolatedStream()
{
    // PSMoveServiceEx sends about 200 interpolated values per second from 60 frames. They must not
    // count as independent measurements, or the noise gets through.
    const auto depth_rms = [](bool interpolated) {
        HeadSimulation sim;
        sim.interpolated = interpolated;
        sim.depth_noise = 0.03;
        const HeadRun run = sim.Run(8.0, StandStill);
        return run.rms;
    };
    CHECK(depth_rms(true) < 1.05 * depth_rms(false));
}

void TestHeadOutlier()
{
    HeadSimulation sim;
    sim.Run(1.0, StandStill);
    const Vec3 before = sim.estimator.Output(sim.time, Quat{}, Vec3{}).position;
    sim.estimator.AddLight(sim.time + 0.001, sim.Light(StandStill(0)) + Vec3{0.4, 0, 0}, true, Vec3{});
    const Vec3 after = sim.estimator.Output(sim.time + 0.001, Quat{}, Vec3{}).position;
    CHECK(Length(after - before) < 0.005);
}

void TestHeadPersistentJump()
{
    // The head (or PSMoveServiceEx's frame) really moves 30 cm at once: start following within
    // 0.3 s, gliding rather than jumping.
    HeadSimulation sim;
    const auto jump = [](double t) {
        HeadMotion m = StandStill(t);
        if (t > 1.5) {
            m.neck = m.neck + Vec3{0.3, 0, 0};
        }
        return m;
    };
    sim.Run(1.5, jump);
    sim.estimator.TakeStats();
    const HeadRun early = sim.Run(0.04 + 0.3, [&](double t) { return jump(t + 1.5); }, nullptr, 0.0);
    CHECK(sim.estimator.TakeStats().resets == 1);
    const HeadRun late = sim.Run(0.5, [&](double t) { return jump(t + 1.84); }, nullptr, 0.4);
    CHECK(late.max_error < 0.01);
    CHECK(early.max_step < 0.02 && late.max_step < 0.02);
}

void TestHeadLossHoldsNeck()
{
    HeadSimulation sim;
    sim.Run(1.0, StandStill);
    const Vec3 neck = StandStill(0).neck;
    // Lost; the head turns 90 degrees in place. The eyes swing around the held neck.
    const auto turn = [&](double t) {
        const double yaw = std::min(t, 1.0) * kPi / 2;
        return HeadMotion{neck, AxisAngle(Vec3{0, 1, 0}, yaw), Vec3{0, t < 1.0 ? kPi / 2 : 0.0, 0}};
    };
    const HeadRun run = sim.Run(2.0, turn, [](double) { return false; }, 0.0);
    CHECK(sim.estimator.state() == HeadPositionEstimator::State::Holding);
    CHECK(run.max_error < 0.01);
    CHECK(std::isfinite(run.last_eyes.x) && std::isfinite(run.last_eyes.y) && std::isfinite(run.last_eyes.z));
}

void TestHeadStaleIsLoss()
{
    HeadPositionEstimator estimator(TestHeadConfig());
    estimator.AddLight(10.0, Vec3{0, 1.6, 0}, true, Vec3{});
    estimator.Tick(10.1, true);
    CHECK(estimator.state() == HeadPositionEstimator::State::Live);
    estimator.Tick(10.3, true);  // still "tracking", but nothing new
    CHECK(estimator.state() == HeadPositionEstimator::State::Holding);
    estimator.AddLight(10.31, Vec3{0, 1.6, 0.001}, true, Vec3{});
    CHECK(estimator.state() == HeadPositionEstimator::State::Live);
    estimator.Tick(10.32, false);  // PSMoveServiceEx went away
    CHECK(estimator.state() == HeadPositionEstimator::State::Holding);
}

void TestHeadReacquire()
{
    // Lost for a second while stepping 30 cm to the side; found again: glide over, no jump.
    HeadSimulation sim;
    const auto step_aside = [](double t) {
        HeadMotion m = StandStill(t);
        m.neck = m.neck + Vec3{0.3 * std::clamp(t - 1.2, 0.0, 0.6) / 0.6, 0, 0};
        return m;
    };
    const HeadRun run = sim.Run(2.8, step_aside, [](double t) { return t < 1.0 || t > 2.0; }, 2.0 + 0.04 + 0.6);
    CHECK(run.max_error < 0.01);
    CHECK(run.max_step < 0.02);
}

void TestHeadYawChangeKeepsLight()
{
    HeadSimulation sim;
    sim.Run(1.5, StandStill);
    const Quat before{};
    const Vec3 eyes_before = sim.estimator.Output(sim.time, before, Vec3{}).position;
    const Vec3 light = eyes_before - Rotate(before, sim.config.eyes_from_light);
    const Quat delta = AxisAngle(Vec3{0, 1, 0}, 0.6);
    sim.estimator.OnYawChange(delta, before);
    const Vec3 eyes_after = sim.estimator.Output(sim.time, delta * before, Vec3{}).position;
    CHECK_NEAR(Length(eyes_after - Rotate(delta * before, sim.config.eyes_from_light) - light), 0.0, 1e-9);
}

void TestHeadFreeze()
{
    HeadSimulation sim;
    const auto sway = [](double t) {
        HeadMotion m = StandStill(t);
        m.neck = m.neck + Vec3{0.15 * std::sin(2.0 * t), 0, 0};
        return m;
    };
    Vec3 frozen_at;
    double frozen_drift = 0.0;
    const HeadRun run = sim.Run(4.0, sway, nullptr, 1.0, [&](double time) {
        if (std::fabs(time - 101.5) < 0.001) {
            sim.estimator.SetFrozen(true, time);
            frozen_at = sim.estimator.Output(time, Quat{}, Vec3{}).position;
        }
        if (time > 101.5 && time < 102.5) {
            frozen_drift = std::max(frozen_drift, Length(sim.estimator.Output(time, Quat{}, Vec3{}).position - frozen_at));
        }
        if (std::fabs(time - 102.5) < 0.001) {
            sim.estimator.SetFrozen(false, time);
        }
    });
    CHECK(frozen_drift < 0.001);
    CHECK(run.max_step < 0.02);
    CHECK(Length(run.last_eyes - run.last_truth) < 0.01);
}

HeadMotion WalkSideways(double t)
{
    // Standing, then stepping to the right at 1 m/s from t = 1 s.
    HeadMotion m = StandStill(t);
    m.neck = m.neck + Vec3{std::max(t - 1.0, 0.0), 0, 0};
    return m;
}

void TestHeadDropoutWhileMoving()
{
    // One camera frame without the light while the head moves: no backwards slide, no jump.
    for (const bool interpolated : {false, true}) {
        HeadSimulation sim;
        sim.interpolated = interpolated;
        sim.Run(1.4, WalkSideways);
        sim.estimator.TakeStats();
        const HeadRun run = sim.Run(0.6, [](double t) { return WalkSideways(t + 1.4); },
                                    [](double t) { return std::fabs(t - 0.1) > 0.01; }, 0.0);
        const HeadPositionEstimator::Stats stats = sim.estimator.TakeStats();
        CHECK(stats.losses == 1);
        CHECK(stats.resets == 0);
        CHECK(run.max_step < 0.01);
        // Held for one frame plus the camera delay while walking at 1 m/s.
        CHECK(run.max_error < 0.08);
    }
}

void TestHeadUnfreezeWhileMoving()
{
    // Frozen while walking 1 m to the side, unfrozen still walking: glide over, no jump.
    HeadSimulation sim;
    const auto walk = [](double t) {
        HeadMotion m = StandStill(t);
        m.neck = m.neck + Vec3{0.8 * std::max(t - 1.0, 0.0), 0, 0};
        return m;
    };
    double max_step_after = 0.0;
    Vec3 previous;
    bool have_previous = false;
    sim.Run(3.5, walk, nullptr, 1.0, [&](double time) {
        if (std::fabs(time - 101.0) < 0.001) {
            sim.estimator.SetFrozen(true, time);
        }
        if (std::fabs(time - 102.25) < 0.001) {
            sim.estimator.SetFrozen(false, time);
        }
        if (time > 102.25) {
            const Vec3 eyes = sim.estimator.Output(time, Quat{}, Vec3{}).position;
            if (have_previous) {
                max_step_after = std::max(max_step_after, Length(eyes - previous));
            }
            previous = eyes;
            have_previous = true;
        }
    });
    CHECK(max_step_after < 0.03);
    CHECK(sim.estimator.state() == HeadPositionEstimator::State::Live);
}

void TestHeadRecenterFrozenBeforeSeen()
{
    // Frozen before the light was ever seen, recentered, unfrozen: the default eyes stay put.
    HeadPositionEstimator estimator(TestHeadConfig());
    estimator.SetFrozen(true, 1.0);
    const Quat before{};
    const Quat delta = AxisAngle(Vec3{0, 1, 0}, kPi / 2);
    estimator.OnYawChange(delta, before);
    const Vec3 frozen = estimator.Output(1.0, delta * before, Vec3{}).position;
    estimator.SetFrozen(false, 1.0);
    const Vec3 unfrozen = estimator.Output(1.0, delta * before, Vec3{}).position;
    CHECK(Length(unfrozen - frozen) < 1e-9);
}

void TestHeadIgnoresNonFinite()
{
    HeadSimulation sim;
    sim.Run(1.5, StandStill);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    sim.estimator.AddLight(sim.time + 0.001, Vec3{nan, 1.6, 0.0}, true, Vec3{});
    const Vec3 eyes = sim.estimator.Output(sim.time + 0.001, Quat{}, Vec3{}).position;
    CHECK(std::isfinite(eyes.x) && std::isfinite(eyes.y) && std::isfinite(eyes.z));
    CHECK(Length(eyes - sim.Eyes(StandStill(0))) < 0.005);
}

int main()
{
    TestParse();
    TestDiscoveryAndLatencyPackets();
    TestPredict();
    TestStillUpright();
    TestTurnLeft();
    TestRecenter();
    TestTiltFollowsGravity();
    TestGyroBiasIsLearned();
    TestRejectsBadSamples();
    TestPacketize();
    TestStartsWithSps();
    TestRgbaToNv12();
    TestRgbaToNv12Parallel();
    TestMeanLuma();
    TestVsyncClock();
    TestChaperoneJson();
    TestRecenterTowards();
    TestNeckModel();
    TestMat3();
    TestHeadNeverSeen();
    TestHeadStillLight();
    TestHeadTurnUsesPhone();
    TestHeadDelayMismatch();
    TestHeadCrouch();
    TestHeadSmoothsDepthNoise();
    TestHeadInterpolatedStream();
    TestHeadOutlier();
    TestHeadPersistentJump();
    TestHeadLossHoldsNeck();
    TestHeadStaleIsLoss();
    TestHeadReacquire();
    TestHeadYawChangeKeepsLight();
    TestHeadFreeze();
    TestHeadDropoutWhileMoving();
    TestHeadUnfreezeWhileMoving();
    TestHeadRecenterFrozenBeforeSeen();
    TestHeadIgnoresNonFinite();
    if (g_failures == 0) {
        std::printf("All tests passed\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
}
