// Small self-contained tests for the parser and the head tracker. Run with ctest.
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "chaperone.h"
#include "color_convert.h"
#include "camera_model.h"
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
    float mag[3] = {0, 0, 0};

    void Run(double seconds, float gx, float gy, float gz, float ax, float ay, float az)
    {
        const int n = static_cast<int>(seconds * kRateHz);
        for (int i = 0; i < n; ++i) {
            protocol::Packet p;
            p.seq = ++seq;
            p.timestamp_ns = (ts += kStepNs);
            p.gyro[0] = gx; p.gyro[1] = gy; p.gyro[2] = gz;
            p.accel[0] = ax; p.accel[1] = ay; p.accel[2] = az;
            for (int k = 0; k < 3; ++k) p.mag[k] = mag[k];
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

// The phone is a little shaken (so the gyro bias is not learned) and its gyro reads 0.02 rad/s
// of yaw too much. Field: 40 uT down and 20 uT horizontal, in phone axes.
double YawAfterDrift(double gain, double seconds)
{
    Feeder f;
    f.tracker.SetMagnetometerGain(gain);
    f.mag[0] = -40.0f;
    f.mag[2] = -20.0f;
    f.Run(seconds, 0.02f, 0, 0, kG * 1.06f, 0, 0);
    const Vec3 fwd = f.Forward();
    return std::atan2(-fwd.x, -fwd.z);
}

void TestMagnetometerHoldsYaw()
{
    CHECK(std::fabs(YawAfterDrift(0.0, 60.0)) > 1.0);
    CHECK(std::fabs(YawAfterDrift(0.2, 60.0)) < 0.2);
}

void TestMagnetometerIgnoresBentField()
{
    // The field suddenly points elsewhere (a magnet or a speaker): the view must not follow it.
    Feeder f;
    f.tracker.SetMagnetometerGain(0.5);
    f.mag[0] = -40.0f;
    f.mag[2] = -20.0f;
    f.Run(5.0, 0, 0, 0, kG, 0, 0);
    f.mag[0] = -20.0f;
    f.mag[1] = 60.0f;
    f.mag[2] = 30.0f;
    f.Run(20.0, 0, 0, 0, kG, 0, 0);
    const Vec3 fwd = f.Forward();
    CHECK_NEAR(fwd.x, 0.0, 0.02);
    CHECK_NEAR(fwd.z, -1.0, 0.02);
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
    // As the driver has it with the body cameras on.
    config.depth_noise_with_sightings_m = 0.3;
    config.learn_light_scale = true;
    config.sighting_pairs = true;
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
    // A slow wander along the ray too, as PSMoveServiceEx's own smoothing leaves: its size and
    // how long it takes to change, s.
    double depth_wander = 0.0;
    double depth_wander_seconds = 0.5;
    double wander = 0.0;
    // The light's distance from the camera times this: PSMoveServiceEx's guess from its size off.
    double light_scale = 1.0;
    bool interpolated = false;   // PSMoveServiceEx's 200 Hz stream between frames
    double time = 100.0;
    double last_frame = 100.0;
    std::mt19937 random{1234};
    Vec3 previous_frame_light, frame_light;
    double frame_arrival = 0.0;
    bool have_frame = false;
    bool frame_seen = false;
    // Two body cameras on either side, 30 frames a second each, arriving body_age late.
    bool body = false;
    std::vector<CameraModel> body_cameras{BodyCamera({-1.6, 1.1, -2.4}), BodyCamera({1.7, 1.0, -2.2})};
    double body_age = 0.15;
    double body_pixel_noise = 3.0;
    Vec3 body_moved;  // how far the body cameras went since they were calibrated
    std::vector<Vec3> body_ears_off{Vec3{}, Vec3{}};  // where each camera's pose model puts the ears wrong
    Vec3 ears_from_neck{0.0, 0.06, 0.01};  // where the pose model really puts them
    std::function<bool(double)> body_visible;
    std::vector<double> last_body_frame{100.0, 100.0 + 1.0 / 60.0};
    std::deque<HeadSighting> body_in_flight;

    static CameraModel BodyCamera(Vec3 position)
    {
        const Vec3 target{0.1, 1.5, 0.2};
        const Vec3 forward = (target - position) * (1.0 / Length(target - position));
        Vec3 right = Cross(forward, Vec3{0, 1, 0});
        right = right * (1.0 / Length(right));
        CameraModel camera;
        camera.width = 480;
        camera.height = 640;
        camera.focal = 520.0;
        camera.rotation = Rows(right, Cross(forward, right), forward);
        camera.position = position;
        return camera;
    }

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
        if (depth_wander > 0.0) {
            const double a = std::exp(-1.0 / 60.0 / depth_wander_seconds);
            wander = a * wander + std::sqrt(1.0 - a * a) * depth_wander * Gaussian();
        }
        return camera + (light - camera) * light_scale + along * (depth_noise * Gaussian() + wander) +
               side * (lateral_noise * Gaussian()) + up * (lateral_noise * Gaussian());
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
            for (size_t c = 0; body && c < body_cameras.size(); ++c) {
                if (last_body_frame[c] < time - 0.1) {
                    last_body_frame[c] = time;
                }
                if (time < last_body_frame[c] + 1.0 / 30.0) {
                    continue;
                }
                last_body_frame[c] += 1.0 / 30.0;
                const double frame_t = last_body_frame[c] - start;
                const HeadMotion then = motion(frame_t);
                HeadSighting sighting;
                sighting.camera = static_cast<uint32_t>(c);
                sighting.time = last_body_frame[c];
                sighting.model = body_cameras[c];
                sighting.noise = body_pixel_noise;
                CameraModel moved = body_cameras[c];
                moved.position = moved.position + body_moved;
                if ((!body_visible || body_visible(frame_t)) &&
                    moved.Project(then.neck + Rotate(then.rotation, ears_from_neck) + body_ears_off[c], &sighting.u,
                                  &sighting.v)) {
                    sighting.u += body_pixel_noise * Gaussian();
                    sighting.v += body_pixel_noise * Gaussian();
                    body_in_flight.push_back(sighting);
                }
            }
            while (!body_in_flight.empty() && body_in_flight.front().time + body_age <= time) {
                estimator.AddSighting(time, body_in_flight.front());
                body_in_flight.pop_front();
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

void TestHeadLateSightingReplayed()
{
    // A view that arrives after newer light samples ends up as if it had come in its place.
    HeadSimulation sim;
    HeadPositionConfig config = TestHeadConfig();
    config.depth_noise_with_sightings_m = config.depth_noise_m;
    config.sighting_pairs = false;  // one view alone counts
    const CameraModel camera = sim.body_cameras[0];
    HeadSighting sighting;
    sighting.time = 1.0 + 20 / 60.0 - 0.041;
    sighting.model = camera;
    camera.Project(Vec3{0.1, 1.61, 0.21}, &sighting.u, &sighting.v);
    sighting.u += 4.0;
    HeadPositionEstimator in_order(config), late(config);
    for (HeadPositionEstimator* e : {&in_order, &late}) {
        e->SetCamera(true, sim.camera, 1);
        e->AddOrientation(0.0, Quat{});
    }
    for (int i = 0; i < 40; ++i) {
        const double t = 1.0 + i / 60.0;
        const Vec3 light = Vec3{0.1, 1.71, 0.18} + Vec3{0.0, 0.0, 0.01 * std::sin(i * 1.3)};
        in_order.AddLight(t, light, true, Vec3{});
        late.AddLight(t, light, true, Vec3{});
        if (i == 20) {
            in_order.AddSighting(t, sighting);
        }
        if (i == 26) {
            late.AddSighting(t, sighting);
        }
    }
    // Once the glide in of either is over.
    const Vec3 a = in_order.Output(10.0, Quat{}, Vec3{}).position, b = late.Output(10.0, Quat{}, Vec3{}).position;
    CHECK(Length(a - b) < 1e-9);
    CHECK(in_order.TakeStats().sightings == 1 && late.TakeStats().sightings == 1);
}

// Along the PS Eye's ray and across it, the eyes' error while standing still, over several runs.
struct DepthAcross {
    double depth = 0.0;
    double across = 0.0;
};

DepthAcross StillError(const std::function<void(HeadSimulation&)>& setup)
{
    double depth_sum = 0.0, across_sum = 0.0;
    int count = 0;
    for (int seed = 0; seed < 6; ++seed) {
        HeadSimulation sim;
        sim.random.seed(100 + seed);
        sim.interpolated = true;
        sim.depth_noise = 0.03;
        sim.lateral_noise = 0.002;
        setup(sim);
        const Vec3 truth = sim.Eyes(StandStill(0));
        const Vec3 along = Unit(truth - sim.camera);
        sim.Run(20.0, StandStill, nullptr, 1.0, [&](double time) {
            if (time < 102.0) {
                return;
            }
            const Vec3 error = sim.estimator.Output(time, Quat{}, Vec3{}).position - truth;
            const double depth = Dot(error, along);
            depth_sum += depth * depth;
            across_sum += Dot(error, error) - depth * depth;
            ++count;
        });
    }
    return {std::sqrt(depth_sum / count), std::sqrt(across_sum / count)};
}

void TestHeadBodySteadiesDepth()
{
    // PSMoveServiceEx's depth wanders slowly, 2 cm, on top of its frame-to-frame noise; the body
    // cameras, 0.15 s late, hold it much steadier, without the sideways getting worse.
    const auto wandering = [](bool body) {
        return StillError([body](HeadSimulation& sim) {
            sim.depth_wander = 0.02;
            sim.body = body;
        });
    };
    const DepthAcross light = wandering(false), both = wandering(true);
    CHECK(both.depth < 0.6 * light.depth);
    CHECK(both.across < 1.2 * light.across);
    // With frame-to-frame noise alone, their depth, about a centimetre, takes over from the light's.
    const DepthAcross white_both = StillError([](HeadSimulation& sim) { sim.body = true; });
    CHECK(white_both.depth < 0.015);
    CHECK(white_both.across < 0.003);
    // Standing still is smoothed more than the settings say: the agility learns it.
    HeadSimulation sim;
    sim.depth_noise = 0.03;
    sim.lateral_noise = 0.002;
    sim.Run(3.0, StandStill);
    sim.estimator.TakeStats();
    sim.Run(2.0, StandStill);
    CHECK(sim.estimator.TakeStats().agility < 0.1);
}

void TestHeadBodyCamerasMoved()
{
    // The body cameras were moved after their calibration: most of their views miss, and the light
    // keeps its depth instead of letting it wander off. The few that happen to fit still pull it.
    const DepthAcross light = StillError([](HeadSimulation& sim) { sim.depth_wander = 0.02; });
    const DepthAcross moved = StillError([](HeadSimulation& sim) {
        sim.depth_wander = 0.02;
        sim.body = true;
        sim.body_moved = Vec3{0.3, 0.0, 0.3};
    });
    CHECK(moved.depth < 2.0 * light.depth);
    CHECK(moved.across < 2.0 * light.across);
}

void TestHeadBodyFixesLightDepth()
{
    // PSMoveServiceEx puts the light half as far again from the PS Eye as it is, wandering 10 cm
    // more: where the body cameras meet on the light's line of sight, the head is, and the light's
    // distance is learned to match.
    HeadSimulation sim;
    sim.interpolated = true;
    sim.depth_noise = 0.03;
    sim.lateral_noise = 0.002;
    sim.light_scale = 1.5;
    sim.depth_wander = 0.1;
    sim.body = true;
    const HeadRun run = sim.Run(10.0, StandStill, nullptr, 2.0);
    const HeadPositionEstimator::Stats stats = sim.estimator.TakeStats();
    CHECK(run.rms < 0.03);
    CHECK(std::fabs(stats.light_scale - 1.0 / 1.5) < 0.03);
    CHECK(stats.on_line_views > 0.95 * stats.paired_views);
    CHECK(stats.sightings > 5 * stats.sightings_rejected);
    // Without them, the head is that far off.
    HeadSimulation alone = sim;
    alone.estimator = HeadPositionEstimator(TestHeadConfig());
    alone.body = false;
    CHECK(alone.Run(5.0, StandStill, nullptr, 2.0).rms > 0.5);
}

void TestHeadBodyFixesLightTooFar()
{
    // As in a test: the PS Eye below the head, PSMoveServiceEx putting the light twice as far as it
    // is, wandering 30 cm more, so that it is often over 4 m from the PS Eye (impossible, alone);
    // and the pose model's ears 5 cm higher and 3 cm further forward than first guessed. The body
    // cameras come in after the light: the head goes where they meet, as soon as their views
    // arrive, and their ears are learned in a couple of seconds.
    HeadSimulation sim;
    sim.camera = Vec3{0.0, 1.25, -1.8};
    sim.interpolated = true;
    sim.depth_noise = 0.03;
    sim.lateral_noise = 0.002;
    sim.light_scale = 2.0;
    sim.depth_wander = 0.3;
    sim.ears_from_neck = sim.config.ears_from_neck + Vec3{0.0, 0.05, -0.03};
    sim.Run(2.0, StandStill);
    sim.estimator.TakeStats();
    sim.body = true;
    sim.Run(3.0, StandStill);
    const HeadPositionEstimator::Stats first = sim.estimator.TakeStats();
    CHECK(std::fabs(first.light_scale - 0.5) < 0.05);
    // The forward 3 cm of the ears cannot be learned from the light's depth, so some of it stays.
    const HeadRun run = sim.Run(5.0, StandStill, nullptr, 0.0);
    CHECK(run.rms < 0.04);
    const HeadPositionEstimator::Stats after = sim.estimator.TakeStats();
    CHECK(after.sightings > 10 * after.sightings_rejected);
    CHECK(std::abs(after.ears_from_neck.y - sim.ears_from_neck.y) < 0.01);
}

void TestHeadBodyFixesFarLightEarsApart()
{
    // As in a VRChat test: PSMoveServiceEx puts the light twice as far as it is, wandering 10 cm,
    // and with the headset over the ears each phone's pose model puts them at another height, 7 cm
    // above and below. Each view alone disagrees with the other camera's; where the two meet is on
    // the light's line of sight, and the head goes there.
    HeadSimulation sim;
    sim.camera = Vec3{0.0, 1.25, -1.2};
    sim.interpolated = true;
    sim.depth_noise = 0.03;
    sim.lateral_noise = 0.002;
    sim.light_scale = 2.0;
    sim.depth_wander = 0.1;
    sim.body_ears_off = {Vec3{0.0, 0.07, 0.0}, Vec3{0.0, -0.07, 0.0}};
    sim.Run(2.0, StandStill);
    sim.body = true;
    const HeadRun run = sim.Run(10.0, StandStill, nullptr, 4.0);
    CHECK(run.rms < 0.03);
    CHECK(std::fabs(sim.estimator.TakeStats().light_scale - 0.5) < 0.03);
}

void TestHeadBodyFollowsWithoutLight()
{
    // Turned away from the PS Eye: the light is out of sight while turning 180 degrees and then
    // stepping 30 cm to the side. The body cameras keep the head followed, coarser and later than
    // the light, but without holding it still or jumping when the light goes or comes back.
    HeadSimulation sim;
    sim.body = true;
    sim.depth_noise = 0.03;
    sim.lateral_noise = 0.002;
    sim.Run(2.0, StandStill);
    const auto turn_and_step = [](double t) {
        HeadMotion m = StandStill(t);
        const double turn = std::clamp(t - 0.5, 0.0, 1.0);
        m.rotation = AxisAngle(Vec3{0, 1, 0}, kPi * turn * turn * (3.0 - 2.0 * turn));
        m.angular_velocity = Vec3{0, t > 0.5 && t < 1.5 ? kPi * 6.0 * turn * (1.0 - turn) : 0.0, 0};
        m.neck = m.neck + Vec3{0.3 * std::clamp(t - 1.6, 0.0, 1.0), 0, 0};
        return m;
    };
    sim.estimator.TakeStats();
    bool alone = true;
    const HeadRun run = sim.Run(4.0, turn_and_step, [](double t) { return t < 0.3 || t > 3.0; }, 0.0, [&](double time) {
        const double t = time - 102.0;
        if (t > 0.6 && t < 2.9) {
            alone = alone && sim.estimator.state() == HeadPositionEstimator::State::Live &&
                    !sim.estimator.LightFollowed(time);
        }
    });
    const HeadPositionEstimator::Stats stats = sim.estimator.TakeStats();
    CHECK(alone);
    CHECK(stats.losses == 0);
    CHECK(run.max_step < 0.01);
    // The step at 0.3 m/s, about 0.4 s behind at worst: holding still would be the whole 30 cm.
    CHECK(run.max_error < 0.15);
    CHECK(Length(run.last_eyes - run.last_truth) < 0.03);
}

void TestHeadBodyStartsWithoutLight()
{
    // No light at all: two cameras find the head, and follow it.
    HeadSimulation sim;
    sim.body = true;
    const HeadRun run = sim.Run(3.0, StandStill, [](double) { return false; }, 1.0);
    const HeadPositionEstimator::Stats stats = sim.estimator.TakeStats();
    CHECK(stats.sighting_starts == 1);
    CHECK(sim.estimator.state() == HeadPositionEstimator::State::Live);
    CHECK(run.rms < 0.02);
    // The cameras lose the person too: the head holds.
    sim.body_visible = [](double) { return false; };
    sim.Run(1.0, StandStill, [](double) { return false; });
    CHECK(sim.estimator.state() == HeadPositionEstimator::State::Holding);
}

void TestHeadLearnsEars()
{
    // The pose model puts the ears 4 cm lower and 3 cm further back than the first guess: the
    // light teaches where, so the head does not move when the light is lost.
    HeadSimulation sim;
    sim.body = true;
    sim.depth_noise = 0.03;
    sim.lateral_noise = 0.002;
    sim.ears_from_neck = sim.config.ears_from_neck + Vec3{0.0, -0.04, 0.03};
    const auto sway = [](double t) {
        HeadMotion m = StandStill(t);
        const double yaw = 0.8 * std::sin(1.3 * t), pitch = 0.3 * std::sin(0.7 * t);
        m.rotation = AxisAngle(Vec3{0, 1, 0}, yaw) * AxisAngle(Vec3{1, 0, 0}, pitch);
        m.angular_velocity = Vec3{0, 0.8 * 1.3 * std::cos(1.3 * t), 0};
        return m;
    };
    sim.Run(30.0, sway);
    // Up and down at once; forwards and backwards, along the PS Eye's line of sight whose depth may
    // be off, only as the head turns.
    const Vec3 error = sim.estimator.TakeStats().ears_from_neck - sim.ears_from_neck;
    CHECK(std::abs(error.y) < 0.005);
    CHECK(Length(error) < 0.02);
    const HeadRun run = sim.Run(1.5, [&](double t) { return sway(t + 30.0); }, [](double t) { return t < 0.5; }, 0.5);
    CHECK(run.max_error < 0.03);
}

void TestHeadBodyMoving()
{
    // Crouching and shaking the head, with the PS Eye's usual noise: as close with the body
    // cameras as with the light alone.
    double crouch[2], settle[2], shake[2];
    for (const bool body : {false, true}) {
        HeadSimulation sim;
        sim.body = body;
        sim.depth_noise = 0.03;
        sim.lateral_noise = 0.002;
        crouch[body] = sim.Run(2.3, Crouch).max_error;
        settle[body] = sim.Run(0.5, [](double t) { return Crouch(t + 2.3); }, nullptr, 0.3).max_error;
        HeadSimulation shaking;
        shaking.body = body;
        shaking.depth_noise = 0.03;
        shaking.lateral_noise = 0.002;
        shake[body] = shaking.Run(4.0, ShakeHead).rms;
    }
    CHECK(crouch[1] < 1.2 * crouch[0]);
    CHECK(settle[1] < settle[0] + 0.01);
    CHECK(shake[1] < 1.3 * shake[0]);
}

// A Move's bulb through the head's filter, as the controllers set it up.
HeadPositionConfig BulbConfig()
{
    HeadPositionConfig bulb;
    bulb.eyes_from_light = bulb.eyes_from_neck = bulb.ears_from_neck = Vec3{};
    bulb.depth_noise_m = bulb.depth_noise_with_sightings_m = 0.04;
    bulb.agility = 5.0;
    bulb.sightings_only_without_light = true;
    bulb.prediction_gain = 1.0;
    bulb.max_speed = 10.0;
    bulb.min_eye_height = -1.0;
    bulb.max_eye_height = 3.5;
    return bulb;
}

struct BulbRun {
    double depth_rms = 0.0;  // output, along the camera's line of sight
    double raw_depth_rms = 0.0;
    double rms = 0.0;
    double max_error = 0.0;
    double raw_rms = 0.0;  // PSMoveServiceEx's own position, as late as it comes
};

// 60 frames a second at 2.5 m from the PS Eye, 3 cm of depth noise; `hidden(frame)` reports the
// bulb 1.5 m farther, as a half hidden one.
BulbRun RunBulb(const std::function<Vec3(double)>& path, double seconds,
                const std::function<bool(int)>& hidden = nullptr, HeadPositionConfig config = BulbConfig())
{
    const Vec3 camera{0.0, 1.2, -2.5};
    HeadPositionEstimator bulb(config);
    bulb.SetCamera(true, camera, 1);
    std::mt19937 random(9);
    const auto gaussian = [&]() {
        const double u = (static_cast<double>(random()) + 0.5) / 4294967296.0;
        const double v = static_cast<double>(random()) / 4294967296.0;
        return std::sqrt(-2.0 * std::log(u)) * std::cos(2.0 * kPi * v);
    };
    BulbRun run;
    double sum = 0.0, depth_sum = 0.0, raw_sum = 0.0, raw_error_sum = 0.0;
    int count = 0;
    for (int frame = 0; frame < seconds * 60.0; ++frame) {
        const double t = frame / 60.0;
        const Vec3 truth = path(t);
        const Vec3 along = Unit(truth - camera);
        const Vec3 side = Unit(Cross(along, Vec3{0, 1, 0}));
        const double depth_noise = 0.03 * gaussian();
        Vec3 seen = truth + along * depth_noise + side * (0.002 * gaussian()) + Cross(side, along) * (0.002 * gaussian());
        if (hidden && hidden(frame)) {
            seen = seen + along * 1.5;
        }
        // Arrives 40 ms after the camera saw it; the output is read when it arrives.
        const double arrived = 10.0 + t + 0.040;
        bulb.AddLight(arrived, seen, true, Vec3{});
        bulb.Tick(arrived, true);
        if (t < 1.0) {
            continue;
        }
        const Vec3 now_truth = path(t + 0.040);
        const Vec3 error = bulb.Output(arrived, Quat{}, Vec3{}).position - now_truth;
        const double depth = Dot(error, along);
        sum += Dot(error, error);
        depth_sum += depth * depth;
        raw_sum += depth_noise * depth_noise;
        raw_error_sum += Dot(seen - now_truth, seen - now_truth);
        run.max_error = std::max(run.max_error, Length(error));
        ++count;
    }
    run.rms = std::sqrt(sum / count);
    run.depth_rms = std::sqrt(depth_sum / count);
    run.raw_depth_rms = std::sqrt(raw_sum / count);
    run.raw_rms = std::sqrt(raw_error_sum / count);
    return run;
}

void TestBulbFilter()
{
    // Held still at 2.5 m from the PS Eye: the depth shakes far less than PSMoveServiceEx's.
    const auto still = [](double) { return Vec3{0.2, 1.1, 0.0}; };
    const BulbRun held = RunBulb(still, 10.0);
    CHECK(held.depth_rms < 0.4 * held.raw_depth_rms);
    // Swung 30 cm to the side and back at 1.5 Hz (up to 2.8 m/s): closer than PSMoveServiceEx's
    // own position, 40 ms late. Reaching 40 cm towards the PS Eye and back at 1 Hz (up to 2.5 m/s),
    // not much farther: the depth is smoothed for a hand held still.
    const auto swing = [](double t) { return Vec3{0.2 + 0.3 * std::sin(2.0 * kPi * 1.5 * t), 1.1, 0.0}; };
    const BulbRun swung = RunBulb(swing, 6.0);
    CHECK(swung.rms < 0.6 * swung.raw_rms);
    const auto reach = [](double t) { return Vec3{0.2, 1.1, -0.4 * std::sin(2.0 * kPi * t)}; };
    const BulbRun reached = RunBulb(reach, 6.0);
    CHECK(reached.rms < 1.2 * reached.raw_rms);
    // Half hidden for 6 frames, reported 1.5 m farther: left out.
    const BulbRun half = RunBulb(still, 6.0, [](int frame) { return frame >= 150 && frame < 156; });
    CHECK(half.max_error < 0.05);
}

// A bulb as the user's PS Eye sees it from 2.5 m: 5 cm of depth noise on top of a slow 4 cm
// wander; and, with `phones`, as two phones beside the PS Eye see it, 25 frames a second each,
// arriving 180 ms late. `lost(t)`: the PS Eye does not see it.
struct PhonesRun {
    double depth_rms = 0.0;
    double max_error = 0.0;
    double tremor = 0.0;  // as the driver measures it: around its average over half a second
    bool always_live = true;
    int sightings = 0;
    int sightings_rejected = 0;
};

// A bulb 2.5 m from the PS Eye, whose depth shakes 5 cm and wanders 4 cm, and two phones that see
// it to a pixel and a half, 0.18 s late; views_off is how far off their calibration puts it.
PhonesRun RunBulbWithPhones(const std::function<Vec3(double)>& path, double seconds, bool phones,
                            const std::function<bool(double)>& lost = nullptr,
                            const std::function<bool(double)>& far = nullptr, Vec3 views_off = Vec3{})
{
    const Vec3 camera{0.0, 1.2, -2.5};
    const std::vector<CameraModel> cameras{HeadSimulation::BodyCamera({-1.5, 0.9, -2.0}),
                                           HeadSimulation::BodyCamera({0.6, 1.0, -2.1})};
    HeadPositionEstimator bulb(BulbConfig());
    bulb.SetCamera(true, camera, 1);
    std::mt19937 random(5), phone_random(6);
    const auto gaussian = [](std::mt19937& r) {
        const double u = (static_cast<double>(r()) + 0.5) / 4294967296.0;
        const double v = static_cast<double>(r()) / 4294967296.0;
        return std::sqrt(-2.0 * std::log(u)) * std::cos(2.0 * kPi * v);
    };
    std::deque<HeadSighting> in_flight;
    double wander = 0.0, next_phone[2] = {0.0, 0.02};
    PhonesRun run;
    double depth_sum = 0.0, tremor_sum = 0.0;
    int count = 0;
    Vec3 mean;
    for (int step = 0; step * 0.002 < seconds; ++step) {
        const double t = step * 0.002, now = 10.0 + t;
        if (step % 8 == 0) {  // about 60 frames a second
            const double a = std::exp(-1.0 / 60.0 / 0.5);
            wander = a * wander + std::sqrt(1.0 - a * a) * 0.04 * gaussian(random);
            const Vec3 truth = path(t - 0.040), along = Unit(truth - camera);
            const Vec3 side = Unit(Cross(along, Vec3{0, 1, 0}));
            const Vec3 seen = truth + along * (0.05 * gaussian(random) + wander + (far && far(t) ? 1.5 : 0.0)) +
                              side * (0.002 * gaussian(random)) + Cross(side, along) * (0.002 * gaussian(random));
            bulb.AddLight(now, seen, !(lost && lost(t)), Vec3{});
        }
        for (int c = 0; phones && c < 2; ++c) {
            if (t < next_phone[c]) {
                continue;
            }
            next_phone[c] += 0.04;
            HeadSighting sighting;
            sighting.camera = static_cast<uint32_t>(c);
            sighting.time = now;
            sighting.model = cameras[c];
            sighting.noise = 1.5;
            if (cameras[c].Project(path(t) + views_off, &sighting.u, &sighting.v)) {
                sighting.u += 1.5 * gaussian(phone_random);
                sighting.v += 1.5 * gaussian(phone_random);
                in_flight.push_back(sighting);
            }
        }
        while (!in_flight.empty() && in_flight.front().time + 0.18 <= now) {
            bulb.AddSighting(now, in_flight.front());
            in_flight.pop_front();
        }
        bulb.Tick(now, true);
        if (t < 2.0) {
            continue;
        }
        run.always_live = run.always_live && bulb.state() == HeadPositionEstimator::State::Live;
        const Vec3 truth = path(t);
        const Vec3 out = bulb.Output(now, Quat{}, Vec3{}).position, error = out - truth;
        const double depth = Dot(error, Unit(truth - camera));
        depth_sum += depth * depth;
        run.max_error = std::max(run.max_error, Length(error));
        mean = count > 0 ? mean + (out - mean) * (1.0 - std::exp(-0.002 / 0.5)) : out;
        tremor_sum += Dot(out - mean, out - mean);
        ++count;
    }
    run.depth_rms = std::sqrt(depth_sum / count);
    run.tremor = std::sqrt(tremor_sum / count);
    const HeadPositionEstimator::Stats stats = bulb.TakeStats();
    run.sightings = stats.sightings;
    run.sightings_rejected = stats.sightings_rejected;
    return run;
}

void TestBulbWithPhones()
{
    // Held still where the phones see it too: it is the PS Eye's, and their views shake it no more,
    // not even with a calibration 2 cm off, or 14 cm (the phones or the PS Eye moved since). Such
    // views are left out.
    const auto still = [](double) { return Vec3{0.2, 1.1, 0.0}; };
    const PhonesRun eye = RunBulbWithPhones(still, 12.0, false), both = RunBulbWithPhones(still, 12.0, true);
    const PhonesRun off = RunBulbWithPhones(still, 12.0, true, nullptr, nullptr, Vec3{0.02, 0.0, 0.0});
    const PhonesRun stale = RunBulbWithPhones(still, 12.0, true, nullptr, nullptr, Vec3{0.10, 0.0, 0.10});
    CHECK(both.tremor < 1.05 * eye.tremor);
    CHECK(off.tremor < 1.05 * eye.tremor);
    CHECK(stale.tremor < 1.05 * eye.tremor);
    CHECK(both.sightings > 500 && both.sightings_rejected < 20);
    CHECK(stale.sightings_rejected > stale.sightings / 2);
    // The PS Eye loses it for a second while it sways 30 cm: the phones keep it followed, about a
    // fifth of a second late (up to 9 cm behind, at 0.47 m/s), from the moment the PS Eye lost it.
    const auto sway = [](double t) { return Vec3{0.2 + 0.15 * std::sin(kPi * t), 1.1, 0.0}; };
    const PhonesRun hidden = RunBulbWithPhones(sway, 8.0, true, [](double t) { return t >= 4.0 && t < 5.0; });
    CHECK(hidden.always_live);
    CHECK(hidden.max_error < 0.12);
    // Half hidden from the PS Eye for half a second, PSMoveServiceEx puts it 1.5 m farther: the
    // phones hold it where it is.
    const PhonesRun steady = RunBulbWithPhones(still, 8.0, true);
    const PhonesRun half = RunBulbWithPhones(still, 8.0, true, nullptr, [](double t) { return t >= 4.0 && t < 4.5; });
    CHECK(half.max_error < steady.max_error + 0.02);
}

int main()
{
    TestParse();
    TestDiscoveryAndLatencyPackets();
    TestPredict();
    TestStillUpright();
    TestTurnLeft();
    TestRecenter();
    TestMagnetometerHoldsYaw();
    TestMagnetometerIgnoresBentField();
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
    TestHeadLateSightingReplayed();
    TestHeadBodySteadiesDepth();
    TestHeadBodyCamerasMoved();
    TestHeadBodyFixesLightDepth();
    TestHeadBodyFixesLightTooFar();
    TestHeadBodyFixesFarLightEarsApart();
    TestHeadBodyFollowsWithoutLight();
    TestHeadBodyStartsWithoutLight();
    TestHeadLearnsEars();
    TestHeadBodyMoving();
    TestBulbFilter();
    TestBulbWithPhones();
    if (g_failures == 0) {
        std::printf("All tests passed\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
}
