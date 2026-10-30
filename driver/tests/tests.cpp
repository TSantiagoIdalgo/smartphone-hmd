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
#include "head_position.h"
#include "head_tracker.h"
#include "move_hand.h"
#include "move_yaw.h"
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
    // A constant 0.01 rad/s offset would drift 0.6 rad in a minute. Lying flat on the table for 5 s
    // (gravity across the screen) it is learned; then, upright in the visor, the view holds still.
    Feeder f;
    f.Run(5.0, 0.01f, 0, 0, 0, 0, kG);
    f.Run(5.0, 0.01f, 0, 0, kG, 0, 0);  // the tilt settles
    const Vec3 before = f.Forward();
    f.Run(60.0, 0.01f, 0, 0, kG, 0, 0);
    const Vec3 after = f.Forward();
    CHECK(std::fabs(std::atan2(-after.x, -after.z) - std::atan2(-before.x, -before.z)) < 0.03);
    CHECK(f.tracker.TakeStats().table_seconds >= 4);
}

void TestSlowTurnIsNotBias()
{
    // Worn, the head turns slowly, 2 degrees a second for 20 s: all of it shows, none of it is
    // taken for the gyroscope's bias.
    Feeder f;
    f.Run(20.0, 0.035f, 0, 0, kG, 0, 0);
    const Vec3 fwd = f.Forward();
    CHECK_NEAR(std::atan2(-fwd.x, -fwd.z), 0.7, 0.01);
    // And still afterwards, it stays where it turned to.
    f.Run(10.0, 0.0f, 0, 0, kG, 0, 0);
    const Vec3 still = f.Forward();
    CHECK_NEAR(std::atan2(-still.x, -still.z), 0.7, 0.01);
}

void TestGapWhileTurningIsJoined()
{
    // Turning at 1 rad/s, 0.3 s of samples lost on the way: the turn is still all there.
    Feeder f;
    f.Run(0.5, 1.0f, 0, 0, kG, 0, 0);
    f.ts += 150 * kStepNs;
    f.Run(0.5, 1.0f, 0, 0, kG, 0, 0);
    const Vec3 fwd = f.Forward();
    CHECK_NEAR(std::atan2(-fwd.x, -fwd.z), 1.3, 0.02);
    CHECK(f.tracker.TakeStats().gaps == 1);
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

// A Move's bulb through the head's filter, as the controllers set it up.
HeadPositionConfig BulbConfig()
{
    HeadPositionConfig bulb;
    bulb.eyes_from_light = bulb.eyes_from_neck = Vec3{};
    bulb.depth_noise_m = 0.04;
    bulb.agility = 5.0;
    bulb.distrust_jumps = true;
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

void TestMoveHandAxes()
{
    // PSMoveServiceEx's identity is the Move pointing along -z, buttons up: SteamVR's too.
    MoveHand hand(true);
    MoveState move;
    move.tracking = true;
    move.orientation = AxisAngle(Vec3{0, 1, 0}, 0.7);
    hand.Update(move, 1.0);
    CHECK(Length(Rotate(hand.Pose().orientation, Vec3{0, 0, -1}) - Rotate(move.orientation, Vec3{0, 0, -1})) < 1e-9);
    // Tilted by the setting: a Move whose identity were upright points forward lying flat.
    HandConfig config;
    config.pitch_degrees = 90.0;
    MoveHand tilted(true, config);
    move.orientation = AxisAngle(Vec3{1, 0, 0}, -kPi / 2);
    tilted.Update(move, 1.0);
    const HandPose pose = tilted.Pose();
    CHECK(Length(Rotate(pose.orientation, Vec3{0, 0, -1}) - Vec3{0, 0, -1}) < 1e-9);
    CHECK(Length(Rotate(pose.orientation, Vec3{0, 1, 0}) - Vec3{0, 1, 0}) < 1e-9);
    // And back: the rotation PSMoveServiceEx would give for a controller's.
    const Quat back = tilted.MoveOrientation(pose.orientation);
    CHECK(Length(Rotate(back, Vec3{0, 1, 0}) - Rotate(move.orientation, Vec3{0, 1, 0})) < 1e-9);
}

void TestMoveHandIgnoresBulbJumps()
{
    // Half hidden for a tenth of a second, the bulb is reported 1.5 m farther, still "tracked":
    // the hand stays put. Truly moved and staying there, it follows after a moment.
    MoveHand hand(false);
    MoveState move;
    move.tracking = true;
    move.position = Vec3{0.2, 1.0, 0.5};
    double time = 10.0, max_jump = 0.0;
    for (int i = 0; i < 120; ++i, time += 1.0 / 60.0) {
        move.time = time;
        const bool hidden = i >= 60 && i < 66;
        MoveState reported = move;
        if (hidden) {
            reported.position = move.position + Vec3{0.0, 0.0, 1.5};
        }
        hand.Update(reported, time);
        if (i >= 30) {
            max_jump = std::max(max_jump, Length(hand.Pose().position - move.position));
        }
    }
    CHECK(max_jump < 0.01);
    CHECK(hand.TakeStats().jumps == 6);
    move.position = Vec3{0.2, 1.0, 1.2};  // moved 70 cm at once, and stays
    for (int i = 0; i < 60; ++i, time += 1.0 / 60.0) {
        move.time = time;
        hand.Update(move, time);
    }
    CHECK(Length(hand.Pose().position - move.position) < 0.02);
}

// The headset 1.6 m up looking along -z, and where a hand rests in front of it.
Headset LookingAhead()
{
    Headset headset;
    headset.valid = true;
    headset.eyes = Vec3{0.0, 1.6, 0.0};
    headset.forward = Vec3{0.0, 0.0, -1.0};
    return headset;
}

// Runs a right hand at 250 updates a second from `time`; `seen(i)` and `where(i)` per update.
struct HandRun {
    MoveHand hand{false};
    double time = 100.0;
    Headset headset = LookingAhead();
    double largest_step = 0.0;
    Vec3 previous;
    bool have_previous = false;

    HandPose Run(int updates, const std::function<bool(int)>& seen, const std::function<Vec3(int)>& where)
    {
        HandPose pose;
        for (int i = 0; i < updates; ++i, time += 0.004) {
            MoveState move;
            move.time = time;
            move.tracking = seen(i);
            move.position = where(i);
            hand.Update(move, time, headset);
            pose = hand.Pose();
            if (have_previous) {
                largest_step = std::max(largest_step, Length(pose.position - previous));
            }
            previous = pose.position;
            have_previous = true;
        }
        return pose;
    }
};

void TestMoveHandRestsInFrontOfTheHead()
{
    // The PS Eye loses the bulb. Hidden a moment it stays where it was; lost longer it glides to
    // where it rests in front of the headset, and follows it there as it turns; seen again, it goes
    // back to the bulb.
    HandRun run;
    const Vec3 bulb{0.3, 1.1, -0.4};
    run.Run(500, [](int) { return true; }, [&](int) { return bulb; });
    HandPose pose = run.Run(50, [](int) { return false; }, [&](int) { return bulb; });  // 0.2 s
    CHECK(!pose.at_rest && Length(pose.position - bulb) < 0.02);  // within the still radius
    run.largest_step = 0.0;
    pose = run.Run(250, [](int) { return false; }, [&](int) { return bulb; });
    // 50 cm under the eyes, 30 cm in front and 20 cm to the right.
    CHECK(pose.at_rest && Length(pose.position - Vec3{0.2, 1.1, -0.3}) < 0.02);
    CHECK(run.largest_step < 0.01);  // glides, at most 10 mm in 4 ms
    // The head turns to look left: the hand goes with it, in front and to its right.
    run.headset.forward = Vec3{-1.0, 0.0, 0.0};
    pose = run.Run(250, [](int) { return false; }, [&](int) { return bulb; });
    CHECK(pose.at_rest && Length(pose.position - Vec3{-0.3, 1.1, -0.2}) < 0.02);
    run.hand.TakeStats();
    run.headset.forward = Vec3{0.0, 0.0, -1.0};
    pose = run.Run(250, [](int) { return true; }, [&](int) { return bulb; });
    CHECK(pose.tracking && Length(pose.position - bulb) < 0.02);
    // Where it was, from the eyes looking ahead: 30 cm to the right, 40 cm in front, 50 cm below.
    const MoveHand::Stats stats = run.hand.TakeStats();
    CHECK(Length(stats.offset - Vec3{0.3, -0.5, 0.4}) < 1e-9);
    CHECK_NEAR(stats.farthest, 0.5, 1e-9);
}

void TestMoveHandIgnoresGlimpses()
{
    // Turned away from the PS Eye, the hand rests in front of the headset. Now and then the PS Eye
    // glimpses a sliver of the bulb, put wherever: the hand stays. Seen for good, it goes back.
    HandRun run;
    const Vec3 rest{0.2, 1.1, -0.3};
    run.Run(250, [](int) { return false; }, [](int) { return Vec3{}; });
    double farthest = 0.0;
    for (int i = 0; i < 10; ++i) {
        // 60 ms seen, near the hand or far off, then 0.4 s not.
        const HandPose pose = run.Run(115, [](int u) { return u < 15; },
                                      [&](int) { return rest + Vec3{0.0, 0.0, i % 2 == 0 ? 0.25 : 0.9}; });
        farthest = std::max(farthest, Length(pose.position - rest));
        CHECK(pose.at_rest);
    }
    CHECK(farthest < 0.001);
    CHECK(run.hand.TakeStats().glimpses == 10);
    const Vec3 bulb = rest + Vec3{0.1, 0.1, 0.1};
    HandPose pose = run.Run(70, [](int) { return true; }, [&](int) { return bulb; });  // 0.28 s
    CHECK(pose.at_rest);
    run.largest_step = 0.0;
    pose = run.Run(300, [](int) { return true; }, [&](int) { return bulb; });
    CHECK(pose.tracking && Length(pose.position - bulb) < 0.02);
    CHECK(run.largest_step < 0.01);
}

void TestMoveHandOutOfReach()
{
    // The PS Eye follows something 2 m from the headset: no arm reaches there.
    HandRun run;
    HandPose pose = run.Run(500, [](int) { return true; }, [](int) { return Vec3{0.0, 1.2, -2.0}; });
    CHECK(pose.at_rest && Length(pose.position - Vec3{0.2, 1.1, -0.3}) < 0.02);
    CHECK(run.hand.TakeStats().out_of_reach == 500);
    // 1.3 m below the eyes, though, it is the hand: the headset may be put too high.
    pose = run.Run(500, [](int) { return true; }, [](int) { return Vec3{0.2, 0.3, -0.3}; });
    CHECK(pose.tracking);
    CHECK(run.hand.TakeStats().out_of_reach == 0);
}

void TestMoveHandHoldsWithoutRest()
{
    // Rest turned off: lost, it stays where it was, still turning, without jumping.
    HandConfig config;
    config.rest_enabled = false;
    HandRun run;
    run.hand = MoveHand(false, config);
    const Vec3 bulb{0.3, 1.1, -0.4};
    run.Run(100, [](int) { return true; }, [&](int) { return bulb; });
    run.hand.TakeStats();
    const Vec3 last = run.hand.Pose().position;
    const HandPose pose = run.Run(500, [](int) { return false; }, [&](int) { return bulb; });
    CHECK(!pose.at_rest && !pose.tracking && Length(pose.position - last) < 1e-9);
    CHECK(run.hand.TakeStats().held == 500);
}

// A hand held still 2.5 m from the PS Eye: PSMoveServiceEx's bulb shakes `depth_noise` in depth,
// its rotation 0.15 degrees, and the gyro reads the hand's own tremor, about 10 Hz. Through the
// bulb filter and the hand, as the controllers have them; with `turn_after`, the hand then turns at
// 1 rad/s.
struct StillHand {
    double drift_m = 0.0;      // how far the controller went a second, held still
    double drift_degrees = 0.0;
    double off_m = 0.0;        // the most the controller was from the bulb
    double off_degrees = 0.0;
    double still_speed = 0.0;  // the velocities SteamVR got while held still
    double lag_degrees = 0.0;  // turning, the most it was behind the Move, after 0.1 s
};

StillHand RunStillHand(double seconds, double depth_noise, bool turn_after)
{
    const Vec3 camera{0.0, 1.2, -2.5}, truth{0.2, 1.1, 0.0};
    HeadPositionEstimator bulb(BulbConfig());
    bulb.SetCamera(true, camera, 1);
    MoveHand hand(false);
    std::mt19937 random(7);
    const auto gaussian = [&]() {
        const double u = (static_cast<double>(random()) + 0.5) / 4294967296.0;
        const double v = static_cast<double>(random()) / 4294967296.0;
        return std::sqrt(-2.0 * std::log(u)) * std::cos(2.0 * kPi * v);
    };
    const auto angle = [](Quat a, Quat b) {
        const double cosine = std::fabs(a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z);
        return 2.0 * std::acos(std::min(cosine, 1.0)) * 180.0 / kPi;
    };
    StillHand result;
    Headset headset = LookingAhead();
    headset.eyes = truth + Vec3{0.0, 0.5, 0.3};
    Vec3 seen = truth, previous_position;
    Quat noisy{}, previous_rotation{};
    const Vec3 along = Unit(truth - camera), side = Unit(Cross(along, Vec3{0, 1, 0}));
    for (int step = 0; step * 0.002 < seconds + (turn_after ? 1.0 : 0.0); ++step) {
        const double t = step * 0.002, now = 10.0 + t;
        const bool turning = turn_after && t >= seconds;
        const Quat rotation = AxisAngle(Vec3{0, 1, 0}, turning ? t - seconds : 0.0);
        if (step % 8 == 0) {  // about 60 frames a second
            seen = truth + along * (depth_noise * gaussian()) + side * (0.002 * gaussian()) +
                   Cross(side, along) * (0.002 * gaussian());
            const Vec3 axis = Unit(Vec3{gaussian(), gaussian(), gaussian()});
            noisy = rotation * AxisAngle(axis, 0.15 * kPi / 180.0 * gaussian());
        }
        bulb.AddLight(now, seen, true, Vec3{});
        bulb.Tick(now, true);
        MoveState move;
        move.time = now;
        move.tracking = bulb.state() == HeadPositionEstimator::State::Live;
        const HeadPoseOut out = bulb.Output(now, Quat{}, Vec3{});
        move.position = out.position;
        move.velocity = out.velocity;
        move.orientation = noisy;
        move.angular_velocity = Vec3{0.1 * std::sin(2.0 * kPi * 10.0 * t), 0.02 * gaussian() + (turning ? 1.0 : 0.0),
                                     0.02 * gaussian()};
        hand.Update(move, now, headset);
        const HandPose& pose = hand.Pose();
        if (t >= 2.0 && !turning) {
            result.drift_m += Length(pose.position - previous_position) / (seconds - 2.0);
            result.drift_degrees += angle(pose.orientation, previous_rotation) / (seconds - 2.0);
            result.off_m = std::max(result.off_m, Length(pose.position - truth));
            result.off_degrees = std::max(result.off_degrees, angle(pose.orientation, rotation));
            result.still_speed = std::max(result.still_speed, Length(pose.velocity) + Length(pose.angular_velocity));
        } else if (turning && t - seconds > 0.1) {
            result.lag_degrees = std::max(result.lag_degrees, angle(pose.orientation, noisy));
        }
        previous_position = pose.position;
        previous_rotation = pose.orientation;
    }
    return result;
}

void TestMoveHandKeepsStill()
{
    // Held still for 20 s, the bulb's depth shaking 1.5 or 3 cm: the controller hardly moves, and
    // SteamVR gets no velocities to carry it on with.
    for (const double depth_noise : {0.015, 0.03}) {
        const StillHand still = RunStillHand(22.0, depth_noise, false);
        std::printf("held still, depth noise %.0f mm: drifts %.2f mm and %.3f degrees a second, off by up to %.1f mm "
                    "and %.2f degrees\n",
                    depth_noise * 1000.0, still.drift_m * 1000.0, still.drift_degrees, still.off_m * 1000.0,
                    still.off_degrees);
        CHECK(still.drift_m < 0.002);
        CHECK(still.drift_degrees < 0.05);
        CHECK(still.off_m < 0.03);
        CHECK(still.off_degrees < 0.8);
        CHECK(still.still_speed < 1e-9);
    }
    // Then turned at 1 rad/s: it follows the Move as it is, soon.
    const StillHand turned = RunStillHand(4.0, 0.03, true);
    std::printf("turning: up to %.2f degrees behind\n", turned.lag_degrees);
    CHECK(turned.lag_degrees < 0.01);
}

// A Move swung about briskly 2.5 m from the PS Eye, its rotation as PSMoveServiceEx gives it
// turned `drift(t)` about the vertical from the truth. The PS Eye's positions arrive 40 ms late,
// 3 cm off in depth and 2 mm sideways each frame; the accelerometer sits 12 cm behind the bulb.
// Returns the correction at each second.
std::vector<double> RunMoveYaw(double seconds, const std::function<double(double)>& drift, bool brisk = true)
{
    const Vec3 camera{0.0, 1.2, -2.5}, base{0.2, 1.1, 0.0}, accel_to_bulb{0.0, 0.0, -0.12};
    std::mt19937 random(11);
    const auto gaussian = [&]() {
        const double u = (static_cast<double>(random()) + 0.5) / 4294967296.0;
        const double v = static_cast<double>(random()) / 4294967296.0;
        return std::sqrt(-2.0 * std::log(u)) * std::cos(2.0 * kPi * v);
    };
    const double k = brisk ? 1.0 : 0.05;
    const auto bulb = [&](double t) {
        return base + Vec3{k * (0.25 * std::sin(2.0 * kPi * 1.3 * t) + 0.1 * std::sin(2.0 * kPi * 2.7 * t + 1.0)),
                           k * 0.15 * std::sin(2.0 * kPi * 0.9 * t),
                           k * (0.2 * std::sin(2.0 * kPi * 1.1 * t + 2.0) + 0.08 * std::sin(2.0 * kPi * 3.1 * t))};
    };
    const auto rotation = [&](double t) {
        const double yaw = 0.5 * std::sin(2.0 * kPi * 0.7 * t) + k * 0.6 * std::sin(2.0 * kPi * 2.3 * t);
        const double pitch = 0.3 * std::sin(2.0 * kPi * 1.7 * t);
        return AxisAngle(Vec3{0, 1, 0}, yaw) * AxisAngle(Vec3{1, 0, 0}, pitch);
    };
    const auto accel_point = [&](double t) { return bulb(t) - Rotate(rotation(t), accel_to_bulb); };
    MoveYaw yaw(0.040);
    std::vector<double> corrections;
    std::deque<std::pair<double, Vec3>> in_flight;
    Vec3 seen = bulb(0.0);
    const double h = 1e-3;
    for (int step = 0; step * 0.002 < seconds; ++step) {
        const double t = step * 0.002, now = 10.0 + t;
        if (step % 8 == 0) {  // a camera frame, arriving 40 ms later
            const Vec3 truth = bulb(t), along = Unit(truth - camera), side = Unit(Cross(along, Vec3{0, 1, 0}));
            in_flight.push_back({now + 0.040, truth + along * (0.03 * gaussian()) + side * (0.002 * gaussian()) +
                                                  Cross(side, along) * (0.002 * gaussian())});
        }
        while (!in_flight.empty() && in_flight.front().first <= now) {
            seen = in_flight.front().second;
            in_flight.pop_front();
        }
        MoveState move;
        move.time = now;
        move.tracking = true;
        move.position = seen;
        const Quat q = rotation(t);
        move.orientation = Normalize(AxisAngle(Vec3{0, 1, 0}, drift(t)) * q);
        if (step % 2 == 0) {  // the sensors, 250 times a second
            const Vec3 a = (accel_point(t + h) - accel_point(t) * 2.0 + accel_point(t - h)) * (1.0 / (h * h));
            const Vec3 force = (a + Vec3{0.0, 9.81, 0.0}) * (1.0 / 9.81);
            move.accel = Rotate(Conjugate(q), force) + Vec3{0.01 * gaussian(), 0.01 * gaussian(), 0.01 * gaussian()};
            const Quat dq = Normalize(rotation(t + h) * Conjugate(rotation(t - h)));
            const Vec3 world_spin = Vec3{dq.x, dq.y, dq.z} * (2.0 / (2.0 * h) * (dq.w < 0.0 ? -1.0 : 1.0));
            move.gyro = Rotate(Conjugate(q), world_spin) + Vec3{0.01 * gaussian(), 0.01 * gaussian(), 0.01 * gaussian()};
            move.sensor_time = t;
        } else {
            move.accel = Vec3{};  // between samples PSMoveServiceEx repeats the last; nothing new here
        }
        if (Length(move.accel) > 0.0) {
            yaw.Add(move, now);
        } else {
            MoveState same = move;
            same.tracking = true;
            yaw.Add(same, now);
        }
        if (step % 500 == 0) {
            corrections.push_back(yaw.Correction());
        }
    }
    return corrections;
}

void TestMoveYawFindsTheDrift()
{
    // No drift: it stays near none. Then PSMoveServiceEx's rotation slips 25 degrees in a flick:
    // within some seconds of swinging, the correction undoes it.
    const auto slip = [](double t) { return t >= 15.0 ? 25.0 * kPi / 180.0 : 0.0; };
    const std::vector<double> c = RunMoveYaw(40.0, slip);
    double before = 0.0;
    for (int s = 5; s < 15; ++s) {
        before = std::max(before, std::fabs(c[s]));
    }
    std::printf("move yaw: up to %.1f degrees before the slip, %.1f at 20 s, %.1f at 25 s, %.1f at 40 s\n",
                before * 180.0 / kPi, c[20] * 180.0 / kPi, c[25] * 180.0 / kPi, c.back() * 180.0 / kPi);
    CHECK(before < 3.0 * kPi / 180.0);
    CHECK(std::fabs(c.back() + 25.0 * kPi / 180.0) < 3.0 * kPi / 180.0);
    // Barely moving, nothing tells it a turn: it corrects nothing.
    const std::vector<double> still = RunMoveYaw(30.0, slip, false);
    CHECK(std::fabs(still.back()) < 1e-9);
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
    TestSlowTurnIsNotBias();
    TestGapWhileTurningIsJoined();
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
    TestBulbFilter();
    TestMoveHandAxes();
    TestMoveHandIgnoresBulbJumps();
    TestMoveHandRestsInFrontOfTheHead();
    TestMoveHandIgnoresGlimpses();
    TestMoveHandOutOfReach();
    TestMoveHandHoldsWithoutRest();
    TestMoveHandKeepsStill();
    TestMoveYawFindsTheDrift();
    if (g_failures == 0) {
        std::printf("All tests passed\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
}
