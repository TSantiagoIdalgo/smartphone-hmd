// Small self-contained tests for the parser and the head tracker. Run with ctest.
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "chaperone.h"
#include "color_convert.h"
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
    if (g_failures == 0) {
        std::printf("All tests passed\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
}
