// Small self-contained tests for the parser and the head tracker. Run with ctest.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "head_tracker.h"
#include "protocol.h"

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

}  // namespace

int main()
{
    TestParse();
    TestStillUpright();
    TestTurnLeft();
    TestRecenter();
    TestTiltFollowsGravity();
    TestGyroBiasIsLearned();
    TestRejectsBadSamples();
    if (g_failures == 0) {
        std::printf("All tests passed\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
}
