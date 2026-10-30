// Tests for the body tracking side of the driver: camera calibration and triangulation, the
// packets from the pose program, and the body filter. Run with ctest.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <functional>
#include <random>
#include <vector>

#include "body_filter.h"
#include "body_protocol.h"
#include "body_session.h"
#include "camera_model.h"
#include "move_hand.h"

namespace {

int g_failures = 0;

#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            std::printf("%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
            ++g_failures;                                                        \
        }                                                                        \
    } while (0)

#define CHECK_NEAR(a, b, tol) CHECK(std::fabs((a) - (b)) <= (tol))

constexpr double kPi = 3.14159265358979;

// Random numbers that come out the same with every compiler: the standard library's
// distributions differ between them, the generator itself does not.
double Uniform(std::mt19937& random, double low = 0.0, double high = 1.0)
{
    return low + (high - low) * ((random() + 0.5) / 4294967296.0);
}

double Gaussian(std::mt19937& random, double sigma)
{
    const double a = Uniform(random), b = Uniform(random);
    return sigma * std::sqrt(-2.0 * std::log(a)) * std::cos(2.0 * kPi * b);
}

// A camera at `position` looking at `target`, upright.
CameraModel LookAt(Vec3 position, Vec3 target, double focal, double k1, int width = 480, int height = 640)
{
    const Vec3 forward = (target - position) * (1.0 / Length(target - position));
    Vec3 right = Cross(forward, Vec3{0, 1, 0});
    right = right * (1.0 / Length(right));
    const Vec3 down = Cross(forward, right);
    CameraModel camera;
    camera.width = width;
    camera.height = height;
    camera.focal = focal;
    camera.k1 = k1;
    camera.rotation = Rows(right, down, forward);
    camera.position = position;
    return camera;
}

double AngleBetween(const Mat3& a, const Mat3& b)
{
    const Mat3 d = a * Transpose(b);
    const double trace = d.m[0][0] + d.m[1][1] + d.m[2][2];
    return std::acos(std::fmax(-1.0, std::fmin(1.0, (trace - 1.0) / 2.0)));
}

// Where someone might wave a Move: in front of the cameras, from the knees to above the head.
Vec3 RoomPoint(std::mt19937& random)
{
    const double x = Uniform(random, -1.0, 1.0), y = Uniform(random, 0.3, 2.1), z = Uniform(random, -1.0, 1.0);
    return {x, y, z};
}

std::vector<uint8_t> FromHex(const std::string& hex)
{
    std::vector<uint8_t> bytes;
    for (size_t i = 0; i + 1 < hex.size(); i += 2) {
        bytes.push_back(static_cast<uint8_t>(std::stoi(hex.substr(i, 2), nullptr, 16)));
    }
    return bytes;
}

void TestBodyPacket()
{
    // As tools/body_tracker.py sends it (see its test_driver_packet_layout).
    const std::vector<uint8_t> data = FromHex(
        "484d44310c00000003000000d2040000000000000d0c0b0ae00180020000f342050100000000003f0000803e0ad7a33c"
        "0000803e0000003fcdccccbd6666663f3333733f");
    body_protocol::Frame frame;
    CHECK(body_protocol::Parse(data.data(), data.size(), &frame));
    CHECK(frame.seq == 3);
    CHECK(frame.camera == 0x0A0B0C0D);
    CHECK(frame.width == 480 && frame.height == 640);
    CHECK_NEAR(frame.age_seconds, 0.1215, 1e-6);
    CHECK(frame.flags == (body_protocol::kFlagPose | body_protocol::kFlagBall));
    CHECK(frame.points == 1);
    CHECK_NEAR(frame.ball[0], 0.5, 1e-6);
    CHECK_NEAR(frame.ball[1], 0.25, 1e-6);
    CHECK_NEAR(frame.ball[2], 0.02, 1e-6);
    CHECK_NEAR(frame.point[0][0], 0.25, 1e-6);
    CHECK_NEAR(frame.point[0][1], 0.5, 1e-6);
    CHECK_NEAR(frame.point[0][3], 0.9, 1e-6);
    CHECK(!body_protocol::Parse(data.data(), data.size() - 1, &frame));  // a point cut short
    std::vector<uint8_t> other = data;
    other[4] = 7;  // the phone's pose packet
    CHECK(!body_protocol::Parse(other.data(), other.size(), &frame));
    CHECK(frame.bulbs == 0);

    // With every bulb by a hand (see its test_driver_packet_with_bulbs).
    const std::vector<uint8_t> bulbs = FromHex(
        "484d44310c00000003000000d2040000000000000d0c0b0ae00180020000f342250100000000003f0000803e0ad7a33c"
        "0000803e0000003fcdccccbd6666663f3333733f020000000000003f0000803e0ad7a33c0000403f0000003f0000003d");
    CHECK(body_protocol::Parse(bulbs.data(), bulbs.size(), &frame));
    CHECK(frame.bulbs == 2);
    CHECK_NEAR(frame.bulb[1][0], 0.75, 1e-6);
    CHECK_NEAR(frame.bulb[1][1], 0.5, 1e-6);
    CHECK_NEAR(frame.bulb[1][2], 0.03125, 1e-6);
    CHECK_NEAR(frame.point[0][0], 0.25, 1e-6);
    CHECK(!body_protocol::Parse(bulbs.data(), bulbs.size() - 1, &frame));
}

void TestCameraProjectsAndCastsRays()
{
    const CameraModel camera = LookAt({2.0, 1.1, -2.0}, {0.0, 1.0, 0.0}, 500.0, -0.1);
    double u = 0.0, v = 0.0;
    CHECK(camera.Project({0.0, 1.0, 0.0}, &u, &v));
    CHECK_NEAR(u, 240.0, 1e-6);  // what it looks at is in the middle
    CHECK_NEAR(v, 320.0, 1e-6);
    CHECK(camera.Project({0.0, 1.5, 0.0}, &u, &v));
    CHECK(v < 320.0);  // higher up is higher in the picture
    const Vec3 point{0.4, 1.6, 0.3};
    CHECK(camera.Project(point, &u, &v));
    const Vec3 ray = camera.Ray(u, v);
    const Vec3 towards = (point - camera.position) * (1.0 / Length(point - camera.position));
    CHECK(Length(ray - towards) < 1e-6);
    CHECK(!camera.Project({4.0, 1.0, -4.0}, &u, &v));  // behind

    CameraModel copy;
    CHECK(CameraModel::Parse(camera.Serialize(), &copy));
    CHECK(copy.width == 480 && copy.height == 640);
    CHECK_NEAR(copy.focal, 500.0, 1e-6);
    CHECK_NEAR(copy.k1, -0.1, 1e-9);
    CHECK(Length(copy.position - camera.position) < 1e-6);
    CHECK(AngleBetween(copy.rotation, camera.rotation) < 1e-6);
    CHECK(!CameraModel::Parse("v1 480 640 nope", &copy));
}

std::vector<CalibrationSample> Samples(const CameraModel& camera, int count, double noise_px, double wrong_share,
                                       unsigned seed)
{
    std::mt19937 random(seed);
    std::vector<CalibrationSample> samples;
    while (static_cast<int>(samples.size()) < count) {
        CalibrationSample s;
        s.world = RoomPoint(random);
        if (!camera.Project(s.world, &s.u, &s.v) || s.u < 0 || s.v < 0 || s.u > camera.width || s.v > camera.height) {
            continue;
        }
        if (Uniform(random) < wrong_share) {
            s.u = Uniform(random) * camera.width;  // a misread bulb
            s.v = Uniform(random) * camera.height;
        } else {
            s.u += Gaussian(random, noise_px);
            s.v += Gaussian(random, noise_px);
        }
        samples.push_back(s);
    }
    return samples;
}

void TestCalibrationFindsTheCamera()
{
    const CameraModel truth = LookAt({1.8, 1.15, -2.3}, {0.0, 1.0, 0.0}, 520.0, -0.08);
    const CalibrationResult result = CalibrateCamera(Samples(truth, 600, 1.0, 0.08, 7), 480, 640);
    CHECK(result.ok);
    CHECK(Length(result.camera.position - truth.position) < 0.02);
    CHECK(AngleBetween(result.camera.rotation, truth.rotation) < 0.5 * kPi / 180.0);
    CHECK_NEAR(result.camera.focal, 520.0, 8.0);
    CHECK_NEAR(result.camera.k1, -0.08, 0.03);
    CHECK_NEAR(result.rms_px, 1.4, 0.4);  // the noise: 1 px in u and v
    CHECK(result.outliers >= 30 && result.outliers <= 70);
    CHECK(result.rms_m < 0.01);
}

void TestCalibrationNeedsAgreeingSamples()
{
    const CameraModel truth = LookAt({1.8, 1.15, -2.3}, {0.0, 1.0, 0.0}, 520.0, 0.0);
    CHECK(!CalibrateCamera(Samples(truth, 20, 1.0, 0.0, 3), 480, 640).ok);
    const CalibrationResult noise = CalibrateCamera(Samples(truth, 300, 1.0, 1.0, 3), 480, 640);
    CHECK(!noise.ok);
    CHECK(!noise.problem.empty());
}

void TestTriangulatesFromTwoCameras()
{
    const CameraModel left = LookAt({-1.6, 1.1, -2.4}, {0.0, 1.0, 0.0}, 520.0, -0.05);
    const CameraModel right = LookAt({1.7, 1.0, -2.2}, {0.0, 1.0, 0.0}, 500.0, -0.08);
    std::mt19937 random(11);
    const auto noise = [&random]() { return Gaussian(random, 1.0); };
    double sum = 0.0;
    int count = 0;
    for (int i = 0; i < 200; ++i) {
        const Vec3 point = RoomPoint(random);
        double u1, v1, u2, v2;
        if (!left.Project(point, &u1, &v1) || !right.Project(point, &u2, &v2)) {
            continue;
        }
        Vec3 found;
        CHECK(Triangulate({{&left, u1 + noise(), v1 + noise()}, {&right, u2 + noise(), v2 + noise()}},
                          &found));
        sum += Length(found - point);
        ++count;
    }
    CHECK(count > 150);
    CHECK(sum / count < 0.01);

    Vec3 found;
    double u, v;
    left.Project({0, 1, 0}, &u, &v);
    CHECK(!Triangulate({{&left, u, v}}, &found));            // one camera has no depth
    CHECK(!Triangulate({{&left, u, v}, {&left, u, v}}, &found));  // nor two views along one ray
}


// A person standing at `root` (the floor under their hips), facing `facing` radians from -z
// (towards the cameras) to the left: where each of the filter's joints is.
struct Person {
    Vec3 root{0.0, 0.0, 0.0};
    double facing = 0.0;
    double left_foot_lift = 0.0;  // m, the left foot raised
    Vec3 left_foot_shift;         // m, world, the left foot moved

    Vec3 Joint(int joint) const
    {
        // Offsets: to the person's right, up, forward.
        static const double offsets[kJointCount][3] = {
            {-0.18, 1.45, 0.0}, {0.18, 1.45, 0.0}, {-0.1, 0.95, 0.0}, {0.1, 0.95, 0.0},   {-0.1, 0.5, 0.03},
            {0.1, 0.5, 0.03},   {-0.1, 0.08, 0.0}, {0.1, 0.08, 0.0}, {-0.1, 0.03, -0.05}, {0.1, 0.03, -0.05},
            {-0.1, 0.03, 0.15}, {0.1, 0.03, 0.15},  {-0.25, 0.85, 0.05}, {0.25, 0.85, 0.05},
        };
        const Vec3 forward{-std::sin(facing), 0.0, -std::cos(facing)};
        const Vec3 right = Cross(forward, Vec3{0, 1, 0});
        Vec3 p = root + right * offsets[joint][0] + Vec3{0, offsets[joint][1], 0} + forward * offsets[joint][2];
        const bool left_foot = joint == kLeftAnkle || joint == kLeftHeel || joint == kLeftToe;
        if (left_foot || joint == kLeftKnee) {
            p = p + left_foot_shift * (left_foot ? 1.0 : 0.5) + Vec3{0, left_foot_lift * (left_foot ? 1.0 : 0.5), 0};
        }
        return p;
    }
};

struct BodySimulation {
    std::vector<CameraModel> cameras{LookAt({-1.6, 1.1, -2.4}, {0.0, 0.9, 0.0}, 520.0, -0.05),
                                     LookAt({1.7, 1.0, -2.2}, {0.0, 0.9, 0.0}, 500.0, -0.08)};
    BodyFilter filter;
    std::mt19937 random{5};
    double time = 100.0;
    double noise_px = 1.0;  // white, per frame: worse than MediaPipe's smoothed points

    BodySimulation() { filter.SetCameras(cameras); }

    BodyView View(int camera, const Person& person, double at)
    {
        BodyView view;
        view.camera = camera;
        view.time = at;
        for (int j = 0; j < kJointCount; ++j) {
            cameras[camera].Project(person.Joint(j), &view.u[j], &view.v[j]);
            view.u[j] += Gaussian(random, noise_px);
            view.v[j] += Gaussian(random, noise_px);
            view.visibility[j] = 0.9;
        }
        return view;
    }

    // Both cameras at 26 frames a second, half a frame apart; `edit` can change a frame.
    void Run(double seconds, const std::function<Person(double)>& person,
             const std::function<void(BodyView*)>& edit = nullptr)
    {
        const double period = 1.0 / 26.0;
        for (double end = time + seconds; time < end; time += period) {
            for (int camera = 0; camera < 2; ++camera) {
                const double at = time + camera * period / 2;
                BodyView view = View(camera, person(at), at);
                if (edit) {
                    edit(&view);
                }
                filter.Add(view);
            }
        }
    }

    double Error(int joint, const Person& person, double at) const
    {
        Vec3 found;
        return filter.Joint(joint, at, &found) ? Length(found - person.Joint(joint)) : 1e9;
    }
};

Person StandingStill(double)
{
    return Person{};
}

void TestBodyStandingStill()
{
    BodySimulation sim;
    sim.Run(2.0, StandingStill);
    const double now = sim.time;
    for (int j = 0; j < kJointCount; ++j) {
        CHECK(sim.Error(j, Person{}, now) < 0.015);
    }
    // The plan's goal: less than 1 cm of shaking while still.
    std::vector<Vec3> feet;
    BodySimulation& s = sim;
    for (int i = 0; i < 26; ++i) {
        s.Run(1.0 / 26.0, StandingStill);
        Vec3 p;
        CHECK(s.filter.Joint(kLeftAnkle, s.time, &p));
        feet.push_back(p);
    }
    Vec3 mean;
    for (const Vec3& p : feet) mean = mean + p * (1.0 / feet.size());
    double spread = 0.0;
    for (const Vec3& p : feet) spread = std::fmax(spread, Length(p - mean));
    CHECK(spread < 0.01);

    const TrackerPose waist = sim.filter.Waist(sim.time);
    CHECK(waist.valid && !waist.holding);
    CHECK(Length(waist.position - Vec3{0.0, 0.95, 0.0}) < 0.015);
    // Facing the cameras (-z): the tracker's +x is the person's right (+x), +z behind them (+z).
    CHECK(Length(Rotate(waist.orientation, {1, 0, 0}) - Vec3{1, 0, 0}) < 0.1);
    CHECK(Length(Rotate(waist.orientation, {0, 1, 0}) - Vec3{0, 1, 0}) < 0.1);
    const TrackerPose foot = sim.filter.Foot(true, sim.time);
    CHECK(foot.valid);
    CHECK(Length(foot.position - Person{}.Joint(kLeftAnkle)) < 0.015);
    CHECK(Length(Rotate(foot.orientation, {0, 0, -1}) - Vec3{0, 0, -1}) < 0.15);  // the toes point forward

    const BodyFilter::Stats stats = sim.filter.TakeStats();
    CHECK(stats.started == kJointCount);
    CHECK(stats.updates > 1000);
    CHECK(stats.outliers < stats.updates / 50);
    CHECK(stats.swapped == 0);
}

void TestBodyTurnsAndWalks()
{
    BodySimulation sim;
    sim.Run(1.0, StandingStill);
    // Turns a quarter left while stepping 1 m sideways in 2 s.
    const double start = sim.time;
    const auto moving = [start](double t) {
        const double f = std::fmin((t - start) / 2.0, 1.0);
        Person p;
        p.root = {-1.0 * f, 0.0, 0.0};
        p.facing = f * kPi / 2;
        return p;
    };
    sim.Run(1.0, moving);
    const Person halfway = moving(sim.time);
    for (int j : {kLeftHip, kRightHip, kLeftAnkle, kRightAnkle}) {
        CHECK(sim.Error(j, halfway, sim.time) < 0.06);  // following at 0.5 m/s while turning
    }
    sim.Run(1.5, moving);
    const Person last = moving(sim.time);
    for (int j : {kLeftHip, kRightHip, kLeftAnkle, kRightAnkle}) {
        CHECK(sim.Error(j, last, sim.time) < 0.03);
    }
    const TrackerPose waist = sim.filter.Waist(sim.time);
    // Facing +x... no: facing a quarter left of -z is -x; the person's right is then -z.
    CHECK(Length(Rotate(waist.orientation, {1, 0, 0}) - Vec3{0, 0, -1}) < 0.15);
}

void TestBodyFootHiddenFromOneCamera()
{
    BodySimulation sim;
    sim.Run(1.5, StandingStill);
    // The left camera stops seeing the left foot while it is lifted 20 cm in 0.3 s.
    const double start = sim.time;
    const auto lifted = [start](double t) {
        Person p;
        p.left_foot_lift = 0.2 * std::fmin((t - start) / 0.3, 1.0);
        return p;
    };
    sim.Run(1.0, lifted, [](BodyView* view) {
        if (view->camera == 0) {
            view->visibility[kLeftAnkle] = view->visibility[kLeftHeel] = view->visibility[kLeftToe] = 0.1;
        }
    });
    // The other camera still sees it rise: across that camera's ray it is right. Its depth along
    // the ray is only roughly known, but it stays put instead of drifting away.
    Vec3 then, later;
    CHECK(sim.filter.Joint(kLeftAnkle, sim.time, &then));
    const Vec3 truth = lifted(sim.time).Joint(kLeftAnkle);
    const Vec3 ray = (truth - sim.cameras[1].position) * (1.0 / Length(truth - sim.cameras[1].position));
    const Vec3 off = then - truth;
    CHECK(Length(off - ray * Dot(off, ray)) < 0.02);
    CHECK(Length(off) < 0.12);
    sim.Run(1.0, lifted, [](BodyView* view) {
        if (view->camera == 0) {
            view->visibility[kLeftAnkle] = view->visibility[kLeftHeel] = view->visibility[kLeftToe] = 0.1;
        }
    });
    sim.filter.Joint(kLeftAnkle, sim.time, &later);
    CHECK(Length(later - then) < 0.035);
    // Once both see it again, it is right.
    sim.Run(0.5, lifted);
    CHECK(sim.Error(kLeftAnkle, lifted(sim.time), sim.time) < 0.015);
}

void TestBodyFootDrawnOnTheOther()
{
    // Standing still, the right camera has the left foot hidden behind the right leg, and in 60% of
    // its frames the pose model puts it on the right foot: the left foot stays where it is.
    BodySimulation sim;
    sim.Run(1.5, StandingStill);
    sim.filter.TakeStats();
    std::mt19937 pick(3);
    const auto drawn = [&](BodyView* view) {
        if (view->camera == 1 && pick() % 10 < 6) {
            for (int j : {kLeftAnkle, kLeftHeel, kLeftToe}) {
                view->u[j] = view->u[j + 1] + Gaussian(sim.random, 2.0);
                view->v[j] = view->v[j + 1] + Gaussian(sim.random, 2.0);
                view->visibility[j] = 0.7;
            }
        }
    };
    double worst = 0.0;
    for (int i = 0; i < 26 * 4; ++i) {
        sim.Run(1.0 / 26.0, StandingStill, drawn);
        worst = std::fmax(worst, sim.Error(kLeftAnkle, Person{}, sim.time));
    }
    CHECK(worst < 0.03);
    CHECK(sim.Error(kRightAnkle, Person{}, sim.time) < 0.015);
    CHECK(sim.filter.TakeStats().collapsed > 100);
}

void TestBodyLeftAndRightMixedUp()
{
    BodySimulation sim;
    sim.Run(1.5, StandingStill);
    // The right camera mixes up left and right for a second (as from behind), while the left
    // foot steps forward.
    const auto stepped = [](double) {
        Person p;
        p.left_foot_shift = {0.0, 0.0, -0.25};
        return p;
    };
    sim.Run(1.0, stepped, [](BodyView* view) {
        if (view->camera == 1) {
            for (int j = 0; j < kJointCount; j += 2) {
                std::swap(view->u[j], view->u[j + 1]);
                std::swap(view->v[j], view->v[j + 1]);
            }
        }
    });
    CHECK(sim.Error(kLeftAnkle, stepped(0), sim.time) < 0.03);
    CHECK(sim.Error(kRightAnkle, stepped(0), sim.time) < 0.03);
    CHECK(sim.filter.TakeStats().swapped > 20);
}

void TestBodyTurnedRoundIsSwappedBack()
{
    // The filter left with the person's left and right the wrong way round (as after the hands
    // together in front of the face): every frame is swapped to fit it, and the hips face back.
    BodySimulation sim;
    sim.Run(1.0, StandingStill);
    sim.filter.SwapSides();
    sim.filter.TakeStats();
    sim.Run(1.0, StandingStill);
    CHECK(sim.filter.TakeStats().swapped > 40);
    const Vec3 looks{0.0, -0.3, -1.0};  // ahead, a little down, as the person faces
    const auto backwards = [&]() {
        return HipsFaceBackwards(looks, sim.filter.Waist(sim.time), sim.filter.Foot(true, sim.time),
                                 sim.filter.Foot(false, sim.time));
    };
    CHECK(backwards());
    // Swapped back, the frames fit as they come.
    sim.filter.SwapSides();
    sim.Run(1.0, StandingStill);
    CHECK(sim.filter.TakeStats().swapped == 0);
    CHECK(!backwards());
    CHECK(sim.Error(kLeftHip, Person{}, sim.time) < 0.02);
    CHECK(sim.Error(kLeftAnkle, Person{}, sim.time) < 0.02);
    // Looking over the shoulder is not enough: the feet still point ahead with the hips.
    CHECK(!HipsFaceBackwards(Vec3{0.0, 0.0, 1.0}, sim.filter.Waist(sim.time), sim.filter.Foot(true, sim.time),
                             sim.filter.Foot(false, sim.time)));
}

void TestBodyLateFrames()
{
    // The right camera's frames arrive 30 ms late, after the left camera's newer ones.
    BodySimulation sim;
    std::vector<BodyView> late;
    const double period = 1.0 / 26.0;
    for (int i = 0; i < 78; ++i) {
        const double t = sim.time + i * period;
        const Person p;
        sim.filter.Add(sim.View(0, p, t));
        late.push_back(sim.View(1, p, t - 0.03));
        if (late.size() > 1) {
            sim.filter.Add(late[late.size() - 2]);
        }
    }
    const double now = sim.time + 77 * period;
    for (int j = 0; j < kJointCount; ++j) {
        CHECK(sim.Error(j, Person{}, now) < 0.02);
    }
}

void TestBodyStartsAgainWhenLost()
{
    // A frame stamped a minute ahead (a clock that jumped) would leave the joints waiting for
    // frames that never come, where they were: they are lost, and start again from both cameras
    // while the person walks 40 cm to the side.
    BodySimulation sim;
    sim.Run(1.0, StandingStill);
    sim.filter.Add(sim.View(0, Person{}, sim.time + 60.0));
    const double start = sim.time;
    const auto walking = [start](double at) {
        Person p;
        p.root = Vec3{0.4 * std::clamp((at - start) / 1.5, 0.0, 1.0), 0.0, 0.0};
        return p;
    };
    sim.Run(2.5, walking);
    CHECK(sim.Error(kLeftHip, walking(sim.time), sim.time) < 0.03);
    CHECK(sim.Error(kLeftAnkle, walking(sim.time), sim.time) < 0.03);
    CHECK(sim.filter.TakeStats().late > 0);
}

void TestBodyNeedsTwoCamerasToStart()
{
    BodySimulation sim;
    for (int i = 0; i < 26; ++i) {
        sim.filter.Add(sim.View(0, Person{}, sim.time + i / 26.0));
    }
    Vec3 p;
    CHECK(!sim.filter.Joint(kLeftHip, sim.time + 1.0, &p));
    CHECK(!sim.filter.Waist(sim.time + 1.0).valid);
}

void TestBodyHoldsThenLetsGo()
{
    BodySimulation sim;
    sim.Run(1.0, StandingStill);
    bool holding = false;
    Vec3 p;
    CHECK(sim.filter.Joint(kLeftAnkle, sim.time + 1.0, &p, &holding));
    CHECK(holding);  // nothing new for a second: held where it was
    CHECK(Length(p - Person{}.Joint(kLeftAnkle)) < 0.015);
    CHECK(!sim.filter.Joint(kLeftAnkle, sim.time + 6.0, &p));
}


// A whole session: the PS Moves through PSMoveServiceEx and two phones' frames, as the driver
// gets them.
struct SessionSimulation {
    std::vector<CameraModel> truth{LookAt({-1.6, 1.1, -2.4}, {0.0, 1.0, 0.0}, 520.0, -0.05),
                                   LookAt({1.7, 1.0, -2.2}, {0.0, 1.0, 0.0}, 500.0, -0.08)};
    std::vector<uint32_t> ids{0x11114444, 0x2222BBBB};
    BodySession session;
    std::mt19937 random{9};
    double time = 50.0;
    double next_move = 50.0;
    double move_latency = 0.04;
    double frame_age = 0.12;
    double second_camera_later = 0.0;  // its frames arrive this much older
    int moves_lit = 1;
    bool others_tracked = true;  // false: the other Moves are on but out of the PS Eye's sight
    double move_jitter = 0.0;    // of PSMoveServiceEx's positions, m
    int move_samples = 0;
    double pixel_noise = 1.0;    // of the pose's points

    // A slow loop through the room, about 0.3 m/s.
    static Vec3 MovePath(double t)
    {
        return {0.8 * std::sin(0.35 * t), 1.2 + 0.7 * std::sin(0.23 * t + 1.0), 0.6 * std::sin(0.29 * t + 2.0)};
    }

    body_protocol::Frame MakeFrame(int camera, double at, const Person* person, bool ball)
    {
        const auto noise = [this]() { return Gaussian(random, 1.0); };
        body_protocol::Frame frame;
        frame.camera = ids[camera];
        frame.width = truth[camera].width;
        frame.height = truth[camera].height;
        frame.age_seconds = frame_age + camera * second_camera_later;
        double u = 0.0, v = 0.0;
        if (ball && truth[camera].Project(MovePath(at), &u, &v) && u > 0 && v > 0 && u < frame.width &&
            v < frame.height) {
            frame.flags |= body_protocol::kFlagBall;
            frame.ball[0] = static_cast<float>((u + noise()) / frame.width);
            frame.ball[1] = static_cast<float>((v + noise()) / frame.height);
            frame.ball[2] = 0.02f;
        }
        if (person) {
            frame.flags |= body_protocol::kFlagPose;
            frame.points = body_protocol::kPoints;
            for (int j = 0; j < kJointCount; ++j) {
                truth[camera].Project(person->Joint(j), &u, &v);
                float* p = frame.point[kJointLandmark[j]];
                p[0] = static_cast<float>((u + pixel_noise * noise()) / frame.width);
                p[1] = static_cast<float>((v + pixel_noise * noise()) / frame.height);
                p[3] = 0.9f;
            }
        }
        return frame;
    }

    void Run(double seconds, const Person* person, bool ball = true)
    {
        const double period = 1.0 / 26.0;
        for (double end = time + seconds; time < end; time += period) {
            // PSMoveServiceEx: 60 positions a second, each arriving a little after the PS Eye saw it.
            for (; next_move <= time; next_move += 1.0 / 60.0, ++move_samples) {
                Vec3 jitter;
                if (move_jitter > 0.0) {
                    jitter = {Gaussian(random, move_jitter), Gaussian(random, move_jitter), Gaussian(random, move_jitter)};
                }
                session.AddMove({next_move + move_latency, 0, MovePath(next_move) + jitter, true});
                for (int id = 1; id < moves_lit; ++id) {
                    // An untracked one is only said to be on, four times a second.
                    if (others_tracked || move_samples % 15 == 0) {
                        session.AddMove({next_move + move_latency, id, MovePath(next_move) + Vec3{0.3 * id, 0, 0},
                                         others_tracked});
                    }
                }
            }
            for (int camera = 0; camera < 2; ++camera) {
                const double age = frame_age + camera * second_camera_later;
                const double at = time - age + camera * period / 2;
                if (session.AddFrame(MakeFrame(camera, at, person, ball), at + age)) {
                    ++fresh;
                }
            }
        }
    }

    int fresh = 0;
};

void TestSessionCalibratesAndChecks()
{
    SessionSimulation sim;
    sim.Run(1.0, nullptr);
    CHECK(sim.fresh == 2);
    sim.session.StartCalibration();
    sim.Run(60.0, nullptr);
    const BodySession::Progress progress = sim.session.CalibrationProgress();
    CHECK(progress.enough);
    CHECK(progress.ambiguous == 0);
    const std::vector<BodySession::CameraCalibration> results = sim.session.FinishCalibration();
    CHECK(results.size() == 2);
    for (size_t i = 0; i < results.size(); ++i) {
        const int camera = results[i].id == sim.ids[0] ? 0 : 1;
        CHECK(results[i].result.ok);
        CHECK(Length(results[i].result.camera.position - sim.truth[camera].position) < 0.03);
        CHECK(results[i].result.rms_px < 2.5);
        CHECK(sim.session.Calibrated(results[i].id));
    }

    // Afterwards the cameras put the Move's bulb where PSMoveServiceEx has it.
    sim.session.TakeReport(sim.time);
    sim.Run(10.0, nullptr);
    BodySession::Report report = sim.session.TakeReport(sim.time);
    CHECK(report.checks > 100);
    CHECK(report.check_mean_m < 0.015);
    CHECK(report.check_depth_m <= report.check_mean_m && report.check_across_m <= report.check_mean_m);
    CHECK(report.check_angle_degrees > 60.0 && report.check_angle_degrees < 85.0);
    CHECK(report.cameras.size() == 2);
    CHECK_NEAR(report.cameras[0].frames_per_second, 26.0, 0.5);
    CHECK_NEAR(report.cameras[0].age_ms, 120.0, 0.5);
    CHECK(report.cameras[0].ball_share > 0.5);
    CHECK(report.cameras[0].paired_share > 0.9);  // 19 ms apart

    // And the body comes out of the person's points.
    const Person person;
    sim.Run(2.0, &person, false);
    const TrackerPose waist = sim.session.Waist(sim.time);
    CHECK(waist.valid);
    CHECK(Length(waist.position - Vec3{0.0, 0.95, 0.0}) < 0.03);
    report = sim.session.TakeReport(sim.time);
    CHECK(report.cameras[0].person_share > 0.99);
    CHECK(report.filter.started == kJointCount);
}

void TestSessionCheckTellsCamerasCloseTogether()
{
    // As the phones of the second test: half a metre apart, side by side, so they see the Move
    // from nearly the same side and put it far worse in depth than across.
    SessionSimulation sim;
    sim.truth = {LookAt({-0.25, 1.0, -2.5}, {0.0, 1.0, 0.0}, 520.0, -0.05),
                 LookAt({0.25, 1.0, -2.5}, {0.0, 1.0, 0.0}, 500.0, -0.08)};
    sim.session.SetCamera(sim.ids[0], sim.truth[0]);
    sim.session.SetCamera(sim.ids[1], sim.truth[1]);
    sim.Run(1.0, nullptr);
    sim.session.TakeReport(sim.time);
    sim.Run(10.0, nullptr);
    const BodySession::Report report = sim.session.TakeReport(sim.time);
    CHECK(report.checks > 100);
    CHECK(report.check_angle_degrees > 8.0 && report.check_angle_degrees < 16.0);
    CHECK(report.check_depth_m > 3.0 * report.check_across_m);
}

void TestSessionNeedsOneMove()
{
    SessionSimulation sim;
    sim.moves_lit = 2;
    sim.session.StartCalibration();
    sim.Run(10.0, nullptr);
    const BodySession::Progress progress = sim.session.CalibrationProgress();
    CHECK(progress.ambiguous > 100);
    CHECK(progress.cameras.size() == 2 && progress.cameras[0].samples == 0);
    const std::vector<BodySession::CameraCalibration> results = sim.session.FinishCalibration();
    CHECK(!results[0].result.ok && !results[0].result.problem.empty());
}

void TestPoseSpread()
{
    PoseSpread spread;
    CHECK(spread.Deviation() == 0.0);
    for (int i = 0; i < 100; ++i) {
        spread.Add({i % 2 ? 0.01 : -0.01, 1.0 + 0.001 * i, 0.0});
    }
    CHECK_NEAR(spread.Mean().y, 1.0495, 1e-9);
    CHECK_NEAR(spread.lowest, 1.0, 1e-12);
    CHECK_NEAR(spread.highest, 1.099, 1e-12);
    CHECK_NEAR(spread.Deviation(), std::sqrt(0.01 * 0.01 + 0.001 * 0.001 * (100 * 100 - 1) / 12.0), 1e-9);
}

void TestSessionTrackersStillWhenStandingStill()
{
    // The fifth stage's test: standing still, hips and feet move less than 1 cm, with the pose's
    // points 2 px unsteady, each frame on its own (body_tracker.py's smoothing leaves less).
    SessionSimulation sim;
    sim.pixel_noise = 2.0;
    sim.session.SetCamera(sim.ids[0], sim.truth[0]);
    sim.session.SetCamera(sim.ids[1], sim.truth[1]);
    const Person person;
    sim.Run(3.0, &person, false);
    PoseSpread spreads[3];
    for (int i = 0; i < 10 * 26; ++i) {
        sim.Run(1.0 / 26.0, &person, false);
        for (int k = 0; k < 4; ++k) {  // published at about 120 Hz, between the frames too
            const double now = sim.time + k / 104.0;
            const TrackerPose poses[3] = {sim.session.Waist(now), sim.session.Foot(true, now),
                                          sim.session.Foot(false, now)};
            for (int t = 0; t < 3; ++t) {
                CHECK(poses[t].valid);
                spreads[t].Add(poses[t].position);
            }
        }
    }
    for (const PoseSpread& s : spreads) {
        CHECK(s.Deviation() < 0.01);
    }
    CHECK_NEAR(spreads[1].Mean().y, person.Joint(kLeftAnkle).y, 0.02);
}

void TestSessionNeedsTheOtherMoveOff()
{
    // As in the second test's check: the other Move on in the other hand, out of the PS Eye's
    // sight. The phones may see its bulb, so neither calibrating nor checking takes any.
    SessionSimulation sim;
    sim.moves_lit = 2;
    sim.others_tracked = false;
    sim.session.StartCalibration();
    sim.Run(10.0, nullptr);
    const BodySession::Progress progress = sim.session.CalibrationProgress();
    CHECK(progress.ambiguous > 100);
    CHECK(progress.cameras.size() == 2 && progress.cameras[0].samples == 0);
    sim.session.CancelCalibration();

    sim.session.SetCamera(sim.ids[0], sim.truth[0]);
    sim.session.SetCamera(sim.ids[1], sim.truth[1]);
    sim.session.TakeReport(sim.time);
    sim.Run(10.0, nullptr);
    BodySession::Report report = sim.session.TakeReport(sim.time);
    CHECK(report.checks == 0);
    CHECK(report.check_ambiguous > 100);

    // Turned off, it soon stops counting.
    sim.moves_lit = 1;
    sim.Run(1.0, nullptr);
    sim.session.TakeReport(sim.time);
    sim.Run(10.0, nullptr);
    report = sim.session.TakeReport(sim.time);
    CHECK(report.checks > 100);
    CHECK(report.check_ambiguous == 0);
}

void TestSessionCalibratesWithAJitteryMove()
{
    // PSMoveServiceEx's positions jitter a few millimetres from one to the next: that alone must
    // not make the Move look too fast to calibrate or check with.
    SessionSimulation sim;
    sim.move_jitter = 0.004;
    sim.session.StartCalibration();
    sim.Run(60.0, nullptr);
    const BodySession::Progress progress = sim.session.CalibrationProgress();
    CHECK(progress.enough);
    const std::vector<BodySession::CameraCalibration> results = sim.session.FinishCalibration();
    for (const BodySession::CameraCalibration& camera : results) {
        CHECK(camera.result.ok);
    }
    sim.session.TakeReport(sim.time);
    sim.Run(10.0, nullptr);
    const BodySession::Report report = sim.session.TakeReport(sim.time);
    CHECK(report.checks > 300);  // judged between two samples, about two thirds look too fast
    CHECK(report.check_mean_m < 0.02);
}

void TestSessionDropsAMovedCamerasCalibration()
{
    // Calibrating again, both cameras see the bulb too little to calibrate; one was moved 40 cm
    // since its last calibration, which no longer fits and is dropped; the other keeps its own.
    SessionSimulation sim;
    CameraModel moved = sim.truth[0];
    moved.position = moved.position + Vec3{0.4, 0.0, 0.0};
    sim.session.SetCamera(sim.ids[0], moved);
    sim.session.SetCamera(sim.ids[1], sim.truth[1]);
    sim.session.StartCalibration();
    sim.Run(3.0, nullptr);
    const std::vector<BodySession::CameraCalibration> results = sim.session.FinishCalibration();
    CHECK(results.size() == 2);
    for (const BodySession::CameraCalibration& c : results) {
        CHECK(!c.result.ok && c.had_old);
        CHECK(c.dropped_old == (c.id == sim.ids[0]));
    }
    CHECK(!sim.session.Calibrated(sim.ids[0]));
    CHECK(sim.session.Calibrated(sim.ids[1]));
}

void TestSessionForgetsACalibrationThatNoLongerFits()
{
    SessionSimulation sim;
    sim.session.SetCamera(sim.ids[0], sim.truth[0]);
    sim.session.SetCamera(sim.ids[1], sim.truth[1]);
    sim.Run(0.5, nullptr);
    CHECK(sim.session.Calibrated(sim.ids[0]));
    // The phone was turned: its picture is now the other way round.
    body_protocol::Frame frame = sim.MakeFrame(0, sim.time, nullptr, false);
    std::swap(frame.width, frame.height);
    sim.session.AddFrame(frame, sim.time + sim.frame_age);
    CHECK(!sim.session.Calibrated(sim.ids[0]));
    CHECK(sim.session.Calibrated(sim.ids[1]));
}


void TestSessionSeesTheBulbs()
{
    // The bulbs by both hands, from a calibrated camera: a view of each, for the controllers.
    SessionSimulation sim;
    sim.session.SetCamera(sim.ids[0], sim.truth[0]);
    const Vec3 bulbs[2] = {{-0.3, 1.2, 0.2}, {0.25, 0.9, 0.1}};
    const auto frame_with_bulbs = [&](int camera) {
        body_protocol::Frame frame = sim.MakeFrame(camera, sim.time, nullptr, false);
        frame.flags |= body_protocol::kFlagBulbs;
        frame.bulbs = 2;
        for (int b = 0; b < 2; ++b) {
            double u = 0.0, v = 0.0;
            sim.truth[camera].Project(bulbs[b], &u, &v);
            frame.bulb[b][0] = static_cast<float>(u / frame.width);
            frame.bulb[b][1] = static_cast<float>(v / frame.height);
            frame.bulb[b][2] = 0.02f;
        }
        return frame;
    };
    sim.session.AddFrame(frame_with_bulbs(0), sim.time + sim.frame_age);
    std::vector<HeadSighting> sightings;
    sim.session.TakeBulbSightings(&sightings);
    CHECK(sightings.size() == 2);
    for (size_t b = 0; b < sightings.size() && b < 2; ++b) {
        const HeadSighting& s = sightings[b];
        CHECK(s.camera == sim.ids[0]);
        CHECK_NEAR(s.time, sim.time, 1e-6);
        const Vec3 ray = s.model.Ray(s.u, s.v), to = bulbs[b] - s.model.position;
        CHECK(Length(to - ray * Dot(to, ray)) < 0.002);
        CHECK(s.noise >= 1.5);
    }
    sim.session.TakeBulbSightings(&sightings);
    CHECK(sightings.empty());
    // Not from a camera that is not calibrated, nor while calibrating.
    sim.session.AddFrame(frame_with_bulbs(1), sim.time + sim.frame_age);
    sim.session.StartCalibration();
    sim.session.AddFrame(frame_with_bulbs(0), sim.time + sim.frame_age);
    sim.session.TakeBulbSightings(&sightings);
    CHECK(sightings.empty());
}

void TestSessionPairsFramesWhicheverArrivesFirst()
{
    // As the two phones of the first test: one's frames arrive 20 ms older than the other's.
    SessionSimulation sim;
    sim.second_camera_later = 0.02;
    sim.Run(1.0, nullptr);
    sim.session.TakeReport(sim.time);
    sim.Run(10.0, nullptr);
    const BodySession::Report report = sim.session.TakeReport(sim.time);
    CHECK(report.cameras.size() == 2);
    for (const BodySession::CameraReport& camera : report.cameras) {
        CHECK(camera.paired_share > 0.95);
    }
    CHECK_NEAR(report.cameras[1].age_ms - report.cameras[0].age_ms, sim.ids[1] > sim.ids[0] ? 20.0 : -20.0, 0.5);
}

}  // namespace

// A Move held in the right hand, the hand turning: the bulb 18 cm from the wrist along the Move.
struct HandSimulation {
    MoveHand hand{false};
    Vec3 bulb_from_wrist{0.0, 0.02, -0.18};  // controller axes
    double time = 100.0;

    // The Move pointing forward and a little up, swinging left and right.
    static Quat Controller(double t) { return AxisAngle(Vec3{0, 1, 0}, 0.6 * std::sin(1.7 * t)) * AxisAngle(Vec3{1, 0, 0}, 0.2); }
    static Vec3 WristAt(double t) { return Vec3{0.25 + 0.05 * std::sin(0.9 * t), 1.0, -0.3}; }

    // Runs `seconds`; `seen(t)` whether the PS Eye sees the bulb. Returns the largest error and step.
    void Run(double seconds, const std::function<bool(double)>& seen, bool wrists, double* max_error, double* max_step,
             int held_wrist = 1)
    {
        Vec3 previous;
        bool have_previous = false;
        const double start = time;
        for (; time < start + seconds; time += 0.004) {
            const double t = time - start;
            const Quat controller = Controller(time);
            const Vec3 bulb = WristAt(time) + Rotate(controller, bulb_from_wrist);
            MoveState move;
            move.time = time;
            move.tracking = seen(t);
            move.position = bulb;
            move.orientation = controller;
            Wrists w;
            w.valid[held_wrist] = wrists;
            w.position[held_wrist] = WristAt(time);
            w.valid[1 - held_wrist] = wrists;
            w.position[1 - held_wrist] = WristAt(time) + Vec3{-0.45, 0.0, 0.0};
            hand.Update(move, w, time);
            const HandPose pose = hand.Pose(time);
            CHECK(pose.valid);
            if (max_error) {
                *max_error = std::max(*max_error, Length(pose.position - bulb));
            }
            if (have_previous && max_step) {
                *max_step = std::max(*max_step, Length(pose.position - previous));
            }
            previous = pose.position;
            have_previous = true;
        }
    }
};

void TestMoveHandAxes()
{
    // PSMoveServiceEx's identity is the Move pointing along -z, buttons up: SteamVR's too.
    MoveHand hand(true);
    MoveState move;
    move.tracking = true;
    move.orientation = AxisAngle(Vec3{0, 1, 0}, 0.7);
    hand.Update(move, Wrists{}, 1.0);
    CHECK(Length(Rotate(hand.Pose(1.0).orientation, Vec3{0, 0, -1}) - Rotate(move.orientation, Vec3{0, 0, -1})) < 1e-9);
    // Tilted by the setting: a Move whose identity were upright points forward lying flat.
    MoveHand tilted(true, 90.0);
    move.orientation = AxisAngle(Vec3{1, 0, 0}, -kPi / 2);
    tilted.Update(move, Wrists{}, 1.0);
    const HandPose pose = tilted.Pose(1.0);
    CHECK(Length(Rotate(pose.orientation, Vec3{0, 0, -1}) - Vec3{0, 0, -1}) < 1e-9);
    CHECK(Length(Rotate(pose.orientation, Vec3{0, 1, 0}) - Vec3{0, 1, 0}) < 1e-9);
    // And back: the rotation PSMoveServiceEx would give for a controller's.
    const Quat back = tilted.MoveOrientation(pose.orientation);
    CHECK(Length(Rotate(back, Vec3{0, 1, 0}) - Rotate(move.orientation, Vec3{0, 1, 0})) < 1e-9);
}

void TestMoveHandFallsBackOnWrist()
{
    // Seen for 3 s, then hidden for 2 s while turning: the bulb stays on the wrist where it was
    // learned to sit, gliding over rather than jumping; seen again, back on the bulb.
    HandSimulation sim;
    double error = 0.0, step = 0.0;
    sim.Run(3.0, [](double) { return true; }, true, nullptr, nullptr);
    CHECK(Length(sim.hand.BulbFromWrist() - sim.bulb_from_wrist) < 0.005);
    sim.Run(2.0, [](double) { return false; }, true, &error, &step);
    CHECK(error < 0.005);
    CHECK(step < 0.01);
    CHECK(sim.hand.TakeStats().from_wrist > 400);
    error = step = 0.0;
    sim.Run(1.0, [](double) { return true; }, true, &error, &step);
    CHECK(error < 0.005);
}

void TestMoveHandFindsItsWrist()
{
    // Started as the left hand, but held in the right: the right wrist, closer, takes over.
    HandSimulation sim;
    sim.hand = MoveHand(true);
    sim.Run(2.0, [](double) { return true; }, true, nullptr, nullptr);
    CHECK(sim.hand.wrist() == 1);
    double error = 0.0;
    sim.Run(1.0, [](double) { return false; }, true, &error, nullptr);
    CHECK(error < 0.005);
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
        hand.Update(reported, Wrists{}, time);
        max_jump = std::max(max_jump, Length(hand.Pose(time).position - move.position));
    }
    CHECK(max_jump < 0.01);
    CHECK(hand.TakeStats().jumps == 6);
    move.position = Vec3{0.2, 1.0, 1.2};  // moved 70 cm at once, and stays
    for (int i = 0; i < 60; ++i, time += 1.0 / 60.0) {
        move.time = time;
        hand.Update(move, Wrists{}, time);
    }
    CHECK(Length(hand.Pose(time).position - move.position) < 0.01);
}

void TestMoveHandHoldsWithoutWrist()
{
    // Neither the bulb nor the wrist: it stays where it was, still turning, without jumping.
    HandSimulation sim;
    sim.Run(1.0, [](double) { return true; }, true, nullptr, nullptr);
    const Vec3 last = sim.hand.Pose(sim.time).position;
    double step = 0.0;
    sim.Run(1.0, [](double) { return false; }, false, nullptr, &step);
    CHECK(Length(sim.hand.Pose(sim.time).position - last) < 1e-9);
    CHECK(step < 0.01);
    CHECK(sim.hand.TakeStats().held > 200);
}

int main()
{
    TestBodyPacket();
    TestCameraProjectsAndCastsRays();
    TestCalibrationFindsTheCamera();
    TestCalibrationNeedsAgreeingSamples();
    TestTriangulatesFromTwoCameras();
    TestBodyStandingStill();
    TestBodyTurnsAndWalks();
    TestBodyFootHiddenFromOneCamera();
    TestBodyFootDrawnOnTheOther();
    TestBodyLeftAndRightMixedUp();
    TestBodyLateFrames();
    TestBodyTurnedRoundIsSwappedBack();
    TestBodyStartsAgainWhenLost();
    TestBodyNeedsTwoCamerasToStart();
    TestBodyHoldsThenLetsGo();
    TestSessionCalibratesAndChecks();
    TestSessionCheckTellsCamerasCloseTogether();
    TestSessionNeedsOneMove();
    TestPoseSpread();
    TestSessionTrackersStillWhenStandingStill();
    TestSessionNeedsTheOtherMoveOff();
    TestSessionCalibratesWithAJitteryMove();
    TestSessionDropsAMovedCamerasCalibration();
    TestSessionForgetsACalibrationThatNoLongerFits();
    TestSessionSeesTheBulbs();
    TestSessionPairsFramesWhicheverArrivesFirst();
    TestMoveHandAxes();
    TestMoveHandFallsBackOnWrist();
    TestMoveHandFindsItsWrist();
    TestMoveHandHoldsWithoutWrist();
    TestMoveHandIgnoresBulbJumps();
    if (g_failures == 0) {
        std::printf("All body tests passed\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
}
