#include "body_filter.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kGate = 16.0;              // squared Mahalanobis distance; above it a point is an outlier
constexpr double kCostCap = 25.0;           // so one wild point does not decide left and right alone
constexpr double kSwapMargin = 4.0;
constexpr int kMinCompared = 3;
constexpr double kMaxLateSeconds = 0.15;    // frames this far behind the filter are too old to use
constexpr double kStartWindowSeconds = 0.1;  // two cameras' frames this close start a joint
constexpr double kStartMaxGapM = 0.08;      // if their rays pass this close
constexpr double kHoldSeconds = 0.5;
constexpr double kLostSeconds = 5.0;
constexpr int kMissesToRestart = 10;
constexpr double kMaxExtrapolationSeconds = 0.1;
constexpr double kStartPositionVariance = 0.03 * 0.03;
constexpr double kStartVelocityVariance = 0.5;
constexpr double kVelocityConfidence = 3.0;
constexpr double kMaxSpeed = 8.0;  // m/s
constexpr double kVelocityMemorySeconds = 0.3;
constexpr double kJacobianStep = 1e-4;
// The agility scale follows the normalized innovations (2 on average when the model fits): it
// shrinks while they are smaller and grows quickly when the joint starts moving.
constexpr double kMinAgility = 0.003;
constexpr double kMaxAgility = 20.0;
constexpr double kAgilityRate = 0.3;

// How quickly each joint changes speed, m^2/s^3: the hips and shoulders move the whole body,
// the feet swing, the hands more.
constexpr double kAgility[kJointCount] = {2, 2, 2, 2, 6, 6, 10, 10, 10, 10, 10, 10, 20, 20};

int Partner(int joint)
{
    return joint ^ 1;
}

// The rotation whose columns are the given axes.
Quat FromAxes(Vec3 x, Vec3 y, Vec3 z)
{
    return Mat3ToQuat(Transpose(Rows(x, y, z)));
}

// How far apart two cameras' rays pass, where they come closest; false if nearly parallel.
bool RayGap(const CameraModel& a, double ua, double va, const CameraModel& b, double ub, double vb, Vec3* point,
            double* gap)
{
    if (!Triangulate({{&a, ua, va}, {&b, ub, vb}}, point)) {
        return false;
    }
    const auto off_ray = [&](const CameraModel& camera, double u, double v) {
        const Vec3 d = camera.Ray(u, v);
        const Vec3 to = *point - camera.position;
        if (Dot(to, d) <= 0.0) {
            return 1e9;  // behind the camera
        }
        return Length(to - d * Dot(to, d));
    };
    *gap = off_ray(a, ua, va) + off_ray(b, ub, vb);
    return true;
}

}  // namespace

BodyFilter::BodyFilter(const BodyFilterConfig& config) : config_(config) {}

void BodyFilter::SetCameras(const std::vector<CameraModel>& cameras)
{
    cameras_ = cameras;
    last_views_.assign(cameras.size(), BodyView{});
    has_view_.assign(cameras.size(), false);
    joints_ = {};
}

void BodyFilter::Predict(State* state, double to, int joint) const
{
    const double dt = to - state->time;
    if (dt <= 0.0) {
        return;
    }
    // The speed fades unless frames keep it up, so a direction no camera measures (along the
    // ray of the only camera that sees a joint) does not drift on: p += b v, v *= a.
    const double a = std::exp(-dt / kVelocityMemorySeconds);
    const double b = kVelocityMemorySeconds * (1.0 - a);
    auto& p = state->covariance;
    // P = F P F^T + Q, with F = [[I, b I], [0, a I]].
    double fp[6][6];
    for (int i = 0; i < 6; ++i)
        for (int k = 0; k < 6; ++k) fp[i][k] = i < 3 ? p[i][k] + b * p[i + 3][k] : a * p[i][k];
    for (int i = 0; i < 6; ++i)
        for (int k = 0; k < 6; ++k) p[i][k] = k < 3 ? fp[i][k] + b * fp[i][k + 3] : a * fp[i][k];
    const double q = kAgility[joint] * config_.agility * state->agility;
    for (int a = 0; a < 3; ++a) {
        p[a][a] += q * dt * dt * dt / 3.0;
        p[a][a + 3] += q * dt * dt / 2.0;
        p[a + 3][a] += q * dt * dt / 2.0;
        p[a + 3][a + 3] += q * dt;
    }
    state->position = state->position + state->velocity * b;
    state->velocity = state->velocity * a;
    state->time = to;
}

double BodyFilter::Update(State* state, int joint, const CameraModel& camera, double time, double u, double v,
                          double visibility, bool apply)
{
    State s = *state;
    Predict(&s, time, joint);
    // A frame older than the state (the other camera's newer one came first) is compared with
    // where the joint was then.
    const double late = s.time - time;
    if (late > kMaxLateSeconds) {
        stats_.late += apply;
        return -1.0;
    }
    const Vec3 at = s.position - s.velocity * late;
    double pu = 0.0, pv = 0.0;
    if (!camera.Project(at, &pu, &pv)) {
        // Behind a camera that sees it in front: as wrong as an outlier.
        if (apply && ++state->misses >= kMissesToRestart) {
            state->live = false;
        }
        return -1.0;
    }
    double h[2][6];
    for (int a = 0; a < 3; ++a) {
        Vec3 step;
        (a == 0 ? step.x : a == 1 ? step.y : step.z) = kJacobianStep;
        double su = 0.0, sv = 0.0;
        if (!camera.Project(at + step, &su, &sv)) {
            return -1.0;
        }
        h[0][a] = (su - pu) / kJacobianStep;
        h[1][a] = (sv - pv) / kJacobianStep;
        h[0][a + 3] = -late * h[0][a];
        h[1][a + 3] = -late * h[1][a];
    }
    const auto& p = s.covariance;
    double pht[6][2];
    for (int i = 0; i < 6; ++i)
        for (int r = 0; r < 2; ++r) {
            pht[i][r] = 0.0;
            for (int k = 0; k < 6; ++k) pht[i][r] += p[i][k] * h[r][k];
        }
    const double sigma = config_.pixel_noise * std::max(camera.width, camera.height) / 640.0 /
                         std::max(visibility, config_.min_visibility);
    double sm[2][2];
    for (int r = 0; r < 2; ++r)
        for (int c = 0; c < 2; ++c) {
            sm[r][c] = r == c ? sigma * sigma : 0.0;
            for (int k = 0; k < 6; ++k) sm[r][c] += h[r][k] * pht[k][c];
        }
    const double det = sm[0][0] * sm[1][1] - sm[0][1] * sm[1][0];
    if (!(det > 0.0)) {
        return -1.0;
    }
    const double inverse[2][2] = {{sm[1][1] / det, -sm[0][1] / det}, {-sm[1][0] / det, sm[0][0] / det}};
    const double y[2] = {u - pu, v - pv};
    const double distance = y[0] * (inverse[0][0] * y[0] + inverse[0][1] * y[1]) +
                            y[1] * (inverse[1][0] * y[0] + inverse[1][1] * y[1]);
    if (!apply) {
        return distance;
    }
    if (distance > kGate) {
        ++stats_.outliers;
        if (++state->misses >= kMissesToRestart) {
            state->live = false;  // it moved away from the filter: start it again
        }
        // Either a wrong point or a joint that started moving faster than the filter expects:
        // widen it, so a few more in the same place get in.
        state->agility = std::min(state->agility * std::exp(4.0 * kAgilityRate), kMaxAgility);
        for (auto& row : state->covariance)
            for (double& x : row) x *= 2.0;
        return distance;
    }
    double gain[6][2];
    for (int i = 0; i < 6; ++i)
        for (int c = 0; c < 2; ++c) gain[i][c] = pht[i][0] * inverse[0][c] + pht[i][1] * inverse[1][c];
    const double dx[6] = {gain[0][0] * y[0] + gain[0][1] * y[1], gain[1][0] * y[0] + gain[1][1] * y[1],
                          gain[2][0] * y[0] + gain[2][1] * y[1], gain[3][0] * y[0] + gain[3][1] * y[1],
                          gain[4][0] * y[0] + gain[4][1] * y[1], gain[5][0] * y[0] + gain[5][1] * y[1]};
    s.position = s.position + Vec3{dx[0], dx[1], dx[2]};
    s.velocity = s.velocity + Vec3{dx[3], dx[4], dx[5]};
    const double speed = Length(s.velocity);
    if (speed > kMaxSpeed) {
        s.velocity = s.velocity * (kMaxSpeed / speed);
    }
    // P = P - K (H P), and H P is (P H^T)^T.
    double updated[6][6];
    for (int i = 0; i < 6; ++i)
        for (int k = 0; k < 6; ++k) updated[i][k] = p[i][k] - (gain[i][0] * pht[k][0] + gain[i][1] * pht[k][1]);
    for (int i = 0; i < 6; ++i)
        for (int k = 0; k < 6; ++k) s.covariance[i][k] = 0.5 * (updated[i][k] + updated[k][i]);
    s.last_update = std::max(s.last_update, time);
    s.misses = 0;
    s.agility = std::clamp(s.agility * std::exp(kAgilityRate * std::clamp(distance / 2.0 - 1.0, -1.0, 4.0)),
                           kMinAgility, kMaxAgility);
    *state = s;
    ++stats_.updates;
    return distance;
}

void BodyFilter::Add(BodyView view)
{
    if (view.camera < 0 || view.camera >= static_cast<int>(cameras_.size())) {
        return;
    }
    const CameraModel& camera = cameras_[view.camera];
    const auto seen = [&](int joint) { return view.visibility[joint] >= config_.min_visibility; };

    // Left and right: whichever way round agrees better with the joints being followed.
    double straight = 0.0, crossed = 0.0;
    int compared = 0;
    for (int j = 0; j < kJointCount; ++j) {
        const int other = Partner(j);
        if (!joints_[j].live || !seen(j) || !seen(other)) {
            continue;
        }
        const double as_is = Update(&joints_[j], j, camera, view.time, view.u[j], view.v[j], view.visibility[j], false);
        const double swapped =
            Update(&joints_[j], j, camera, view.time, view.u[other], view.v[other], view.visibility[other], false);
        if (as_is >= 0.0 && swapped >= 0.0) {
            straight += std::min(as_is, kCostCap);
            crossed += std::min(swapped, kCostCap);
            ++compared;
        }
    }
    if (compared >= kMinCompared && crossed + kSwapMargin < straight) {
        for (int j = 0; j < kJointCount; j += 2) {
            std::swap(view.u[j], view.u[j + 1]);
            std::swap(view.v[j], view.v[j + 1]);
            std::swap(view.visibility[j], view.visibility[j + 1]);
        }
        ++stats_.swapped;
    }

    bool missing = false;
    for (int j = 0; j < kJointCount; ++j) {
        if (!seen(j)) {
            continue;
        }
        if (joints_[j].live) {
            Update(&joints_[j], j, camera, view.time, view.u[j], view.v[j], view.visibility[j], true);
        }
        missing = missing || !Usable(joints_[j], view.time, nullptr);
    }
    if (missing) {
        Start(view);
    }
    last_views_[view.camera] = view;
    has_view_[view.camera] = true;
}

// Starts the joints that are not followed from this frame and another camera's at about the
// same time, where their rays meet. Left and right are taken as the cameras say: how close the
// rays pass cannot tell them apart, as a left and a right joint at the same height lie on the
// same line across both pictures, and each frame's check against the filter sorts out a camera
// that has them the wrong way round afterwards.
void BodyFilter::Start(const BodyView& view)
{
    const CameraModel& camera = cameras_[view.camera];
    const auto seen = [&](const BodyView& b, int joint) { return b.visibility[joint] >= config_.min_visibility; };
    for (size_t c = 0; c < cameras_.size(); ++c) {
        const BodyView& other = last_views_[c];
        if (static_cast<int>(c) == view.camera || !has_view_[c] || std::fabs(other.time - view.time) > kStartWindowSeconds) {
            continue;
        }
        const CameraModel& other_camera = cameras_[c];
        const double time = std::max(view.time, other.time);
        for (int j = 0; j < kJointCount; ++j) {
            Vec3 point;
            double gap = 0.0;
            // A joint lost a while (out of view, or stuck ahead of the frames) starts again too.
            if (Usable(joints_[j], time, nullptr) || !seen(view, j) || !seen(other, j) ||
                !RayGap(camera, view.u[j], view.v[j], other_camera, other.u[j], other.v[j], &point, &gap) ||
                gap > kStartMaxGapM) {
                continue;
            }
            State& s = joints_[j];
            s = State{};
            s.live = true;
            s.position = point;
            for (int a = 0; a < 3; ++a) {
                s.covariance[a][a] = kStartPositionVariance;
                s.covariance[a + 3][a + 3] = kStartVelocityVariance;
            }
            s.time = time;
            s.last_update = time;
            ++stats_.started;
        }
    }
}

Vec3 BodyFilter::Output(const State& state, double now) const
{
    if (now - state.last_update > kHoldSeconds) {
        return state.position;  // holding: no extrapolating a speed that is not being measured
    }
    // Only as much of the velocity as stands out from its own uncertainty, so a still body does
    // not shake with the noise of the speed estimate.
    Mat3 vv;
    for (int i = 0; i < 3; ++i)
        for (int k = 0; k < 3; ++k) vv.m[i][k] = state.covariance[i + 3][k + 3];
    Mat3 inverse;
    double gain = 0.0;
    if (Inverse(vv, &inverse)) {
        const double significance = Dot(state.velocity, inverse * state.velocity);
        gain = config_.prediction * significance / (significance + kVelocityConfidence);
    }
    const double ahead = std::clamp(now - state.time, 0.0, kMaxExtrapolationSeconds);
    return state.position + state.velocity * (gain * ahead);
}

bool BodyFilter::Usable(const State& state, double now, bool* holding) const
{
    // Lost: nothing new for a while, or stamped far ahead of now (a clock that jumped).
    if (!state.live || std::fabs(now - state.last_update) > kLostSeconds) {
        return false;
    }
    if (holding) {
        *holding = now - state.last_update > kHoldSeconds;
    }
    return true;
}

bool BodyFilter::Joint(int joint, double now, Vec3* position, bool* holding) const
{
    if (joint < 0 || joint >= kJointCount || !Usable(joints_[joint], now, holding)) {
        return false;
    }
    *position = Output(joints_[joint], now);
    return true;
}

TrackerPose BodyFilter::Waist(double now) const
{
    TrackerPose pose;
    Vec3 left, right;
    bool hold_left = false, hold_right = false;
    if (!Joint(kLeftHip, now, &left, &hold_left) || !Joint(kRightHip, now, &right, &hold_right)) {
        return pose;
    }
    pose.valid = true;
    pose.holding = hold_left || hold_right;
    pose.position = (left + right) * 0.5;
    pose.velocity = (joints_[kLeftHip].velocity + joints_[kRightHip].velocity) * 0.5;
    const Vec3 x = Unit(right - left);
    Vec3 up{0.0, 1.0, 0.0};
    Vec3 left_shoulder, right_shoulder;
    if (Joint(kLeftShoulder, now, &left_shoulder) && Joint(kRightShoulder, now, &right_shoulder)) {
        up = (left_shoulder + right_shoulder) * 0.5 - pose.position;
    }
    Vec3 y = Unit(up - x * Dot(up, x));
    if (Length(y) < 0.5) {
        y = Unit(Vec3{0.0, 1.0, 0.0} - x * x.y);
    }
    pose.orientation = FromAxes(x, y, Cross(x, y));
    return pose;
}

TrackerPose BodyFilter::Foot(bool left, double now) const
{
    const int side = left ? 0 : 1;
    TrackerPose pose;
    Vec3 ankle;
    if (!Joint(kLeftAnkle + side, now, &ankle, &pose.holding)) {
        return pose;
    }
    pose.valid = true;
    pose.position = ankle;
    pose.velocity = joints_[kLeftAnkle + side].velocity;
    Vec3 heel, toe, knee;
    const bool has_heel = Joint(kLeftHeel + side, now, &heel);
    const bool has_toe = Joint(kLeftToe + side, now, &toe);
    Vec3 forward{0.0, 0.0, -1.0};
    if (has_toe && has_heel) {
        forward = toe - heel;
    } else if (has_toe) {
        forward = toe - ankle;
    } else {
        const TrackerPose waist = Waist(now);
        if (waist.valid) {
            forward = Rotate(waist.orientation, Vec3{0.0, 0.0, -1.0});
        }
    }
    const Vec3 z = Unit(forward * -1.0);
    Vec3 up = Joint(kLeftKnee + side, now, &knee) ? knee - ankle : Vec3{0.0, 1.0, 0.0};
    Vec3 y = Unit(up - z * Dot(up, z));
    if (Length(y) < 0.5) {
        y = Unit(Vec3{0.0, 1.0, 0.0} - z * z.y);
    }
    pose.orientation = FromAxes(Cross(y, z), y, z);
    return pose;
}

TrackerPose BodyFilter::Wrist(bool left, double now) const
{
    TrackerPose pose;
    const int joint = left ? kLeftWrist : kRightWrist;
    if (Joint(joint, now, &pose.position, &pose.holding)) {
        pose.valid = true;
        pose.velocity = joints_[joint].velocity;
    }
    return pose;
}

BodyFilter::Stats BodyFilter::TakeStats()
{
    const Stats stats = stats_;
    stats_ = Stats{};
    return stats;
}
