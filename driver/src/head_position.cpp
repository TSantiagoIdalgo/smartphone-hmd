#include "head_position.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr size_t kHistorySize = 512;        // phone orientations, about 1 s
constexpr double kTimingNoiseSeconds = 0.010;  // uncertainty of the camera latency
constexpr double kGate = 16.0;              // squared Mahalanobis distance; above it a sample is an outlier
constexpr double kResetWindowSeconds = 0.25;  // this many outliers this close together...
constexpr size_t kResetCount = 5;
constexpr double kResetSpreadM = 0.05;      // ...and this close to each other mean the head really moved
constexpr double kLostAfterSeconds = 0.15;
constexpr double kMaxExtrapolationSeconds = 0.10;
constexpr double kMaxExtrapolationM = 0.08;
constexpr double kMaxSpeed = 3.0;           // m/s
constexpr double kVelocityConfidence = 3.0;  // squared Mahalanobis length of the velocity at half prediction
constexpr double kInitialVelocityVariance = 0.25;
constexpr double kBlendSeconds = 0.15;
constexpr double kMinEyeHeight = 0.2;
constexpr double kMaxEyeHeight = 2.4;
constexpr double kMinCameraDistance = 0.3;
constexpr double kMaxCameraDistance = 4.0;
constexpr double kReferenceDistance = 2.0;  // the noise settings are given at this distance

Mat3 Symmetric(const Mat3& a)
{
    return (a + Transpose(a)) * 0.5;
}

}  // namespace

HeadPositionEstimator::HeadPositionEstimator(const HeadPositionConfig& config)
    : config_(config), light_from_neck_(config.eyes_from_neck - config.eyes_from_light)
{
}

void HeadPositionEstimator::AddOrientation(double time, Quat orientation)
{
    if (!history_.empty()) {
        time = std::max(time, history_.back().time);
    }
    history_.push_back({time, orientation});
    if (history_.size() > kHistorySize) {
        history_.pop_front();
    }
}

void HeadPositionEstimator::SetCamera(bool valid, Vec3 position, int cameras)
{
    camera_valid_ = valid;
    camera_ = position;
    cameras_ = cameras;
}

Quat HeadPositionEstimator::OrientationAt(double time) const
{
    if (history_.empty()) {
        return Quat{};
    }
    if (time >= history_.back().time) {
        return history_.back().orientation;
    }
    for (size_t i = history_.size() - 1; i > 0; --i) {
        const TimedOrientation& a = history_[i - 1];
        if (a.time <= time) {
            const TimedOrientation& b = history_[i];
            const double span = b.time - a.time;
            return Slerp(a.orientation, b.orientation, span > 0.0 ? (time - a.time) / span : 1.0);
        }
    }
    return history_.front().orientation;
}

// One camera measures sideways positions well and distance poorly (from the apparent size of the
// ball), and both get worse with distance. PSMoveServiceEx interpolates between camera frames, so
// several samples per frame carry the same information: each one counts for less.
Mat3 HeadPositionEstimator::MeasurementNoise(Vec3 light, Vec3 world_angular_velocity, double since_previous) const
{
    Mat3 noise;
    const double distance = camera_valid_ ? Length(light - camera_) : 0.0;
    if (cameras_ == 1 && distance > 1e-6) {
        const Vec3 along = (light - camera_) * (1.0 / distance);
        const double ratio = distance / kReferenceDistance;
        const double lateral = config_.lateral_noise_m * ratio;
        const double depth = config_.depth_noise_m * ratio * ratio;
        noise = Mat3::Identity(lateral * lateral) + Outer(along, along) * (depth * depth - lateral * lateral);
    } else if (cameras_ >= 2) {
        noise = Mat3::Identity(config_.lateral_noise_m * config_.lateral_noise_m);
    } else {
        noise = Mat3::Identity(config_.depth_noise_m * config_.depth_noise_m);
    }
    // A wrong latency guess turns into error while the head turns.
    const double timing = Length(world_angular_velocity) * Length(light_from_neck_) * kTimingNoiseSeconds;
    noise = noise + Mat3::Identity(timing * timing);
    const double frames = config_.camera_period_seconds / std::max(since_previous, 1e-4);
    return noise * std::clamp(frames, 1.0, 10.0);
}

void HeadPositionEstimator::Predict(double to)
{
    const double dt = to - state_time_;
    if (dt <= 0.0) {
        return;
    }
    const double q = config_.agility;
    const Mat3 pp = p_pp_ + (p_pv_ + Transpose(p_pv_)) * dt + p_vv_ * (dt * dt) + Mat3::Identity(q * dt * dt * dt / 3.0);
    const Mat3 pv = p_pv_ + p_vv_ * dt + Mat3::Identity(q * dt * dt / 2.0);
    const Mat3 vv = p_vv_ + Mat3::Identity(q * dt);
    p_pp_ = pp;
    p_pv_ = pv;
    p_vv_ = vv;
    pivot_ = pivot_ + velocity_ * dt;
    state_time_ = to;
}

void HeadPositionEstimator::Reset(double time, double measured_at, Vec3 pivot, const Mat3& noise)
{
    pivot_ = pivot;
    velocity_ = Vec3{};
    p_pp_ = noise;
    p_pv_ = Mat3{};
    p_vv_ = Mat3::Identity(kInitialVelocityVariance);
    state_time_ = measured_at;
    state_ = State::Live;
    ever_seen_ = true;
    last_accept_time_ = time;
    rejected_.clear();
}

void HeadPositionEstimator::AddLight(double time, Vec3 light, bool tracking, Vec3 world_angular_velocity)
{
    if (!std::isfinite(light.x) || !std::isfinite(light.y) || !std::isfinite(light.z)) {
        return;
    }
    tracking_ = tracking;
    if (!tracking || state_ == State::Frozen) {
        return;
    }
    if (have_last_light_ && light.x == last_light_.x && light.y == last_light_.y && light.z == last_light_.z) {
        return;  // nothing new since the last poll
    }
    const double since_previous = have_last_light_ ? time - last_light_time_ : config_.camera_period_seconds;
    have_last_light_ = true;
    last_light_ = light;
    last_light_time_ = time;

    const double measured_at = time - config_.latency_seconds;
    const Quat orientation = OrientationAt(measured_at);
    const double eye_height = (light + Rotate(orientation, config_.eyes_from_light)).y;
    const double distance = camera_valid_ ? Length(light - camera_) : 0.0;
    if (eye_height < kMinEyeHeight || eye_height > kMaxEyeHeight ||
        (camera_valid_ && (distance < kMinCameraDistance || distance > kMaxCameraDistance))) {
        ++stats_.implausible;
        return;
    }
    const Vec3 pivot = light - Rotate(orientation, light_from_neck_);
    const Mat3 noise = MeasurementNoise(light, world_angular_velocity, since_previous);
    stats_.eye_height_m = eye_height;
    stats_.camera_distance_m = distance;

    if (state_ == State::NeverSeen) {
        const Vec3 before = Pivot(time, nullptr);
        Reset(time, measured_at, pivot, noise);
        StartBlend(time, before);
        blend_event_time_ = time;
        ++stats_.accepted;
        return;
    }

    Predict(measured_at);
    Mat3 s_inv;
    if (!Inverse(p_pp_ + noise, &s_inv)) {
        return;
    }
    const Vec3 innovation = pivot - pivot_;
    if (Dot(innovation, s_inv * innovation) > kGate) {
        ++stats_.rejected;
        rejected_.push_back({time, pivot});
        while (rejected_.front().time < time - kResetWindowSeconds) {
            rejected_.pop_front();
        }
        if (rejected_.size() < kResetCount) {
            return;
        }
        Vec3 mean;
        for (const Rejected& r : rejected_) {
            mean = mean + r.pivot * (1.0 / static_cast<double>(rejected_.size()));
        }
        for (const Rejected& r : rejected_) {
            if (Length(r.pivot - mean) > kResetSpreadM) {
                return;
            }
        }
        // Not noise: the head (or the PSMoveServiceEx frame) really moved.
        const Vec3 before = Pivot(time, nullptr);
        Reset(time, measured_at, mean, noise);
        StartBlend(time, before);
        blend_event_time_ = time;
        ++stats_.resets;
        return;
    }

    const Vec3 before = Pivot(time, nullptr);
    const bool reacquired = state_ == State::Holding;
    const Mat3 gain_p = p_pp_ * s_inv;
    const Mat3 gain_v = Transpose(p_pv_) * s_inv;
    pivot_ = pivot_ + gain_p * innovation;
    velocity_ = velocity_ + gain_v * innovation;
    const Mat3 pp = p_pp_ - gain_p * p_pp_;
    const Mat3 pv = p_pv_ - gain_p * p_pv_;
    const Mat3 vv = p_vv_ - gain_v * p_pv_;
    p_pp_ = Symmetric(pp);
    p_pv_ = pv;
    p_vv_ = Symmetric(vv);
    const double speed = Length(velocity_);
    if (speed > kMaxSpeed) {
        velocity_ = velocity_ * (kMaxSpeed / speed);
    }
    state_ = State::Live;
    last_accept_time_ = time;
    rejected_.clear();
    // The first updates after a jump still move the state a lot (zero velocity, wide covariance).
    if (reacquired || time - blend_event_time_ < kBlendSeconds) {
        StartBlend(time, before);
    }
    if (reacquired) {
        blend_event_time_ = time;
    }

    ++stats_.accepted;
    if (cameras_ == 1 && distance > 1e-6) {
        const Vec3 along = (light - camera_) * (1.0 / distance);
        const double depth = Dot(innovation, along);
        depth_innovation_sum_ += depth * depth;
        lateral_innovation_sum_ += Dot(innovation, innovation) - depth * depth;
    } else {
        lateral_innovation_sum_ += Dot(innovation, innovation);
    }
}

void HeadPositionEstimator::Tick(double now, bool source_ok)
{
    if (state_ == State::Live && (!source_ok || !tracking_ || now - last_accept_time_ > kLostAfterSeconds)) {
        EnterHolding(now);
    }
}

// Holds the neck pivot where the filter last had it; turning the head still moves the eyes around
// it. The extrapolation the output had glides away instead of being baked into the held state,
// where the next sample would read it as the head moving backwards.
void HeadPositionEstimator::EnterHolding(double now)
{
    const Vec3 before = Pivot(now, nullptr);
    const double held_at = now - config_.latency_seconds;
    Predict(std::min(held_at, state_time_ + kMaxExtrapolationSeconds));
    velocity_ = Vec3{};
    p_pv_ = Mat3{};
    p_vv_ = Mat3::Identity(kInitialVelocityVariance);
    state_time_ = held_at;
    state_ = State::Holding;
    rejected_.clear();
    StartBlend(now, before);
    ++stats_.losses;
}

void HeadPositionEstimator::SetFrozen(bool frozen, double now)
{
    if (frozen == (state_ == State::Frozen)) {
        return;
    }
    if (frozen) {
        pivot_ = Pivot(now, nullptr);
        velocity_ = Vec3{};
        state_ = State::Frozen;
        return;
    }
    state_ = ever_seen_ ? State::Holding : State::NeverSeen;
    p_pv_ = Mat3{};
    p_vv_ = Mat3::Identity(kInitialVelocityVariance);
    state_time_ = now - config_.latency_seconds;
    rejected_.clear();
}

void HeadPositionEstimator::OnYawChange(Quat delta, Quat before)
{
    for (TimedOrientation& entry : history_) {
        entry.orientation = Normalize(delta * entry.orientation);
    }
    if (ever_seen_) {
        pivot_ = pivot_ + Rotate(before, light_from_neck_) - Rotate(delta * before, light_from_neck_);
    }
    rejected_.clear();
}

Vec3 HeadPositionEstimator::Pivot(double now, Vec3* velocity) const
{
    Vec3 linear;
    Vec3 pivot = pivot_;
    if (state_ == State::NeverSeen) {
        pivot = config_.default_eyes - config_.eyes_from_neck;
    } else if (state_ == State::Live) {
        // Only as much of the velocity as stands out from its own uncertainty is extrapolated, so a
        // still head does not shake with the noise of the velocity estimate.
        Mat3 inverse;
        double gain = 0.0;
        if (Inverse(p_vv_, &inverse)) {
            const double significance = Dot(velocity_, inverse * velocity_);
            gain = config_.prediction_gain * significance / (significance + kVelocityConfidence);
        }
        const double ahead = std::clamp(now + horizon_ - state_time_, 0.0, kMaxExtrapolationSeconds);
        Vec3 extrapolation = velocity_ * (gain * ahead);
        const double length = Length(extrapolation);
        if (length > kMaxExtrapolationM) {
            extrapolation = extrapolation * (kMaxExtrapolationM / length);
        } else {
            linear = velocity_ * gain;
        }
        pivot = pivot + extrapolation;
    }
    if (velocity) {
        *velocity = linear;
    }
    return pivot;
}

Vec3 HeadPositionEstimator::BlendOffset(double now) const
{
    return blend_ * std::exp(-std::max(now - blend_time_, 0.0) / kBlendSeconds);
}

void HeadPositionEstimator::StartBlend(double now, Vec3 pivot_before)
{
    blend_ = BlendOffset(now) + pivot_before - Pivot(now, nullptr);
    blend_time_ = now;
}

HeadPoseOut HeadPositionEstimator::Output(double now, Quat orientation, Vec3 world_angular_velocity) const
{
    Vec3 linear;
    const Vec3 pivot = Pivot(now, &linear);
    const Vec3 neck_to_eyes = Rotate(orientation, config_.eyes_from_neck);
    return {pivot + neck_to_eyes + BlendOffset(now), linear + Cross(world_angular_velocity, neck_to_eyes)};
}

HeadPositionEstimator::Stats HeadPositionEstimator::TakeStats()
{
    Stats stats = stats_;
    if (stats.accepted > 0) {
        stats.depth_innovation_rms_m = std::sqrt(depth_innovation_sum_ / stats.accepted);
        stats.lateral_innovation_rms_m = std::sqrt(lateral_innovation_sum_ / stats.accepted);
    }
    stats_ = Stats{};
    stats_.eye_height_m = stats.eye_height_m;
    stats_.camera_distance_m = stats.camera_distance_m;
    depth_innovation_sum_ = 0.0;
    lateral_innovation_sum_ = 0.0;
    return stats;
}
