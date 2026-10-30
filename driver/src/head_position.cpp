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
constexpr double kImpossibleJumpM = 0.5;    // from where it should be, between two camera frames
constexpr double kLostAfterSeconds = 0.15;
constexpr double kMaxExtrapolationSeconds = 0.10;
constexpr double kMaxExtrapolationM = 0.08;
constexpr double kVelocityConfidence = 3.0;  // squared Mahalanobis length of the velocity at half prediction
constexpr double kInitialVelocityVariance = 0.25;
constexpr double kBlendSeconds = 0.15;
constexpr double kMinCameraDistance = 0.3;
constexpr double kMaxCameraDistance = 4.0;
constexpr double kReferenceDistance = 2.0;  // the noise settings are given at this distance
// The body cameras' views arrive about 0.1 s after the light of the same moment; this much of
// the light is kept to put them in their place.
constexpr double kReplaySeconds = 0.35;
constexpr double kSightingsLostSeconds = 0.3;  // without the light, the body cameras alone keep it live
constexpr double kRayVariance = 1.0;           // m^2: a view says nothing along its line of sight
constexpr double kStartPairSeconds = 0.05;     // two cameras' views this close start the head
constexpr double kStartMaxGapM = 0.10;         // if their rays pass this close
constexpr double kStartVariance = 0.03 * 0.03;
constexpr double kMaxLightGapSeconds = 0.05;   // to compare a view with the light between two samples
// Two views that meet this close to the light's line of sight (at whatever depth the light is) see
// what the PS Eye sees; a bulb's, without the ears' guess in between, this close.
constexpr double kLineGapM = 0.12;
constexpr double kBulbLineGapM = 0.05;
constexpr double kPairDepthNoiseM = 0.03;
constexpr double kLightScaleRate = 0.01;        // per pair of views: a few seconds
constexpr double kLightScaleWarmupViews = 10.0;
constexpr double kMinLightScale = 0.2;
constexpr double kMaxLightScale = 5.0;
constexpr double kEarsLearningRate = 0.005;    // per view: some seconds
constexpr double kEarsWarmupViews = 20.0;      // the first views count for more: an average of them
constexpr double kMaxEarsCorrectionM = 0.12;
constexpr double kMinRaysSine = 0.25;           // a view this close to the PS Eye's line of sight teaches nothing
// The agility scale follows the normalized innovations (1 per dimension on average when the model
// fits), as the body filter's does: it shrinks while the head keeps still and grows quickly when
// it starts moving.
constexpr double kMinAgility = 0.01;
constexpr double kMaxAgility = 10.0;
constexpr double kAgilityRate = 0.3;
constexpr double kDepthTrendRate = 0.25;  // per camera frame: about the last four
// The variance of that average when the innovations are noise of unit variance.
constexpr double kDepthTrendVariance = kDepthTrendRate / (2.0 - kDepthTrendRate);

Mat3 Symmetric(const Mat3& a)
{
    return (a + Transpose(a)) * 0.5;
}

}  // namespace

HeadPositionEstimator::HeadPositionEstimator(const HeadPositionConfig& config)
    : config_(config),
      light_from_neck_(config.eyes_from_neck - config.eyes_from_light),
      ears_from_neck_(config.ears_from_neck)
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
Mat3 HeadPositionEstimator::MeasurementNoise(Vec3 light, Vec3 world_angular_velocity, double since_previous,
                                             bool sightings) const
{
    Mat3 noise;
    const double distance = camera_valid_ ? Length(light - camera_) : 0.0;
    if (cameras_ == 1 && distance > 1e-6) {
        const Vec3 along = (light - camera_) * (1.0 / distance);
        const double ratio = distance / kReferenceDistance;
        const double lateral = config_.lateral_noise_m * ratio;
        const double depth = (sightings ? config_.depth_noise_with_sightings_m : config_.depth_noise_m) * ratio * ratio;
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
    const double q = config_.agility * agility_;
    const Mat3 pp = p_pp_ + (p_pv_ + Transpose(p_pv_)) * dt + p_vv_ * (dt * dt) + Mat3::Identity(q * dt * dt * dt / 3.0);
    const Mat3 pv = p_pv_ + p_vv_ * dt + Mat3::Identity(q * dt * dt / 2.0);
    const Mat3 vv = p_vv_ + Mat3::Identity(q * dt);
    p_pp_ = pp;
    p_pv_ = pv;
    p_vv_ = vv;
    pivot_ = pivot_ + velocity_ * dt;
    state_time_ = to;
}

void HeadPositionEstimator::Reset(double measured_at, Vec3 pivot, const Mat3& noise)
{
    pivot_ = pivot;
    velocity_ = Vec3{};
    p_pp_ = noise;
    p_pv_ = Mat3{};
    p_vv_ = Mat3::Identity(kInitialVelocityVariance);
    state_time_ = measured_at;
    agility_ = 1.0;
    depth_trend_ = 0.0;
    state_ = State::Live;
    ever_seen_ = true;
    rejected_.clear();
    rejected_sightings_.clear();
    ForgetSteps();
}

HeadPositionEstimator::Snapshot HeadPositionEstimator::Save() const
{
    return {pivot_, velocity_, p_pp_, p_pv_, p_vv_, state_time_, agility_, depth_trend_};
}

void HeadPositionEstimator::Restore(const Snapshot& snapshot)
{
    pivot_ = snapshot.pivot;
    velocity_ = snapshot.velocity;
    p_pp_ = snapshot.p_pp;
    p_pv_ = snapshot.p_pv;
    p_vv_ = snapshot.p_vv;
    state_time_ = snapshot.time;
    agility_ = snapshot.agility;
    depth_trend_ = snapshot.depth_trend;
}

void HeadPositionEstimator::ForgetSteps()
{
    steps_.clear();
    base_ = Save();
}

bool HeadPositionEstimator::Rewind(double time, size_t* index)
{
    if (time >= state_time_) {
        *index = steps_.size();
        Predict(time);
        return true;
    }
    size_t i = steps_.size();
    while (i > 0 && steps_[i - 1].time > time) {
        --i;
    }
    if (i == 0 && time < base_.time) {
        return false;
    }
    Restore(i == 0 ? base_ : steps_[i - 1].after);
    Predict(time);
    *index = i;
    return true;
}

double HeadPositionEstimator::Correct(const Measurement& m, bool apply)
{
    Mat3 s_inv;
    if (!Inverse(p_pp_ + m.noise, &s_inv)) {
        return -1.0;
    }
    const Vec3 innovation = m.pivot - pivot_;
    const Vec3 weighted = s_inv * innovation;
    const double distance = Dot(innovation, weighted);
    if (!apply) {
        return distance;
    }
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
    if (speed > config_.max_speed) {
        velocity_ = velocity_ * (config_.max_speed / speed);
    }
    // Its poor direction (the PS Eye's depth) would say more about its own noise than about how
    // the head moves: the rest counts sample by sample, the distance as if that direction were not
    // measured. For the light, that direction counts by its recent average instead, so going
    // towards or away from the PS Eye is still followed.
    double counted = distance, dimensions = 3.0;
    const double blind_weight = Dot(m.blind, s_inv * m.blind);
    if (blind_weight > 0.0) {
        const double along = Dot(m.blind, weighted);
        counted = std::max(distance - along * along / blind_weight, 0.0);
        dimensions = 2.0;
        if (m.light) {
            const double rate = kDepthTrendRate * m.weight;
            depth_trend_ = (1.0 - rate) * depth_trend_ + rate * along / std::sqrt(blind_weight);
            counted += depth_trend_ * depth_trend_ / kDepthTrendVariance;
            dimensions = 3.0;
        }
    }
    agility_ = std::clamp(agility_ * std::exp(kAgilityRate * m.weight * std::clamp(counted / dimensions - 1.0, -1.0, 4.0)),
                          kMinAgility, kMaxAgility);
    return distance;
}

// Applies a measurement with the filter at its time; one left out only makes the head look agile,
// so the next ones get in if it really moved.
void HeadPositionEstimator::Apply(const Measurement& m)
{
    if (m.missed) {
        agility_ = std::min(agility_ * std::exp(4.0 * kAgilityRate * m.weight), kMaxAgility);
    } else {
        Correct(m, true);
    }
}

void HeadPositionEstimator::Keep(size_t index, double time, const Measurement& m)
{
    steps_.insert(steps_.begin() + static_cast<std::ptrdiff_t>(index), Step{time, m, Save()});
    for (size_t i = index + 1; i < steps_.size(); ++i) {
        Predict(steps_[i].time);
        Apply(steps_[i].measurement);
        steps_[i].after = Save();
    }
    while (steps_.front().time < steps_.back().time - kReplaySeconds) {
        base_ = steps_.front().after;
        steps_.pop_front();
    }
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
    if (config_.learn_light_scale && camera_valid_ && cameras_ == 1) {
        light = camera_ + (light - camera_) * light_scale_;
    }

    const double measured_at = time - config_.latency_seconds;
    const Quat orientation = OrientationAt(measured_at);
    const double eye_height = (light + Rotate(orientation, config_.eyes_from_light)).y;
    const double distance = camera_valid_ ? Length(light - camera_) : 0.0;
    // While the body cameras see the head, the light gives its direction from the PS Eye and they
    // its depth: a depth that puts the eyes where none are does not make the direction wrong. As
    // soon as their views arrive, before any fits: a far-off depth would otherwise keep them out.
    // It is let go only while their views fit, or meet two by two on the light's line of sight,
    // though: views that miss it (cameras that moved since their calibration) fix nothing, and the
    // depth would wander off.
    const bool views = time - last_view_time_ <= kSightingsLostSeconds;
    const bool sightings = time - std::max(last_sighting_time_, last_line_view_time_) <= kSightingsLostSeconds;
    if (!views && (eye_height < config_.min_eye_height || eye_height > config_.max_eye_height ||
                   (camera_valid_ && (distance < kMinCameraDistance || distance > kMaxCameraDistance)))) {
        ++stats_.implausible;
        return;
    }
    Measurement m;
    m.pivot = light - Rotate(orientation, light_from_neck_);
    m.noise = MeasurementNoise(light, world_angular_velocity, since_previous, sightings);
    if (cameras_ == 1 && distance > 1e-6) {
        m.blind = (light - camera_) * (1.0 / distance);
    }
    // PSMoveServiceEx's values between camera frames count for less, as in MeasurementNoise.
    m.weight = std::clamp(since_previous / config_.camera_period_seconds, 0.1, 1.0);
    stats_.eye_height_m = eye_height;
    stats_.camera_distance_m = distance;

    const Vec3 before = Pivot(time, nullptr);
    if (state_ == State::NeverSeen) {
        Reset(measured_at, m.pivot, m.noise);
        Keep(0, measured_at, m);
        last_accept_time_ = time;
        StartBlend(time, before);
        blend_event_time_ = time;
        ++stats_.accepted;
        return;
    }

    const Snapshot current = Save();
    size_t index = 0;
    if (!Rewind(measured_at, &index)) {
        Restore(current);
        return;
    }
    const Vec3 innovation = m.pivot - pivot_;
    const double mahalanobis = Correct(m, false);
    if (mahalanobis < 0.0) {
        Restore(current);
        return;
    }
    if (mahalanobis > kGate) {
        Measurement missed = m;
        missed.missed = true;
        // For a bulb the PS Eye follows or the phones see, a jump nobody makes between two frames
        // (a ball half hidden, put far away) does not make the filter readier to follow; if it
        // lasts, the reset below takes it in.
        if (config_.sightings_only_without_light && (sightings || LightFollowed(time)) &&
            Length(innovation) > kImpossibleJumpM) {
            missed.weight = 0.0;
        }
        Apply(missed);
        Keep(index, measured_at, missed);
        ++stats_.rejected;
        rejected_.push_back({time, m.pivot});
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
        // While the body cameras' views fit they hold it: a jump of the light alone (a ball half
        // hidden, put far away) is not believed. A bulb's light still is across its line of sight,
        // which the PS Eye sees well: there the views were off, not the light.
        const Vec3 off = mean - pivot_;
        const Vec3 across = off - m.blind * Dot(off, m.blind);
        if (sightings && (!config_.sightings_only_without_light || Length(across) <= kResetSpreadM)) {
            return;
        }
        // Not noise: the head (or the PSMoveServiceEx frame) really moved.
        m.pivot = mean;
        Reset(measured_at, mean, m.noise);
        Keep(0, measured_at, m);
        last_accept_time_ = time;
        StartBlend(time, before);
        blend_event_time_ = time;
        ++stats_.resets;
        return;
    }

    // Found again, after holding or after the body cameras alone.
    const bool reacquired = state_ == State::Holding || time - last_accept_time_ > kLostAfterSeconds;
    Apply(m);
    Keep(index, measured_at, m);
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
    agility_sum_ += agility_;
    if (cameras_ == 1 && distance > 1e-6) {
        const double depth = Dot(innovation, m.blind);
        depth_innovation_sum_ += depth * depth;
        lateral_innovation_sum_ += Dot(innovation, innovation) - depth * depth;
    } else {
        lateral_innovation_sum_ += Dot(innovation, innovation);
    }
}

bool HeadPositionEstimator::LightFollowed(double now) const
{
    return state_ == State::Live && tracking_ && now - last_accept_time_ <= kLostAfterSeconds;
}

void HeadPositionEstimator::Tick(double now, bool source_ok)
{
    const bool light = source_ok && LightFollowed(now);
    if (state_ == State::Live && !light && now - last_sighting_time_ > kSightingsLostSeconds) {
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
    ForgetSteps();
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
    rejected_sightings_.clear();
    ForgetSteps();
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
    rejected_sightings_.clear();
    ForgetSteps();
}

// A view as a measurement of the neck pivot: anywhere along the camera's line of sight through
// the ears, as precise across it as the pose points are. The filter must be at the view's time.
void HeadPositionEstimator::AddSighting(double now, const HeadSighting& sighting)
{
    if (!std::isfinite(sighting.u) || !std::isfinite(sighting.v) || !(sighting.model.focal > 0.0) ||
        !(sighting.noise > 0.0)) {
        return;
    }
    last_view_time_ = now;
    bool replaced = false;
    for (HeadSighting& r : recent_) {
        if (r.camera == sighting.camera) {
            r = sighting;
            replaced = true;
        }
    }
    if (!replaced) {
        recent_.push_back(sighting);
    }
    if (state_ == State::Frozen) {
        return;
    }
    if (state_ == State::NeverSeen) {
        StartFromSightings(now);
        return;
    }

    const Quat orientation = OrientationAt(sighting.time);
    const Vec3 ears_offset = Rotate(orientation, ears_from_neck_);
    double line_gap = 0.0, depth_ratio = 1.0;
    Vec3 pair_pivot;
    const bool paired = LineGap(sighting, orientation, &line_gap, &depth_ratio, &pair_pivot);
    const bool on_line = paired && line_gap <= (config_.sightings_only_without_light ? kBulbLineGapM : kLineGapM);
    if (paired) {
        ++stats_.paired_views;
        stats_.on_line_views += on_line;
        line_gap_sum_ += line_gap;
    }
    if (on_line && config_.learn_light_scale && depth_ratio > 0.0) {
        const double rate = std::max(kLightScaleRate, 1.0 / (light_scale_views_ + kLightScaleWarmupViews));
        ++light_scale_views_;
        light_scale_ = std::clamp(light_scale_ * std::pow(depth_ratio, rate), kMinLightScale, kMaxLightScale);
    }
    if (on_line) {
        last_line_view_time_ = now;
    }
    const Vec3 before = Pivot(now, nullptr);
    const Snapshot current = Save();
    size_t index = 0;
    if (!Rewind(sighting.time, &index)) {
        Restore(current);
        ++stats_.sightings_late;
        return;
    }
    const Vec3 ray = sighting.model.Ray(sighting.u, sighting.v);
    const Vec3 to_ears = pivot_ + ears_offset - sighting.model.position;
    const double depth = Dot(to_ears, ray);
    if (depth < kMinCameraDistance) {
        Restore(current);
        return;
    }
    Measurement m;
    m.light = false;
    m.pivot = sighting.model.position + ray * depth - ears_offset;
    const double across = sighting.noise * depth / sighting.model.focal;
    m.noise = Mat3::Identity(across * across) + Outer(ray, ray) * (kRayVariance - across * across);
    m.blind = ray;
    const double mahalanobis = Correct(m, false);
    if (mahalanobis < 0.0) {
        Restore(current);
        return;
    }
    // A bulb the PS Eye follows stays where the PS Eye puts it: a view that fits only tells that it
    // is still there, which keeps a jump of the light from being believed and the views ready for
    // when the PS Eye loses it. The head's views go in two by two instead.
    if ((config_.sightings_only_without_light || config_.sighting_pairs) && LightFollowed(now)) {
        Restore(current);
        if (config_.sighting_pairs) {
            if (on_line) {
                LearnEars(sighting, orientation);
                AddPair(now, sighting.time, pair_pivot);
            } else {
                ++stats_.sightings_rejected;
            }
            return;
        }
        if (mahalanobis <= kGate) {
            last_sighting_time_ = now;
        }
        // Whether the phones see it where the PS Eye does, its noisy depth aside: their calibration.
        ++((paired ? on_line : mahalanobis <= kGate) ? stats_.sightings : stats_.sightings_rejected);
        return;
    }
    if (mahalanobis > kGate) {
        m.missed = true;
        Apply(m);
        Keep(index, sighting.time, m);
        ++stats_.sightings_rejected;
        rejected_sightings_.push_back(now);
        while (rejected_sightings_.front() < now - kResetWindowSeconds) {
            rejected_sightings_.pop_front();
        }
        // Without the light the head may have gone where the filter no longer looks; with it, the
        // light's depth may be off from where the cameras see the head.
        if (rejected_sightings_.size() >= kResetCount && StartFromSightings(now)) {
            rejected_sightings_.clear();
        }
        return;
    }
    LearnEars(sighting, orientation);
    const bool reacquired = state_ == State::Holding;
    Apply(m);
    Keep(index, sighting.time, m);
    agility_sum_ += agility_;
    state_ = State::Live;
    last_sighting_time_ = now;
    rejected_sightings_.clear();
    // The views are coarse and correct a moment already past: each one glides in rather than
    // jumping, where the light's corrections are fresh and fine.
    StartBlend(now, before);
    if (reacquired) {
        blend_event_time_ = now;
    }
    ++stats_.sightings;
}

// Where two cameras meet on the light's line of sight: how far along it the neck pivot is, at the
// time of the newer view.
void HeadPositionEstimator::AddPair(double now, double time, Vec3 pivot)
{
    const Vec3 before = Pivot(now, nullptr);
    const Snapshot current = Save();
    size_t index = 0;
    if (!Rewind(time, &index)) {
        Restore(current);
        ++stats_.sightings_late;
        return;
    }
    Measurement m;
    m.light = false;
    m.pivot = pivot;
    const Vec3 along = Unit(pivot - camera_);
    m.noise = Mat3::Identity(kRayVariance) + Outer(along, along) * (kPairDepthNoiseM * kPairDepthNoiseM - kRayVariance);
    const double mahalanobis = Correct(m, false);
    if (mahalanobis < 0.0 || mahalanobis > kGate) {
        Restore(current);
        ++stats_.sightings_rejected;
        return;
    }
    Apply(m);
    Keep(index, time, m);
    agility_sum_ += agility_;
    last_sighting_time_ = now;
    StartBlend(now, before);
    ++stats_.sightings;
}

bool HeadPositionEstimator::StartFromSightings(double now)
{
    const HeadSighting* newest = nullptr;
    for (const HeadSighting& r : recent_) {
        if (!newest || r.time > newest->time) {
            newest = &r;
        }
    }
    if (!newest) {
        return false;
    }
    for (const HeadSighting& other : recent_) {
        if (other.camera == newest->camera || newest->time - other.time > kStartPairSeconds) {
            continue;
        }
        Vec3 ears;
        if (!Triangulate({{&newest->model, newest->u, newest->v}, {&other.model, other.u, other.v}}, &ears)) {
            continue;
        }
        double gap = 0.0;
        for (const HeadSighting* s : {newest, &other}) {
            const Vec3 ray = s->model.Ray(s->u, s->v);
            const Vec3 to = ears - s->model.position;
            gap += Dot(to, ray) > kMinCameraDistance ? Length(to - ray * Dot(to, ray)) : 1e9;
        }
        const Quat orientation = OrientationAt(newest->time);
        const Vec3 pivot = ears - Rotate(orientation, ears_from_neck_);
        const double eye_height = (pivot + Rotate(orientation, config_.eyes_from_neck)).y;
        if (gap > kStartMaxGapM || eye_height < config_.min_eye_height || eye_height > config_.max_eye_height) {
            continue;
        }
        // With the light followed, only its depth may be given up: the head must be on its line
        // of sight.
        const bool light = LightFollowed(now);
        if (light) {
            Vec3 light_pivot;
            if (!camera_valid_ || !LightAt(newest->time, &light_pivot)) {
                continue;
            }
            const Vec3 along = Unit(light_pivot + Rotate(orientation, light_from_neck_) - camera_);
            const Vec3 to = pivot + Rotate(orientation, light_from_neck_) - camera_;
            if (Length(to - along * Dot(to, along)) > kStartMaxGapM) {
                continue;
            }
        }
        const Vec3 before = Pivot(now, nullptr);
        Reset(newest->time, pivot, Mat3::Identity(kStartVariance));
        last_sighting_time_ = now;
        StartBlend(now, before);
        blend_event_time_ = now;
        ++(light ? stats_.light_depth_resets : stats_.sighting_starts);
        return true;
    }
    return false;
}

// Where this view and another camera's at about the same time meet, how far that is from the
// light's line of sight (the ears' place on it, at any depth): PSMoveServiceEx's depth of the light
// can be far off, its direction cannot, and neither can two cameras that agree on a calibration.
bool HeadPositionEstimator::LineGap(const HeadSighting& sighting, Quat orientation, double* gap,
                                    double* depth_ratio, Vec3* on_line) const
{
    Vec3 light_pivot;
    if (!camera_valid_ || cameras_ != 1 || !LightAt(sighting.time, &light_pivot)) {
        return false;
    }
    for (const HeadSighting& other : recent_) {
        Vec3 point;
        if (other.camera == sighting.camera || std::fabs(other.time - sighting.time) > kStartPairSeconds ||
            !Triangulate({{&sighting.model, sighting.u, sighting.v}, {&other.model, other.u, other.v}}, &point)) {
            continue;
        }
        const Vec3 light = light_pivot + Rotate(orientation, light_from_neck_);
        const Vec3 along = Unit(light - camera_);
        const Vec3 to = point - Rotate(orientation, ears_from_neck_) + Rotate(orientation, light_from_neck_) - camera_;
        *gap = Length(to - along * Dot(to, along));
        *depth_ratio = Dot(to, along) / std::max(Length(light - camera_), kMinCameraDistance);
        if (on_line) {
            *on_line = camera_ + along * Dot(to, along) - Rotate(orientation, light_from_neck_);
        }
        return true;
    }
    return false;
}

bool HeadPositionEstimator::LightAt(double time, Vec3* pivot) const
{
    const Step* before = nullptr;
    for (const Step& step : steps_) {
        if (!step.measurement.light || step.measurement.missed) {
            continue;
        }
        if (step.time <= time) {
            before = &step;
        } else if (before && step.time - before->time <= kMaxLightGapSeconds) {
            const Vec3 a = before->measurement.pivot, b = step.measurement.pivot;
            *pivot = a + (b - a) * ((time - before->time) / (step.time - before->time));
            return true;
        } else {
            return false;
        }
    }
    return false;
}

// Where the ears sit varies with the person and with where the pose model puts them: while the
// light is followed, the guess moves a little with each view towards where the light says they
// are, across that view's line of sight.
void HeadPositionEstimator::LearnEars(const HeadSighting& sighting, Quat orientation)
{
    Vec3 light_pivot;
    if (!LightAt(sighting.time, &light_pivot)) {
        return;
    }
    const Vec3 ray = sighting.model.Ray(sighting.u, sighting.v);
    const Vec3 to_ears = light_pivot + Rotate(orientation, ears_from_neck_) - sighting.model.position;
    Vec3 across = ray * Dot(to_ears, ray) - to_ears;
    // Only where the light is sure: its depth, along the PS Eye's line of sight, may be off. What
    // neither line of sight can tell is the one direction across both.
    if (camera_valid_) {
        const Vec3 light = light_pivot + Rotate(orientation, light_from_neck_);
        const Vec3 both = Cross(ray, Unit(light - camera_));
        if (Length(both) < kMinRaysSine) {
            return;
        }
        const Vec3 unseen = Unit(both);
        across = unseen * Dot(across, unseen);
    }
    const double rate = std::max(kEarsLearningRate, 1.0 / (ears_views_ + kEarsWarmupViews));
    ++ears_views_;
    Vec3 learned = ears_from_neck_ + Rotate(Conjugate(orientation), across) * rate;
    const Vec3 correction = learned - config_.ears_from_neck;
    if (Length(correction) > kMaxEarsCorrectionM) {
        learned = config_.ears_from_neck + correction * (kMaxEarsCorrectionM / Length(correction));
    }
    ears_from_neck_ = learned;
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

HeadPoseOut NeckModel(Vec3 eyes, Vec3 eyes_from_neck, Quat orientation, Vec3 world_angular_velocity)
{
    const Vec3 neck_to_eyes = Rotate(orientation, eyes_from_neck);
    return {eyes - eyes_from_neck + neck_to_eyes, Cross(world_angular_velocity, neck_to_eyes)};
}

HeadPositionEstimator::Stats HeadPositionEstimator::TakeStats()
{
    Stats stats = stats_;
    if (stats.accepted > 0) {
        stats.depth_innovation_rms_m = std::sqrt(depth_innovation_sum_ / stats.accepted);
        stats.lateral_innovation_rms_m = std::sqrt(lateral_innovation_sum_ / stats.accepted);
    }
    if (stats.accepted + stats.sightings > 0) {
        stats.agility = agility_sum_ / (stats.accepted + stats.sightings);
    }
    stats.ears_from_neck = ears_from_neck_;
    if (stats.paired_views > 0) {
        stats.line_gap_m = line_gap_sum_ / stats.paired_views;
    }
    stats.light_scale = light_scale_;
    line_gap_sum_ = 0.0;
    stats_ = Stats{};
    stats_.eye_height_m = stats.eye_height_m;
    stats_.camera_distance_m = stats.camera_distance_m;
    depth_innovation_sum_ = 0.0;
    lateral_innovation_sum_ = 0.0;
    agility_sum_ = 0.0;
    return stats;
}
