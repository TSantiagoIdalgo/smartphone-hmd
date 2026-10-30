#include "move_yaw.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kGravity = 9.81;
constexpr double kHalfTurn = 3.14159265358979;
constexpr double kHistorySeconds = 0.8;
constexpr double kChangeSeconds = 0.2;   // the velocity change compared, over this long
constexpr double kFitSeconds = 0.05;     // each optical velocity, from the positions this close
constexpr int kMinFitSamples = 4;
constexpr double kCompareSeconds = 0.05;
constexpr double kMinChange = 0.6;       // m/s, both: a brisk moment
constexpr double kMaxMismatch = 2.0;     // one change at most this many times the other
constexpr double kEvidenceSeconds = 5.0;  // older moments count less and less
constexpr double kMinEvidence = 5.0;     // (m/s)^2: a few brisk moments before correcting
constexpr double kMaxTurnRate = 20.0 * kHalfTurn / 180.0;  // rad/s, the correction at most
constexpr int kGravityWarmup = 200;      // accelerometer samples averaged for its sense and units
constexpr double kGravityRate = 0.002;
constexpr double kMaxSensorStep = 0.05;  // s; longer between samples, not summed
// From the accelerometer to the bulb's centre, in the Move's axes (-z towards the bulb).
const Vec3 kAccelToBulb{0.0, 0.0, -0.1};

}  // namespace

void MoveYaw::Reset()
{
    cross_ = dot_ = 0.0;
    correction_ = 0.0;
}

void MoveYaw::Add(const MoveState& move, double now)
{
    // A new sample from the Move's sensors: sum the acceleration across the floor.
    const bool fresh = !have_sensor_ || move.sensor_time != last_sensor_time_ || move.accel.x != last_accel_.x ||
                       move.accel.y != last_accel_.y || move.accel.z != last_accel_.z;
    if (fresh && Length(move.accel) > 0.0) {
        double dt = move.sensor_time - last_sensor_time_;
        if (!(dt > 0.0 && dt < kMaxSensorStep)) {
            dt = std::clamp(now - last_imu_now_, 0.0, kMaxSensorStep);
        }
        if (!have_sensor_) {
            dt = 0.0;
        }
        have_sensor_ = true;
        last_sensor_time_ = move.sensor_time;
        last_accel_ = move.accel;
        last_imu_now_ = now;
        stats_.peak_spin_degrees = std::max(stats_.peak_spin_degrees, Length(move.gyro) * 180.0 / kHalfTurn);
        const Vec3 force = Rotate(move.orientation, move.accel);
        const double rate = gravity_samples_ < kGravityWarmup ? 1.0 / (gravity_samples_ + 1) : kGravityRate;
        gravity_ += (force.y - gravity_) * rate;
        ++gravity_samples_;
        if (gravity_samples_ >= kGravityWarmup && std::fabs(gravity_) > 0.0) {
            // Across the floor the accelerometer reads the acceleration alone, in its units.
            const double scale = kGravity / gravity_;
            vx_ += force.x * scale * dt;
            vz_ += force.z * scale * dt;
            const Vec3 spin = Cross(Rotate(move.orientation, move.gyro), Rotate(move.orientation, kAccelToBulb));
            inertial_.push_back({now, vx_ + spin.x, vz_ + spin.z});
        }
    }
    // A new position of the bulb from the PS Eye.
    if (move.tracking && (!have_position_ || move.position.x != last_position_.x ||
                          move.position.y != last_position_.y || move.position.z != last_position_.z)) {
        have_position_ = true;
        last_position_ = move.position;
        optical_.push_back({now - optical_latency_, move.position.x, move.position.z});
    }
    while (!optical_.empty() && optical_.front().time < now - kHistorySeconds) {
        optical_.pop_front();
    }
    while (!inertial_.empty() && inertial_.front().time < now - kHistorySeconds) {
        inertial_.pop_front();
    }
    if (now >= next_compare_) {
        next_compare_ = now + kCompareSeconds;
        Compare(now);
    }
}

bool MoveYaw::OpticalVelocity(double time, double* x, double* z) const
{
    int n = 0, before = 0, after = 0;
    double st = 0.0, sx = 0.0, sz = 0.0, stt = 0.0, stx = 0.0, stz = 0.0;
    for (const Sample& s : optical_) {
        const double t = s.time - time;
        if (std::fabs(t) > kFitSeconds) {
            continue;
        }
        ++n;
        before += t < 0.0;
        after += t > 0.0;
        st += t;
        sx += s.x;
        sz += s.z;
        stt += t * t;
        stx += t * s.x;
        stz += t * s.z;
    }
    const double spread = stt - st * st / std::max(n, 1);
    if (n < kMinFitSamples || before == 0 || after == 0 || spread <= 0.0) {
        return false;
    }
    *x = (stx - st * sx / n) / spread;
    *z = (stz - st * sz / n) / spread;
    return true;
}

bool MoveYaw::InertialVelocity(double time, double* x, double* z) const
{
    for (size_t i = 1; i < inertial_.size(); ++i) {
        const Sample& a = inertial_[i - 1];
        const Sample& b = inertial_[i];
        if (a.time <= time && b.time >= time) {
            const double f = b.time > a.time ? (time - a.time) / (b.time - a.time) : 0.0;
            *x = a.x + (b.x - a.x) * f;
            *z = a.z + (b.z - a.z) * f;
            return true;
        }
    }
    return false;
}

// How the bulb's velocity changed over the last moment the PS Eye has seen whole, by each: the
// turn from the accelerometer's to the PS Eye's, weighted by how brisk the moment was.
void MoveYaw::Compare(double now)
{
    const double fade = std::exp(-std::max(now - last_fade_, 0.0) / kEvidenceSeconds);
    last_fade_ = now;
    cross_ *= fade;
    dot_ *= fade;

    const double end = now - optical_latency_ - kFitSeconds - 0.01, start = end - kChangeSeconds;
    double ox0, oz0, ox1, oz1, ix0, iz0, ix1, iz1;
    if (OpticalVelocity(start, &ox0, &oz0) && OpticalVelocity(end, &ox1, &oz1) &&
        InertialVelocity(start, &ix0, &iz0) && InertialVelocity(end, &ix1, &iz1)) {
        const double ax = ix1 - ix0, az = iz1 - iz0;  // the accelerometer's, in PSMoveServiceEx's turn
        const double bx = ox1 - ox0, bz = oz1 - oz0;  // the PS Eye's
        const double a = std::hypot(ax, az), b = std::hypot(bx, bz);
        if (a > kMinChange && b > kMinChange && a < kMaxMismatch * b && b < kMaxMismatch * a) {
            // About +y, from a to b.
            cross_ += az * bx - ax * bz;
            dot_ += ax * bx + az * bz;
            ++stats_.moments;
        }
    }

    const double step = std::clamp(now - last_step_, 0.0, 0.1);
    last_step_ = now;
    if (std::hypot(cross_, dot_) > kMinEvidence) {
        const double target = std::atan2(cross_, dot_);
        const double turn = std::remainder(target - correction_, 2.0 * kHalfTurn);
        correction_ += std::clamp(turn, -kMaxTurnRate * step, kMaxTurnRate * step);
    }
}

MoveYaw::Stats MoveYaw::TakeStats()
{
    Stats stats = stats_;
    stats.correction_degrees = correction_ * 180.0 / kHalfTurn;
    stats_ = Stats{};
    return stats;
}
