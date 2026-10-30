#include "head_tracker.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kGravity = 9.81;
constexpr double kMaxGyro = 35.0;          // rad/s; above this the sample is junk
constexpr double kMaxAccel = 16.0 * kGravity;
// Samples this far apart (some lost on the way) are joined from both rates; farther (the app
// paused), not.
constexpr double kMaxStepSeconds = 0.5;
constexpr double kGapSeconds = 0.1;        // farther apart than this, counted for the log
constexpr double kTiltGain = 1.0;          // how fast tilt follows gravity (1/s)
constexpr double kAccelTrustBand = 0.15 * kGravity;  // only trust gravity when |a| is close to g
// The gyroscope's bias is learned second by second while the phone keeps still. Lying flat on a
// table, it reads only its bias and noise: a few seconds learn it. Worn, the head is never quite
// still, and a slow turn must not pass for bias or the view would lose it: such seconds must read
// close to the bias already learned, and move it only a little.
constexpr double kBiasWindowSeconds = 1.0;
constexpr double kStillAccelRange = 0.3;   // m/s^2, of |a| over the second
constexpr double kFlatGravity = 0.9;       // share of gravity across the screen: lying flat
constexpr double kTableSpread = 0.01;      // rad/s, of the gyroscope over the second
constexpr double kTableBiasSeconds = 2.0;
constexpr double kWornSpread = 0.03;
constexpr double kWornMean = 0.01;         // rad/s, from the bias learned
constexpr double kWornBiasSeconds = 300.0;
constexpr double kMaxBias = 0.05;          // rad/s
constexpr double kMagMinNorm = 15.0;       // uT; below this the sensor is not reporting a field
constexpr double kMagMaxNorm = 120.0;
constexpr double kMagNormBand = 0.12;      // accepted samples stay within this of the running average
constexpr double kMagDipBand = 0.12;       // and so does the inclination
constexpr double kMagMinHorizontal = 0.2;  // fraction of the field that must be horizontal
constexpr double kMagAverageRate = 0.01;   // per accepted sample
constexpr int kMagSeedSamples = 100;
constexpr double kMagMaxRate = 0.035;      // rad/s, the most the correction ever turns the view
constexpr int32_t kRestartSeqJump = -1000; // a big jump back means the app restarted

Vec3 PhoneToHead(const float v[3])
{
    return {-static_cast<double>(v[1]), static_cast<double>(v[0]), static_cast<double>(v[2])};
}

bool IsFinite(const float v[3])
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

}  // namespace

bool HeadTracker::AddSample(const protocol::Packet& packet)
{
    if (packet.type != protocol::PacketType::Sensors) {
        return false;
    }
    if (!IsFinite(packet.gyro) || !IsFinite(packet.accel)) {
        return false;
    }
    const Vec3 gyro = PhoneToHead(packet.gyro);
    const Vec3 accel = PhoneToHead(packet.accel);
    const double accel_norm = Length(accel);
    if (Length(gyro) > kMaxGyro || accel_norm > kMaxAccel) {
        return false;
    }

    if (has_data_) {
        const int32_t seq_step = static_cast<int32_t>(packet.seq - last_seq_);
        if (seq_step <= 0 && seq_step > kRestartSeqJump) {
            return false;  // duplicate or out of order
        }
    }
    const double dt = has_data_ ? static_cast<double>(packet.timestamp_ns - last_timestamp_ns_) * 1e-9 : 0.0;
    last_seq_ = packet.seq;
    last_timestamp_ns_ = packet.timestamp_ns;
    has_data_ = true;

    LearnBias(gyro, accel, dt);
    const Vec3 rate = gyro - gyro_bias_;
    const Vec3 previous_rate = angular_velocity_;
    angular_velocity_ = rate;

    if (dt <= 0.0 || dt > kMaxStepSeconds) {
        return true;  // first sample or a long pause: nothing to integrate
    }
    if (dt > kGapSeconds) {
        ++gaps_;
        gap_seconds_ += dt;
    }
    // Between two samples the head turned at about the average of both rates.
    Vec3 omega = (rate + previous_rate) * 0.5;

    // Pull the estimated "up" towards the measured one.
    if (accel_norm > 0.0 && std::fabs(accel_norm - kGravity) < kAccelTrustBand) {
        const Vec3 measured_up = accel * (1.0 / accel_norm);
        const Vec3 estimated_up = Rotate(Conjugate(orientation_), Vec3{0, 1, 0});
        omega = omega + Cross(measured_up, estimated_up) * (kTiltGain * std::min(dt, kGapSeconds) / dt);
    }

    const double angle = Length(omega) * dt;
    if (angle > 0.0) {
        orientation_ = Normalize(orientation_ * AxisAngle(omega * (1.0 / Length(omega)), angle));
    }

    if (mag_gain_ > 0.0 && IsFinite(packet.mag)) {
        CorrectYaw(packet.mag, dt);
    }
    return true;
}

void HeadTracker::LearnBias(Vec3 gyro, Vec3 accel, double dt)
{
    if (dt <= 0.0 || dt > kGapSeconds) {
        window_ = BiasWindow{};
        return;
    }
    BiasWindow& w = window_;
    const double accel_norm = Length(accel);
    if (w.seconds == 0.0) {
        w.accel_min = w.accel_max = accel_norm;
    }
    w.seconds += dt;
    w.samples += 1;
    w.sum = w.sum + gyro;
    w.squares += Dot(gyro, gyro);
    w.accel_sum = w.accel_sum + accel;
    w.accel_min = std::min(w.accel_min, accel_norm);
    w.accel_max = std::max(w.accel_max, accel_norm);
    if (w.seconds < kBiasWindowSeconds) {
        return;
    }
    const Vec3 mean = w.sum * (1.0 / w.samples);
    const double spread = std::sqrt(std::max(w.squares / w.samples - Dot(mean, mean), 0.0));
    const bool steady = w.accel_max - w.accel_min < kStillAccelRange;
    const bool flat = std::fabs(w.accel_sum.z / w.samples) > kFlatGravity * kGravity;
    double seconds = 0.0;  // over which the bias follows
    if (steady && flat && spread < kTableSpread) {
        seconds = kTableBiasSeconds;
        ++table_seconds_;
    } else if (steady && spread < kWornSpread && Length(mean - gyro_bias_) < kWornMean) {
        seconds = kWornBiasSeconds;
    }
    if (seconds > 0.0) {
        gyro_bias_ = gyro_bias_ + (mean - gyro_bias_) * std::min(w.seconds / seconds, 1.0);
        if (Length(gyro_bias_) > kMaxBias) {
            gyro_bias_ = gyro_bias_ * (kMaxBias / Length(gyro_bias_));
        }
    }
    window_ = BiasWindow{};
}

HeadTracker::Stats HeadTracker::TakeStats()
{
    Stats stats{gaps_, gap_seconds_, table_seconds_, Length(gyro_bias_)};
    gaps_ = 0;
    gap_seconds_ = 0.0;
    table_seconds_ = 0;
    return stats;
}

void HeadTracker::CorrectYaw(const float mag[3], double dt)
{
    const Vec3 field = Rotate(orientation_, PhoneToHead(mag));
    const double norm = Length(field);
    if (norm < kMagMinNorm || norm > kMagMaxNorm) {
        return;
    }
    const double dip = field.y / norm;
    Vec3 horizontal{field.x, 0.0, field.z};
    const double horizontal_norm = Length(horizontal);
    if (horizontal_norm < kMagMinHorizontal * norm) {
        return;
    }
    horizontal = horizontal * (1.0 / horizontal_norm);

    if (mag_seed_count_ == 0) {
        mag_norm_avg_ = norm;
        mag_dip_avg_ = dip;
    } else if (std::fabs(norm - mag_norm_avg_) > kMagNormBand * mag_norm_avg_ ||
               std::fabs(dip - mag_dip_avg_) > kMagDipBand) {
        return;  // something nearby is bending the field
    }
    mag_norm_avg_ += (norm - mag_norm_avg_) * kMagAverageRate;
    mag_dip_avg_ += (dip - mag_dip_avg_) * kMagAverageRate;

    if (mag_seed_count_ < kMagSeedSamples) {
        mag_seed_sum_ = mag_seed_sum_ + horizontal;
        if (++mag_seed_count_ == kMagSeedSamples) {
            const double length = Length(mag_seed_sum_);
            if (length < 0.5 * kMagSeedSamples) {
                mag_seed_count_ = 0;  // the field kept moving while seeding: start over
                mag_seed_sum_ = Vec3{};
                return;
            }
            mag_reference_ = mag_seed_sum_ * (1.0 / length);
        }
        return;
    }

    // Signed yaw from the reference to the current direction, right-handed about world +y.
    const double error = std::atan2(Cross(mag_reference_, horizontal).y, Dot(mag_reference_, horizontal));
    const double step = std::clamp(-mag_gain_ * error, -kMagMaxRate, kMagMaxRate) * dt;
    orientation_ = Normalize(AxisAngle(Vec3{0, 1, 0}, step) * orientation_);
}

void HeadTracker::Recenter()
{
    RecenterTowards(Vec3{0, 0, -1});
}

Quat HeadTracker::RecenterTowards(Vec3 world_forward)
{
    const Vec3 forward = Rotate(orientation_, Vec3{0, 0, -1});
    const double yaw = std::atan2(-forward.x, -forward.z);
    const double target_yaw = std::atan2(-world_forward.x, -world_forward.z);
    const Quat old_recenter = recenter_;
    recenter_ = AxisAngle(Vec3{0, 1, 0}, target_yaw - yaw);
    return Normalize(recenter_ * Conjugate(old_recenter));
}

Quat HeadTracker::Orientation() const
{
    return Normalize(recenter_ * orientation_);
}

Vec3 HeadTracker::AngularVelocity() const
{
    return Rotate(Orientation(), angular_velocity_);
}
