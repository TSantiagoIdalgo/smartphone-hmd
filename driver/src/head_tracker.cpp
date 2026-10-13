#include "head_tracker.h"

#include <cmath>

namespace {

constexpr double kGravity = 9.81;
constexpr double kMaxGyro = 35.0;          // rad/s; above this the sample is junk
constexpr double kMaxAccel = 16.0 * kGravity;
constexpr double kMaxStepSeconds = 0.1;    // longer gaps are not integrated
constexpr double kTiltGain = 1.0;          // how fast tilt follows gravity (1/s)
constexpr double kAccelTrustBand = 0.15 * kGravity;  // only trust gravity when |a| is close to g
constexpr double kStillGyro = 0.05;        // rad/s
constexpr double kStillAccelBand = 0.03 * kGravity;
constexpr double kBiasRate = 0.002;        // per sample while still
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

    // Learn the gyroscope bias while the phone is not moving.
    const bool still = Length(gyro - gyro_bias_) < kStillGyro && std::fabs(accel_norm - kGravity) < kStillAccelBand;
    if (still) {
        gyro_bias_ = gyro_bias_ + (gyro - gyro_bias_) * kBiasRate;
    }
    Vec3 omega = gyro - gyro_bias_;
    angular_velocity_ = omega;

    if (dt <= 0.0 || dt > kMaxStepSeconds) {
        return true;  // first sample or a gap: nothing to integrate
    }

    // Pull the estimated "up" towards the measured one.
    if (accel_norm > 0.0 && std::fabs(accel_norm - kGravity) < kAccelTrustBand) {
        const Vec3 measured_up = accel * (1.0 / accel_norm);
        const Vec3 estimated_up = Rotate(Conjugate(orientation_), Vec3{0, 1, 0});
        omega = omega + Cross(measured_up, estimated_up) * kTiltGain;
    }

    const double angle = Length(omega) * dt;
    if (angle > 0.0) {
        orientation_ = Normalize(orientation_ * AxisAngle(omega * (1.0 / Length(omega)), angle));
    }
    return true;
}

void HeadTracker::Recenter()
{
    const Vec3 forward = Rotate(orientation_, Vec3{0, 0, -1});
    const double yaw = std::atan2(-forward.x, -forward.z);
    recenter_ = AxisAngle(Vec3{0, 1, 0}, -yaw);
}

Quat HeadTracker::Orientation() const
{
    return Normalize(recenter_ * orientation_);
}

Vec3 HeadTracker::AngularVelocity() const
{
    return Rotate(Orientation(), angular_velocity_);
}
