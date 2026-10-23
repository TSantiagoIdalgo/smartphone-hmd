#pragma once

#include <cstdint>

#include "math3d.h"
#include "protocol.h"

// Turns raw phone sensor packets into a head orientation for SteamVR.
//
// Cleaning: drops duplicated, out-of-order and impossible samples, and learns the gyroscope bias
// while the phone is still. Fusion: integrates the gyroscope and pulls the tilt towards gravity
// measured by the accelerometer (Mahony complementary filter). Yaw is only corrected by recentering.
//
// Axes: the phone sits in landscape inside the headset, screen towards the eyes, top edge on the
// user's left. In OpenVR head axes (+x right, +y up, +z back) that maps phone (x, y, z) to (-y, x, z).
class HeadTracker {
public:
    // Returns true if the sample was used.
    bool AddSample(const protocol::Packet& packet);

    // Makes the current horizontal direction the new "forward" (-z).
    void Recenter();

    // Turns the yaw so the current horizontal direction points along `world_forward` instead.
    // Returns the world yaw rotation that was added on top of the old orientation.
    Quat RecenterTowards(Vec3 world_forward);

    // Head orientation in the SteamVR world, with the recenter applied.
    Quat Orientation() const;

    // Angular velocity in the SteamVR world (rad/s), so SteamVR can predict ahead.
    Vec3 AngularVelocity() const;

    bool HasData() const { return has_data_; }

    // Turns on the slow yaw correction from the magnetometer. `gain` is the fraction of the yaw
    // error removed per second (0 = off, the default). It only acts while the field looks clean.
    void SetMagnetometerGain(double gain) { mag_gain_ = gain; }

private:
    bool has_data_ = false;
    uint32_t last_seq_ = 0;
    int64_t last_timestamp_ns_ = 0;

    Quat orientation_;     // body -> world, before recenter
    Quat recenter_;        // yaw correction applied on top
    Vec3 gyro_bias_;       // rad/s, head axes
    Vec3 angular_velocity_;  // bias-corrected, head axes

    // Magnetometer yaw correction. The reference is the horizontal field direction in the
    // pre-recenter world frame, averaged over the first clean samples; recentering does not touch it.
    void CorrectYaw(const float mag[3], double dt);
    double mag_gain_ = 0.0;
    int mag_seed_count_ = 0;
    Vec3 mag_reference_;       // horizontal unit vector, once seeded
    Vec3 mag_seed_sum_;
    double mag_norm_avg_ = 0.0;  // uT, running average of accepted samples
    double mag_dip_avg_ = 0.0;   // sin(inclination) in the world frame
};
