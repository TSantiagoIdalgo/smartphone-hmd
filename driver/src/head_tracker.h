#pragma once

#include <cstdint>

#include "math3d.h"
#include "protocol.h"

// Turns raw phone sensor packets into a head orientation for SteamVR.
//
// Cleaning: drops duplicated, out-of-order and impossible samples, and learns the gyroscope bias
// while the phone keeps still, best lying on a table. Fusion: integrates the gyroscope and pulls
// the tilt towards gravity measured by the accelerometer (Mahony complementary filter). Yaw is
// only corrected by recentering (or the magnetometer, if asked).
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

    struct Stats {
        int gaps = 0;              // samples more than 0.1 s apart...
        double gap_seconds = 0.0;  // ...and the time between them
        int table_seconds = 0;     // seconds lying still on a table, learning the gyroscope's bias
        double bias = 0.0;         // the bias learned, rad/s
    };
    // Since the last call.
    Stats TakeStats();

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
    int gaps_ = 0;
    double gap_seconds_ = 0.0;
    int table_seconds_ = 0;

    // The second of samples being gathered to tell whether the head kept still.
    struct BiasWindow {
        double seconds = 0.0;
        int samples = 0;
        Vec3 sum;
        double squares = 0.0;
        Vec3 accel_sum;
        double accel_min = 0.0;
        double accel_max = 0.0;
    };
    void LearnBias(Vec3 gyro, Vec3 accel, double dt);
    BiasWindow window_;

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
