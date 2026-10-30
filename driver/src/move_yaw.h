#pragma once

#include <deque>

#include "math3d.h"
#include "move_hand.h"

// Undoes the drift of a PS Move's rotation about the vertical, which PSMoveServiceEx's gyroscope
// sums up over time and loses most in fast flicks. The rest of the rotation is kept right by
// gravity; this turn has nothing to go by but the Move's own motion.
//
// When the hand moves briskly, the Move's accelerometer says how its velocity changed, in the
// directions PSMoveServiceEx's rotation has; the PS Eye sees how the bulb's velocity really
// changed. Over a fifth of a second the two match, turned about the vertical by the drift. The
// turn between them, gathered over many such moments, is the correction. The bulb sits a little
// beyond the accelerometer, so the Move's spin is added to its velocity at both ends.
//
// World: PSMoveServiceEx's, SteamVR raw space in metres. Times: MonoSeconds().
class MoveYaw {
public:
    struct Stats {
        int moments = 0;   // brisk moments compared
        double correction_degrees = 0.0;  // as it is now
        double peak_spin_degrees = 0.0;   // the fastest the Move turned, a second
    };

    // `optical_latency`: how much later the bulb's position arrives than the Move's sensors.
    explicit MoveYaw(double optical_latency = 0.040) : optical_latency_(optical_latency) {}

    // Each new state of the Move: its rotation, accelerometer and gyroscope as PSMoveServiceEx has
    // them, and the bulb's position.
    void Add(const MoveState& move, double now);
    // What to turn PSMoveServiceEx's rotation by about the vertical, radians, right-handed.
    double Correction() const { return correction_; }
    // PSMoveServiceEx's rotation was reset: it starts again with no drift.
    void Reset();
    // Counters since the last call.
    Stats TakeStats();

private:
    struct Sample {
        double time;  // when the Move was there
        double x, z;  // across the floor
    };

    // The bulb's velocity across the floor at `time`, from the PS Eye's positions around it.
    bool OpticalVelocity(double time, double* x, double* z) const;
    // The accelerometer's summed velocity of the bulb at `time`, up to a constant.
    bool InertialVelocity(double time, double* x, double* z) const;
    void Compare(double now);

    double optical_latency_;
    std::deque<Sample> optical_;
    std::deque<Sample> inertial_;
    bool have_sensor_ = false;
    double last_sensor_time_ = 0.0;
    Vec3 last_accel_;
    double last_imu_now_ = 0.0;
    double gravity_ = 0.0;  // the accelerometer's vertical reading at rest, signed: its units and sense
    int gravity_samples_ = 0;
    double vx_ = 0.0, vz_ = 0.0;  // summed from the accelerometer
    bool have_position_ = false;
    Vec3 last_position_;
    double next_compare_ = 0.0;
    double cross_ = 0.0, dot_ = 0.0;  // the evidence, fading
    double last_fade_ = 0.0;
    double correction_ = 0.0;
    double last_step_ = 0.0;
    Stats stats_;
};
