#pragma once

#include <deque>

#include "math3d.h"

// Turns the position of a light on the headset, seen by the PS Eye through PSMoveServiceEx, into
// the eye position SteamVR needs, while the phone keeps providing the rotation.
//
// The light arrives about 40 ms later than the phone's rotation. Filtering the light directly
// would make every head turn lag too, because turning swings the light around the neck. So the
// filter tracks the neck pivot instead: each light sample is moved to the neck with the phone
// rotation from the moment the camera saw it, and the eyes are put back on top with the newest
// rotation. Turning the head then has the phone's latency; only moving the whole head (crouching,
// leaning, stepping) carries the camera's, partly hidden by extrapolating the pivot's velocity.
//
// The PS Moves' bulbs go through the same filter, with no offsets.
//
// World: SteamVR raw space in metres, which is PSMoveServiceEx tracking space / 100 once VDM's
// playspace calibration is reset. Head axes: +x right, +y up, +z back, origin between the eyes.
// Times are seconds on one steady clock, when samples arrived at the PC.
struct HeadPositionConfig {
    Vec3 eyes_from_light;                     // head axes, m
    Vec3 eyes_from_neck{0.0, 0.075, -0.08};   // head axes, m
    Vec3 default_eyes{0.0, 1.7, 0.0};         // until the light is first seen
    double latency_seconds = 0.040;           // camera path minus phone path
    double camera_period_seconds = 1.0 / 60.0;
    double lateral_noise_m = 0.002;           // per camera frame at 2 m, across the camera ray
    double depth_noise_m = 0.03;              // per camera frame at 2 m, along the ray
    double agility = 1.0;                     // process noise, m^2/s^3, at its usual scale
    double prediction_gain = 0.8;             // share of the pivot velocity extrapolated
    double max_speed = 3.0;                   // m/s
    double min_eye_height = 0.2;              // m; lights that put the eyes outside are wrong
    double max_eye_height = 2.4;
    // For a Move's bulb: a jump no hand makes between two camera frames (a bulb half hidden, put
    // far away) does not make the filter readier to follow.
    bool distrust_jumps = false;
};

struct HeadPoseOut {
    Vec3 position;  // eyes, world
    Vec3 velocity;
};

// Without the head light: the neck stays where it is when the eyes are at `eyes` looking straight
// ahead (identity orientation), and the eyes turn around it.
HeadPoseOut NeckModel(Vec3 eyes, Vec3 eyes_from_neck, Quat orientation, Vec3 world_angular_velocity);

class HeadPositionEstimator {
public:
    enum class State {
        NeverSeen,  // eyes at default_eyes, with our own neck model
        Live,
        Holding,    // the light is lost: the neck pivot stays where it was
        Frozen,     // Ctrl+Alt+P: like Holding, ignoring the light
    };

    struct Stats {
        int accepted = 0;
        int rejected = 0;     // outliers
        int implausible = 0;  // eyes underground, too high, or too close/far from the camera
        int resets = 0;
        int losses = 0;
        double depth_innovation_rms_m = 0.0;
        double lateral_innovation_rms_m = 0.0;
        double eye_height_m = 0.0;
        double camera_distance_m = 0.0;
        double agility = 0.0;    // its scale on average: below 1 the head is smoothed more
    };

    explicit HeadPositionEstimator(const HeadPositionConfig& config = HeadPositionConfig());

    // Each accepted phone sample: when it arrived and the head orientation it produced.
    void AddOrientation(double time, Quat orientation);
    // The camera position in the world, used to tell depth noise from sideways noise.
    void SetCamera(bool valid, Vec3 position, int cameras);
    // How far past now the output rotation is predicted; the pivot is extrapolated as far.
    void SetHorizon(double seconds) { horizon_ = seconds; }
    // A light sample. `tracking` false means the camera lost it.
    void AddLight(double time, Vec3 position, bool tracking, Vec3 world_angular_velocity);
    // Call often. `source_ok` is false while PSMoveServiceEx is not streaming.
    void Tick(double now, bool source_ok);
    void SetFrozen(bool frozen, double now);
    // A recenter turned the world by `delta` on top of `before`: keeps the light where it is.
    void OnYawChange(Quat delta, Quat before);

    HeadPoseOut Output(double now, Quat orientation, Vec3 world_angular_velocity) const;
    State state() const { return state_; }
    // Counters since the last call.
    Stats TakeStats();

private:
    struct Rejected {
        double time;
        Vec3 pivot;
    };

    // A light sample as a measurement of the neck pivot.
    struct Measurement {
        Vec3 pivot;
        Mat3 noise;
        Vec3 blind;  // the direction it measures poorly (unit, or zero): the PS Eye's depth
        double weight = 1.0;  // how much it counts towards the agility
    };

    Quat OrientationAt(double time) const;
    Mat3 MeasurementNoise(Vec3 light, Vec3 world_angular_velocity, double since_previous) const;
    Vec3 Pivot(double now, Vec3* velocity) const;
    Vec3 BlendOffset(double now) const;
    void StartBlend(double now, Vec3 pivot_before);
    void Reset(double measured_at, Vec3 pivot, const Mat3& noise);
    void EnterHolding(double now);
    void Predict(double to);
    // The squared Mahalanobis distance of a measurement of the pivot, and optionally applies it.
    // Negative if it cannot be used.
    double Correct(const Measurement& m, bool apply);

    HeadPositionConfig config_;
    Vec3 light_from_neck_;  // eyes_from_neck - eyes_from_light

    struct TimedOrientation {
        double time;
        Quat orientation;
    };
    std::deque<TimedOrientation> history_;

    bool camera_valid_ = false;
    Vec3 camera_;
    int cameras_ = 0;
    double horizon_ = 0.0;

    State state_ = State::NeverSeen;
    bool ever_seen_ = false;
    bool tracking_ = false;
    bool have_last_light_ = false;
    Vec3 last_light_;
    double last_light_time_ = 0.0;
    double last_accept_time_ = 0.0;

    // Constant-velocity Kalman filter on the neck pivot, at the time the camera saw it.
    Vec3 pivot_;
    Vec3 velocity_;
    Mat3 p_pp_, p_pv_, p_vv_;
    double state_time_ = 0.0;
    // Scales the process noise, learned from how well the head is being predicted.
    double agility_ = 1.0;
    // The light's recent innovations along its poor direction, normalized and averaged: noise
    // averages out, moving towards or away from the PS Eye does not.
    double depth_trend_ = 0.0;

    std::deque<Rejected> rejected_;

    // Smooths jumps (first sighting, reacquiring, resets) out over a short time.
    Vec3 blend_;
    double blend_time_ = 0.0;
    // Accepted samples right after a first sighting, reacquisition or reset are blended too.
    double blend_event_time_ = -1e9;

    Stats stats_;
    double depth_innovation_sum_ = 0.0;
    double lateral_innovation_sum_ = 0.0;
    double agility_sum_ = 0.0;
};
