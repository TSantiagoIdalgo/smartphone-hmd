#pragma once

#include <array>
#include <vector>

#include "camera_model.h"
#include "math3d.h"

// Turns the 2D pose points of two or more calibrated cameras into the body in 3D, and the body
// into the poses of the hip and foot trackers, and the wrists the PS Moves fall back on.
//
// The cameras are not synchronized, so frames are not paired: each joint has its own Kalman
// filter (position and velocity), and every camera frame updates it with the pixel where that
// camera saw the joint, at the time the camera took it. Two cameras pin a joint down; one alone
// still corrects it sideways, and its depth holds from before. A frame that arrives after a newer
// one from the other camera is applied as of its own time.
//
// How quickly a joint may change speed adapts to how well it is being predicted: a still joint
// is smoothed a lot, a moving one followed closely, as MediaPipe's own smoothing does in 2D.
//
// The pose model sometimes mixes up left and right, mostly from behind; each frame is checked
// both ways against the filter, and the better one is used.
//
// World: the same space as the cameras' calibration, SteamVR raw space in metres. Times are
// seconds on one clock (MonoSeconds()).

// The joints the trackers and the hands need. Even ones are the person's left.
enum BodyJoint {
    kLeftShoulder,
    kRightShoulder,
    kLeftHip,
    kRightHip,
    kLeftKnee,
    kRightKnee,
    kLeftAnkle,
    kRightAnkle,
    kLeftHeel,
    kRightHeel,
    kLeftToe,
    kRightToe,
    kLeftWrist,
    kRightWrist,
    kJointCount
};

// Each joint's number among MediaPipe's 33 pose points.
constexpr int kJointLandmark[kJointCount] = {11, 12, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 15, 16};

// One camera frame: where the camera saw each joint, in pixels, and how sure the pose model was
// that the joint is in view (0 to 1).
struct BodyView {
    int camera = 0;     // index in the cameras given to the filter
    double time = 0.0;  // when the camera took the frame
    std::array<double, kJointCount> u{};
    std::array<double, kJointCount> v{};
    std::array<double, kJointCount> visibility{};
};

struct BodyFilterConfig {
    double pixel_noise = 3.0;    // pixels, at full visibility in a picture 640 pixels across
    double min_visibility = 0.5;  // points less visible than this are left out
    double agility = 1.0;         // how quickly the joints are expected to change speed (scale)
    double prediction = 0.3;      // share of the velocity extrapolated past the newest frame
};

struct TrackerPose {
    bool valid = false;
    bool holding = false;  // nothing new for a moment: the last pose stays
    Vec3 position;
    Quat orientation;  // tracker axes to world: +x the person's right, +y up, +z back
    Vec3 velocity;
};

class BodyFilter {
public:
    struct Stats {
        int updates = 0;
        int outliers = 0;
        int swapped = 0;  // frames whose left and right were the wrong way round
        int late = 0;     // points older than what their joint already had: left out
        int started = 0;  // joints found again after not being followed
    };

    explicit BodyFilter(const BodyFilterConfig& config = BodyFilterConfig());

    // The calibrated cameras; frames name them by their index here. Forgets the body.
    void SetCameras(const std::vector<CameraModel>& cameras);
    void Add(BodyView view);

    // Where a joint is at `now`; false if it is not followed.
    bool Joint(int joint, double now, Vec3* position, bool* holding = nullptr) const;
    TrackerPose Waist(double now) const;
    TrackerPose Foot(bool left, double now) const;
    // Position and velocity only.
    TrackerPose Wrist(bool left, double now) const;

    // Counters since the last call.
    Stats TakeStats();

private:
    struct State {
        bool live = false;
        Vec3 position;
        Vec3 velocity;
        double covariance[6][6] = {};
        double time = 0.0;         // of the state: the newest frame applied
        double last_update = 0.0;  // of the newest frame that agreed with it
        int misses = 0;            // frames in a row that did not
        double agility = 1.0;      // scale, learned from how well the joint is predicted
    };

    void Predict(State* state, double to, int joint) const;
    // The squared Mahalanobis distance of a measurement, and optionally applies it.
    double Update(State* state, int joint, const CameraModel& camera, double time, double u, double v,
                  double visibility, bool apply);
    void Start(const BodyView& view);
    Vec3 Output(const State& state, double now) const;
    bool Usable(const State& state, double now, bool* holding) const;

    BodyFilterConfig config_;
    std::vector<CameraModel> cameras_;
    std::vector<BodyView> last_views_;  // each camera's newest frame, for starting joints
    std::vector<bool> has_view_;
    std::array<State, kJointCount> joints_{};
    Stats stats_;
};
