#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <vector>

#include "body_filter.h"
#include "body_protocol.h"
#include "camera_model.h"
#include "head_position.h"
#include "psms_client.h"

// Everything the body tracking does with the cameras' frames, apart from SteamVR: puts each frame
// on the driver's clock, keeps per-camera numbers for the log, calibrates the cameras against a
// PS Move that PSMoveServiceEx follows, checks the calibration against that Move afterwards, and
// feeds the calibrated cameras' points to the body filter.
//
// Calibrating: while someone waves a lit Move around the room, each camera frame that shows its
// bulb is paired with where PSMoveServiceEx had the Move when the camera took the frame. With
// enough of those spread through the room, each camera's position, direction and lens follow.
// Since the Move's positions are the ones the head light and the hands use, the body comes out in
// the same space as them.
// How a tracker went over a while, for the log: how high on average and from where to where, and
// how far on average from where it was on average, which standing still is its tremor.
struct PoseSpread {
    int count = 0;
    Vec3 sum;
    double squares = 0.0;
    double lowest = 0.0;
    double highest = 0.0;

    void Add(Vec3 p)
    {
        lowest = count == 0 ? p.y : std::min(lowest, p.y);
        highest = count == 0 ? p.y : std::max(highest, p.y);
        ++count;
        sum = sum + p;
        squares += Dot(p, p);
    }
    Vec3 Mean() const { return count > 0 ? sum * (1.0 / count) : Vec3{}; }
    double Deviation() const
    {
        const Vec3 mean = Mean();
        return count > 0 ? std::sqrt(std::max(squares / count - Dot(mean, mean), 0.0)) : 0.0;
    }
};

struct BodySessionConfig {
    double move_latency_seconds = 0.040;  // from the PS Eye seeing the Move to its position arriving
    int calibration_controller_id = -1;   // -1: whichever single Move is tracked
    BodyFilterConfig filter;
};

class BodySession {
public:
    struct CameraReport {
        uint32_t id = 0;
        int width = 0;
        int height = 0;
        bool calibrated = false;
        double frames_per_second = 0.0;
        double age_ms = 0.0;        // median, from the camera taking a frame to it arriving here
        double person_share = 0.0;  // frames with a person
        double ball_share = 0.0;    // frames with a Move's bulb
        double paired_share = 0.0;  // frames another camera took within 20 ms of
    };

    struct Report {
        std::vector<CameraReport> cameras;
        BodyFilter::Stats filter;
        // The calibration checked against the Move: where the cameras put its bulb, as far from
        // PSMoveServiceEx's position on average and at most.
        int checks = 0;
        int check_ambiguous = 0;  // not checked: more than one Move was on, either could be the bulb
        double check_mean_m = 0.0;
        double check_max_m = 0.0;
        // Of that, how much along the cameras' view (in depth, where two cameras close together
        // pin a point down worst) and how much across it; and how far apart the cameras see it.
        double check_depth_m = 0.0;
        double check_across_m = 0.0;
        double check_angle_degrees = 0.0;
    };

    struct CameraProgress {
        uint32_t id = 0;
        int samples = 0;
        Vec3 extent;  // of the samples in the room, m
    };

    struct Progress {
        std::vector<CameraProgress> cameras;
        int ambiguous = 0;  // bulbs seen while more than one Move was on: not used
        bool enough = false;  // every camera has enough samples, spread enough
    };

    struct CameraCalibration {
        uint32_t id = 0;
        CalibrationResult result;  // applied when ok
    };

    explicit BodySession(const BodySessionConfig& config = BodySessionConfig());

    // A frame from the pose program that arrived at `arrived` (MonoSeconds()). True the first
    // time a camera shows up, so its saved calibration can be given with SetCamera.
    bool AddFrame(const body_protocol::Frame& frame, double arrived);
    void AddMove(const ControllerSample& sample);
    void SetCamera(uint32_t id, const CameraModel& camera);
    bool Calibrated(uint32_t id) const;

    void StartCalibration();
    void CancelCalibration();
    bool Calibrating() const { return calibrating_; }
    Progress CalibrationProgress() const;
    // Solves every camera with enough samples and uses the good ones.
    std::vector<CameraCalibration> FinishCalibration();

    TrackerPose Waist(double now) const { return filter_.Waist(now); }
    TrackerPose Foot(bool left, double now) const { return filter_.Foot(left, now); }
    TrackerPose Wrist(bool left, double now) const { return filter_.Wrist(left, now); }
    // Where the calibrated cameras saw the head (midway between the ears) since the last call.
    void TakeHeadSightings(std::vector<HeadSighting>* out);
    // Where they saw the lit bulbs by the hands since the last call, whichever Moves they are.
    void TakeBulbSightings(std::vector<HeadSighting>* out);

    // Numbers since the last call.
    Report TakeReport(double now);

private:
    struct Sighting {
        bool valid = false;
        double time = 0.0;
        double u = 0.0;
        double v = 0.0;
    };

    struct Camera {
        int width = 0;
        int height = 0;
        bool calibrated = false;
        CameraModel model;
        int index = -1;  // in the filter
        Sighting ball;
        std::vector<CalibrationSample> samples;
        // Report counters.
        int frames = 0;
        int persons = 0;
        int balls = 0;
        std::vector<double> ages;
        std::vector<double> times;  // when the frames were taken, sorted
    };

    // Where the Move was at `time`, and how fast it went; false unless a single Move was on then,
    // and tracked.
    bool MoveAt(double time, Vec3* position, double* speed, bool* ambiguous) const;
    void RebuildFilter();

    BodySessionConfig config_;
    std::map<uint32_t, Camera> cameras_;
    std::map<int, std::deque<ControllerSample>> moves_;
    BodyFilter filter_;
    std::vector<HeadSighting> head_sightings_;
    std::vector<HeadSighting> bulb_sightings_;
    bool calibrating_ = false;
    int ambiguous_ = 0;
    double report_start_ = -1.0;
    int checks_ = 0;
    int check_ambiguous_ = 0;
    double check_sum_ = 0.0;
    double check_max_ = 0.0;
    double check_depth_sum_ = 0.0;
    double check_across_sum_ = 0.0;
    double check_angle_sum_ = 0.0;
};
