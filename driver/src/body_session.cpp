#include "body_session.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kMoveHistorySeconds = 3.0;
constexpr double kMaxMoveGapSeconds = 0.12;      // between the two Move samples around a frame
constexpr double kMaxCalibrationSpeed = 0.5;     // m/s: faster, a few ms of timing error matter
constexpr double kMinSampleSpacingM = 0.03;      // a Move held still adds one sample, not hundreds
constexpr size_t kMinCalibrationSamples = 60;
constexpr int kEnoughSamples = 300;
constexpr double kEnoughWidthM = 1.0;   // across the room, the wider way
constexpr double kEnoughDepthM = 0.5;   // the narrower way
constexpr double kEnoughHeightM = 0.8;
constexpr double kPairWindowSeconds = 0.02;
constexpr double kCheckWindowSeconds = 0.025;
constexpr double kMaxCheckSpeed = 0.3;  // m/s
constexpr size_t kRecentFrames = 4;

double Median(std::vector<double> values)
{
    if (values.empty()) {
        return 0.0;
    }
    std::nth_element(values.begin(), values.begin() + values.size() / 2, values.end());
    return values[values.size() / 2];
}

Vec3 Extent(const std::vector<CalibrationSample>& samples)
{
    if (samples.empty()) {
        return {};
    }
    Vec3 low = samples.front().world, high = low;
    for (const CalibrationSample& s : samples) {
        low = {std::min(low.x, s.world.x), std::min(low.y, s.world.y), std::min(low.z, s.world.z)};
        high = {std::max(high.x, s.world.x), std::max(high.y, s.world.y), std::max(high.z, s.world.z)};
    }
    return high - low;
}

}  // namespace

BodySession::BodySession(const BodySessionConfig& config) : config_(config), filter_(config.filter) {}

bool BodySession::AddFrame(const body_protocol::Frame& frame, double arrived)
{
    const double time = arrived - frame.age_seconds;
    const bool fresh = cameras_.find(frame.camera) == cameras_.end();
    Camera& camera = cameras_[frame.camera];
    if (camera.width != frame.width || camera.height != frame.height) {
        camera.width = frame.width;
        camera.height = frame.height;
        if (camera.calibrated && (camera.model.width != frame.width || camera.model.height != frame.height)) {
            camera.calibrated = false;  // turned, or another resolution: the calibration no longer fits
            RebuildFilter();
        }
    }

    ++camera.frames;
    camera.ages.push_back(frame.age_seconds * 1000.0);
    bool paired = false;
    for (const auto& entry : cameras_) {
        if (entry.first == frame.camera) {
            continue;
        }
        for (double t : entry.second.recent) {
            paired = paired || std::fabs(t - time) <= kPairWindowSeconds;
        }
    }
    camera.paired += paired;
    camera.recent.push_back(time);
    if (camera.recent.size() > kRecentFrames) {
        camera.recent.pop_front();
    }

    const bool person = (frame.flags & body_protocol::kFlagPose) && frame.points >= body_protocol::kPoints;
    camera.persons += person;
    if (person && camera.calibrated && camera.index >= 0) {
        BodyView view;
        view.camera = camera.index;
        view.time = time;
        for (int j = 0; j < kJointCount; ++j) {
            const float* p = frame.point[kJointLandmark[j]];
            view.u[j] = p[0] * frame.width;
            view.v[j] = p[1] * frame.height;
            view.visibility[j] = p[3];
        }
        filter_.Add(view);
    }

    if (frame.flags & body_protocol::kFlagBall) {
        ++camera.balls;
        const double u = frame.ball[0] * frame.width, v = frame.ball[1] * frame.height;
        if (calibrating_) {
            Vec3 move;
            double speed = 0.0;
            bool ambiguous = false;
            if (MoveAt(time, &move, &speed, &ambiguous) && speed < kMaxCalibrationSpeed &&
                (camera.samples.empty() || Length(move - camera.samples.back().world) >= kMinSampleSpacingM)) {
                camera.samples.push_back({move, u, v});
            }
            ambiguous_ += ambiguous;
        }
        if (camera.calibrated) {
            camera.ball = {true, time, u, v};
            for (const auto& entry : cameras_) {
                const Camera& other = entry.second;
                if (entry.first == frame.camera || !other.calibrated || !other.ball.valid ||
                    std::fabs(other.ball.time - time) > kCheckWindowSeconds) {
                    continue;
                }
                Vec3 seen, move;
                double speed = 0.0;
                bool ambiguous = false;
                if (Triangulate({{&camera.model, u, v}, {&other.model, other.ball.u, other.ball.v}}, &seen) &&
                    MoveAt(0.5 * (time + other.ball.time), &move, &speed, &ambiguous) && speed < kMaxCheckSpeed) {
                    const double off = Length(seen - move);
                    ++checks_;
                    check_sum_ += off;
                    check_max_ = std::max(check_max_, off);
                }
            }
        }
    }
    return fresh;
}

void BodySession::AddMove(const ControllerSample& sample)
{
    std::deque<ControllerSample>& history = moves_[sample.id];
    history.push_back(sample);
    while (history.front().time < sample.time - kMoveHistorySeconds) {
        history.pop_front();
    }
}

bool BodySession::MoveAt(double time, Vec3* position, double* speed, bool* ambiguous) const
{
    int found = 0;
    for (const auto& entry : moves_) {
        if (config_.calibration_controller_id >= 0 && entry.first != config_.calibration_controller_id) {
            continue;
        }
        // PSMoveServiceEx's positions are of a moment a little before they arrive.
        const ControllerSample* before = nullptr;
        const ControllerSample* after = nullptr;
        for (const ControllerSample& s : entry.second) {
            if (s.time - config_.move_latency_seconds <= time) {
                before = &s;
            } else {
                after = &s;
                break;
            }
        }
        if (!before || !after || !before->tracking || !after->tracking) {
            continue;
        }
        const double t0 = before->time - config_.move_latency_seconds;
        const double t1 = after->time - config_.move_latency_seconds;
        if (t1 - t0 > kMaxMoveGapSeconds) {
            continue;
        }
        const double f = t1 > t0 ? (time - t0) / (t1 - t0) : 0.0;
        *position = before->position + (after->position - before->position) * f;
        *speed = t1 > t0 ? Length(after->position - before->position) / (t1 - t0) : 0.0;
        ++found;
    }
    *ambiguous = found > 1;
    return found == 1;
}

void BodySession::SetCamera(uint32_t id, const CameraModel& model)
{
    Camera& camera = cameras_[id];
    if (camera.width != 0 && (camera.width != model.width || camera.height != model.height)) {
        return;  // calibrated with the picture another size
    }
    camera.width = model.width;
    camera.height = model.height;
    camera.model = model;
    camera.calibrated = true;
    RebuildFilter();
}

bool BodySession::Calibrated(uint32_t id) const
{
    const auto found = cameras_.find(id);
    return found != cameras_.end() && found->second.calibrated;
}

void BodySession::RebuildFilter()
{
    std::vector<CameraModel> models;
    for (auto& entry : cameras_) {
        Camera& camera = entry.second;
        camera.index = camera.calibrated ? static_cast<int>(models.size()) : -1;
        if (camera.calibrated) {
            models.push_back(camera.model);
        }
    }
    filter_.SetCameras(models);
}

void BodySession::StartCalibration()
{
    calibrating_ = true;
    ambiguous_ = 0;
    for (auto& entry : cameras_) {
        entry.second.samples.clear();
    }
}

void BodySession::CancelCalibration()
{
    calibrating_ = false;
    for (auto& entry : cameras_) {
        entry.second.samples.clear();
    }
}

BodySession::Progress BodySession::CalibrationProgress() const
{
    Progress progress;
    progress.ambiguous = ambiguous_;
    progress.enough = !cameras_.empty();
    for (const auto& entry : cameras_) {
        CameraProgress camera;
        camera.id = entry.first;
        camera.samples = static_cast<int>(entry.second.samples.size());
        camera.extent = Extent(entry.second.samples);
        const double wide = std::max(camera.extent.x, camera.extent.z);
        const double narrow = std::min(camera.extent.x, camera.extent.z);
        progress.enough = progress.enough && camera.samples >= kEnoughSamples && wide >= kEnoughWidthM &&
                          narrow >= kEnoughDepthM && camera.extent.y >= kEnoughHeightM;
        progress.cameras.push_back(camera);
    }
    return progress;
}

std::vector<BodySession::CameraCalibration> BodySession::FinishCalibration()
{
    calibrating_ = false;
    std::vector<CameraCalibration> results;
    for (auto& entry : cameras_) {
        Camera& camera = entry.second;
        CameraCalibration calibration;
        calibration.id = entry.first;
        if (camera.samples.size() < kMinCalibrationSamples) {
            calibration.result.problem = "too few samples: the camera hardly saw the Move's bulb";
        } else {
            calibration.result = CalibrateCamera(camera.samples, camera.width, camera.height);
            if (calibration.result.ok) {
                camera.model = calibration.result.camera;
                camera.calibrated = true;
            }
        }
        camera.samples.clear();
        results.push_back(calibration);
    }
    RebuildFilter();
    return results;
}

BodySession::Report BodySession::TakeReport(double now)
{
    Report report;
    const double seconds = report_start_ >= 0.0 ? now - report_start_ : 0.0;
    report_start_ = now;
    for (auto& entry : cameras_) {
        Camera& camera = entry.second;
        CameraReport r;
        r.id = entry.first;
        r.width = camera.width;
        r.height = camera.height;
        r.calibrated = camera.calibrated;
        if (camera.frames > 0) {
            r.frames_per_second = seconds > 0.0 ? camera.frames / seconds : 0.0;
            r.age_ms = Median(camera.ages);
            r.person_share = static_cast<double>(camera.persons) / camera.frames;
            r.ball_share = static_cast<double>(camera.balls) / camera.frames;
            r.paired_share = static_cast<double>(camera.paired) / camera.frames;
        }
        camera.frames = camera.persons = camera.balls = camera.paired = 0;
        camera.ages.clear();
        report.cameras.push_back(r);
    }
    report.filter = filter_.TakeStats();
    report.checks = checks_;
    report.check_mean_m = checks_ > 0 ? check_sum_ / checks_ : 0.0;
    report.check_max_m = check_max_;
    checks_ = 0;
    check_sum_ = check_max_ = 0.0;
    return report;
}
