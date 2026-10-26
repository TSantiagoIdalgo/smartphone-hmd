#include "body_session.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kMoveHistorySeconds = 3.0;
constexpr double kMaxMoveGapSeconds = 0.12;      // between the two Move samples around a frame
constexpr double kMoveOnSeconds = 0.5;           // a sample this near a frame: the Move is on
constexpr double kSpeedSeconds = 0.1;
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
constexpr double kDegreesPerRadian = 180.0 / 3.14159265358979;
constexpr int kLeftEar = 7;  // among MediaPipe's pose points
constexpr int kRightEar = 8;
constexpr size_t kMaxHeadSightings = 64;

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
    camera.times.insert(std::upper_bound(camera.times.begin(), camera.times.end(), time), time);

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

        const float* left = frame.point[kLeftEar];
        const float* right = frame.point[kRightEar];
        const double visibility = std::min(left[3], right[3]);
        if (visibility >= config_.filter.min_visibility && head_sightings_.size() < kMaxHeadSightings) {
            HeadSighting sighting;
            sighting.camera = frame.camera;
            sighting.time = time;
            sighting.model = camera.model;
            sighting.u = 0.5 * (left[0] + right[0]) * frame.width;
            sighting.v = 0.5 * (left[1] + right[1]) * frame.height;
            sighting.noise = config_.filter.pixel_noise * std::max(frame.width, frame.height) / 640.0 / visibility;
            head_sightings_.push_back(sighting);
        }
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
        // Not while calibrating: the calibration about to be replaced may no longer fit.
        if (camera.calibrated && !calibrating_) {
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
                const bool one = MoveAt(0.5 * (time + other.ball.time), &move, &speed, &ambiguous);
                check_ambiguous_ += ambiguous;
                if (one && speed < kMaxCheckSpeed &&
                    Triangulate({{&camera.model, u, v}, {&other.model, other.ball.u, other.ball.v}}, &seen)) {
                    const Vec3 off = seen - move;
                    const double distance = Length(off);
                    const Vec3 a = Unit(seen - camera.model.position), b = Unit(seen - other.model.position);
                    const Vec3 depth = Unit(a + b);
                    const double along = Dot(off, depth);
                    ++checks_;
                    check_sum_ += distance;
                    check_max_ = std::max(check_max_, distance);
                    check_depth_sum_ += std::fabs(along);
                    check_across_sum_ += Length(off - depth * along);
                    check_angle_sum_ += std::acos(std::clamp(Dot(a, b), -1.0, 1.0));
                }
            }
        }
    }
    return fresh;
}

void BodySession::TakeHeadSightings(std::vector<HeadSighting>* out)
{
    out->swap(head_sightings_);
    head_sightings_.clear();
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
    // PSMoveServiceEx's positions are of a moment a little before they arrive.
    const double latency = config_.move_latency_seconds;
    const auto at = [latency](const std::deque<ControllerSample>& history, double t, Vec3* out) {
        const ControllerSample* before = nullptr;
        const ControllerSample* after = nullptr;
        for (const ControllerSample& s : history) {
            if (s.time - latency <= t) {
                before = &s;
            } else {
                after = &s;
                break;
            }
        }
        if (!before || !after || !before->tracking || !after->tracking ||
            after->time - before->time > kMaxMoveGapSeconds) {
            return false;
        }
        const double t0 = before->time - latency, t1 = after->time - latency;
        *out = before->position + (after->position - before->position) * (t1 > t0 ? (t - t0) / (t1 - t0) : 0.0);
        return true;
    };
    // Every Move that is on has its bulb lit, tracked or not, and the phones may see that one.
    int on = 0;
    bool found = false;
    for (const auto& entry : moves_) {
        if (config_.calibration_controller_id >= 0 && entry.first != config_.calibration_controller_id) {
            continue;
        }
        const std::deque<ControllerSample>& history = entry.second;
        const bool lit = std::any_of(history.begin(), history.end(), [&](const ControllerSample& s) {
            return std::fabs(s.time - latency - time) <= kMoveOnSeconds;
        });
        on += lit;
        // How fast over a tenth of a second, not between two samples: PSMoveServiceEx's own
        // jitter would make a Move held still look fast.
        Vec3 early, late;
        if (lit && at(history, time, position) && at(history, time - 0.5 * kSpeedSeconds, &early) &&
            at(history, time + 0.5 * kSpeedSeconds, &late)) {
            *speed = Length(late - early) / kSpeedSeconds;
            found = true;
        }
    }
    *ambiguous = on > 1;
    return found && on == 1;
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
    // Paired when another camera took a frame within 20 ms, whichever arrived first.
    const auto paired = [this](uint32_t id, double time) {
        for (const auto& entry : cameras_) {
            const std::vector<double>& times = entry.second.times;
            const auto next = std::lower_bound(times.begin(), times.end(), time - kPairWindowSeconds);
            if (entry.first != id && next != times.end() && *next <= time + kPairWindowSeconds) {
                return true;
            }
        }
        return false;
    };
    std::map<uint32_t, int> paired_frames;
    for (const auto& entry : cameras_) {
        for (double time : entry.second.times) {
            paired_frames[entry.first] += paired(entry.first, time);
        }
    }
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
            r.paired_share = static_cast<double>(paired_frames[entry.first]) / camera.frames;
        }
        camera.frames = camera.persons = camera.balls = 0;
        camera.ages.clear();
        report.cameras.push_back(r);
    }
    for (auto& entry : cameras_) {
        entry.second.times.clear();
    }
    report.filter = filter_.TakeStats();
    report.checks = checks_;
    report.check_ambiguous = check_ambiguous_;
    check_ambiguous_ = 0;
    report.check_mean_m = checks_ > 0 ? check_sum_ / checks_ : 0.0;
    report.check_max_m = check_max_;
    if (checks_ > 0) {
        report.check_depth_m = check_depth_sum_ / checks_;
        report.check_across_m = check_across_sum_ / checks_;
        report.check_angle_degrees = check_angle_sum_ / checks_ * kDegreesPerRadian;
    }
    checks_ = 0;
    check_sum_ = check_max_ = check_depth_sum_ = check_across_sum_ = check_angle_sum_ = 0.0;
    return report;
}
