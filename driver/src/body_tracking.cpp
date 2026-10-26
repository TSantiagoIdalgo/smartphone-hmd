#include "body_tracking.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "driver_log.h"
#include "mono_clock.h"
#include "move_hand.h"
#include "psms_client.h"
#include "settings.h"
#include "tracker_device.h"
#include "udp_socket.h"

namespace {

constexpr double kCountdownSeconds = 5.0;
constexpr double kProgressSeconds = 5.0;
constexpr double kReportSeconds = 10.0;
constexpr double kPublishSeconds = 1.0 / 120.0;
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979;
constexpr double kFewDegreesApart = 40.0;  // between two cameras' views, for depth
// A tracker with no pose this long shows as disconnected: VRChat then does not count it, instead
// of waiting in T-pose for full body trackers that have nowhere to be.
constexpr double kConnectedSeconds = 10.0;

// Ctrl+Alt+C starts calibrating the body cameras, or stops it.
bool CalibrateHotkeyDown()
{
    return HotkeyDown('C');
}

BodySessionConfig ReadSessionConfig()
{
    BodySessionConfig config;
    const double head_latency_ms = GetDoubleSetting("head_light_latency_ms", 40.0);
    config.move_latency_seconds = std::clamp(GetDoubleSetting("body_psms_latency_ms", head_latency_ms), 0.0, 200.0) / 1000.0;
    config.calibration_controller_id = GetIntSetting("body_calibration_controller_id", -1);
    config.filter.pixel_noise = std::clamp(GetDoubleSetting("body_pixel_noise", 3.0), 0.5, 20.0);
    config.filter.agility = std::clamp(GetDoubleSetting("body_agility", 1.0), 0.01, 100.0);
    config.filter.prediction = std::clamp(GetDoubleSetting("body_prediction", 0.3), 0.0, 1.0);
    return config;
}

// The setting a camera's calibration is saved under.
std::string CameraKey(uint32_t id)
{
    char key[64];
    std::snprintf(key, sizeof(key), "body_camera_%08X", static_cast<unsigned>(id));
    return key;
}

// As the pose program and the phone show it.
unsigned ShortId(uint32_t id)
{
    return static_cast<unsigned>(id & 0xFFFF);
}

}  // namespace

BodyTracking::BodyTracking(PsmsClient* psms, TrackerDevice* waist, TrackerDevice* left_foot, TrackerDevice* right_foot,
                           HeadSightingQueue* head, SharedWrists* wrists)
    : psms_(psms),
      trackers_{waist, left_foot, right_foot},
      head_(head),
      wrists_(wrists),
      session_(ReadSessionConfig())
{
    port_ = GetIntSetting("body_port", body_protocol::kPort);
    calibration_seconds_ = std::clamp(GetDoubleSetting("body_calibration_seconds", 60.0), 10.0, 600.0);
}

BodyTracking::~BodyTracking()
{
    Stop();
}

void BodyTracking::Start()
{
    if (running_) {
        return;
    }
    running_ = true;
    thread_ = std::thread(&BodyTracking::Run, this);
}

void BodyTracking::Stop()
{
    running_ = false;
    if (thread_.joinable()) {
        thread_.join();
    }
}

void BodyTracking::Run()
{
    UdpSocket socket;
    if (!socket.Open(static_cast<uint16_t>(port_), 5)) {
        DriverLog("Body tracking: could not listen on UDP port %d", port_);
        return;
    }
    DriverLog("Body tracking: listening for tools/body_tracker.py on UDP port %d; Ctrl+Alt+C calibrates the cameras "
              "with a PS Move",
              port_);
    uint8_t buffer[2048];
    std::vector<ControllerSample> moves;
    std::vector<HeadSighting> sightings;
    while (running_) {
        const int size = socket.Receive(buffer, sizeof(buffer));
        const double now = MonoSeconds();
        body_protocol::Frame frame;
        if (size > 0 && body_protocol::Parse(buffer, static_cast<size_t>(size), &frame) &&
            session_.AddFrame(frame, now)) {
            OnNewCamera(frame.camera);
        }
        session_.TakeHeadSightings(&sightings);
        for (const HeadSighting& sighting : sightings) {
            if (head_) {
                head_->Push(sighting);
            }
        }
        if (psms_) {
            psms_->DrainControllers(&moves);
            for (const ControllerSample& move : moves) {
                session_.AddMove(move);
            }
        }

        const bool hotkey_down = CalibrateHotkeyDown();
        if (hotkey_down && !hotkey_was_down_) {
            ToggleCalibration(now);
        }
        hotkey_was_down_ = hotkey_down;
        if (calibration_ == Calibration::Countdown && now >= calibration_at_) {
            session_.StartCalibration();
            calibration_ = Calibration::Collecting;
            calibration_at_ = now;
            next_progress_ = now + kProgressSeconds;
            DriverLog("Body calibration: now. Wave the Move slowly for about a minute; Ctrl+Alt+C stops");
        }
        if (calibration_ == Calibration::Collecting) {
            const bool enough = session_.CalibrationProgress().enough;
            if (enough || now - calibration_at_ >= calibration_seconds_) {
                if (enough) {
                    DriverLog("Body calibration: enough samples");
                }
                FinishCalibration();
            } else if (now >= next_progress_) {
                next_progress_ = now + kProgressSeconds;
                LogProgress();
            }
        }

        if (now >= next_publish_) {
            next_publish_ = now + kPublishSeconds;
            Publish(now);
        }
        if (now >= next_report_) {
            if (next_report_ > 0.0) {
                LogReport(now);
            } else {
                session_.TakeReport(now);  // starts the counting
            }
            next_report_ = now + kReportSeconds;
        }
    }
}

void BodyTracking::OnNewCamera(uint32_t id)
{
    const std::string saved = GetStringSetting(CameraKey(id).c_str());
    CameraModel camera;
    if (!saved.empty() && CameraModel::Parse(saved, &camera)) {
        session_.SetCamera(id, camera);
    }
    if (session_.Calibrated(id)) {
        DriverLog("Body camera %04X: first frame; using its saved calibration", ShortId(id));
    } else if (!saved.empty()) {
        DriverLog("Body camera %04X: first frame; its saved calibration was for the picture turned another way: "
                  "calibrate again (Ctrl+Alt+C)",
                  ShortId(id));
    } else {
        DriverLog("Body camera %04X: first frame; not calibrated yet (Ctrl+Alt+C)", ShortId(id));
    }
}

void BodyTracking::ToggleCalibration(double now)
{
    if (calibration_ != Calibration::Off) {
        calibration_ = Calibration::Off;
        session_.CancelCalibration();
        DriverLog("Body calibration: stopped (Ctrl+Alt+C); nothing changed");
        return;
    }
    if (!psms_) {
        DriverLog("Body calibration: needs PSMoveServiceEx, for the Move's position");
        return;
    }
    calibration_ = Calibration::Countdown;
    calibration_at_ = now + kCountdownSeconds;
    DriverLog("Body calibration: starts in %.0f s. Turn on ONE Move only and wave it slowly around the room, high and "
              "low, near and far, where both phones and the PS Eye see it; the phones ring its bulb in cyan",
              kCountdownSeconds);
}

void BodyTracking::LogProgress()
{
    const BodySession::Progress progress = session_.CalibrationProgress();
    std::string text = "Body calibration:";
    for (const BodySession::CameraProgress& camera : progress.cameras) {
        char part[160];
        std::snprintf(part, sizeof(part), " camera %04X %d samples over %.1f x %.1f x %.1f m;", ShortId(camera.id),
                      camera.samples, camera.extent.x, camera.extent.y, camera.extent.z);
        text += part;
    }
    if (progress.ambiguous > 0) {
        text += " " + std::to_string(progress.ambiguous) + " bulbs left out: more than one Move was on";
    }
    DriverLog("%s", text.c_str());
}

void BodyTracking::FinishCalibration()
{
    calibration_ = Calibration::Off;
    LogProgress();
    for (const BodySession::CameraCalibration& camera : session_.FinishCalibration()) {
        const CalibrationResult& r = camera.result;
        if (!r.ok) {
            DriverLog("Body calibration: camera %04X not calibrated: %s", ShortId(camera.id), r.problem.c_str());
            continue;
        }
        const Vec3 forward = Row(r.camera.rotation, 2);
        const double down = std::asin(std::clamp(-forward.y, -1.0, 1.0)) * kRadiansToDegrees;
        DriverLog("Body calibration: camera %04X calibrated, %.1f px (%.1f cm) off on average over %d samples, %d left "
                  "out; at (%.2f, %.2f, %.2f) m, looking %.0f degrees %s, %.0f degrees across",
                  ShortId(camera.id), r.rms_px, r.rms_m * 100.0, r.inliers, r.outliers, r.camera.position.x,
                  r.camera.position.y, r.camera.position.z, std::fabs(down), down >= 0.0 ? "down" : "up",
                  2.0 * std::atan(0.5 * r.camera.width / r.camera.focal) * kRadiansToDegrees);
        vr::VRSettings()->SetString(kSettingsSection, CameraKey(camera.id).c_str(), r.camera.Serialize().c_str());
    }
}

void BodyTracking::Publish(double now)
{
    const TrackerPose poses[3] = {session_.Waist(now), session_.Foot(true, now), session_.Foot(false, now)};
    ++published_;
    for (int i = 0; i < 3; ++i) {
        if (poses[i].valid) {
            spreads_[i].Add(poses[i].position);
            last_valid_[i] = now;
        }
        vr::DriverPose_t pose{};
        pose.qWorldFromDriverRotation.w = 1.0;
        pose.qDriverFromHeadRotation.w = 1.0;
        pose.deviceIsConnected = now - last_valid_[i] < kConnectedSeconds;
        pose.poseIsValid = poses[i].valid;
        pose.result = poses[i].valid ? vr::TrackingResult_Running_OK : vr::TrackingResult_Running_OutOfRange;
        pose.vecPosition[0] = poses[i].position.x;
        pose.vecPosition[1] = poses[i].position.y;
        pose.vecPosition[2] = poses[i].position.z;
        const Quat q = poses[i].valid ? poses[i].orientation : Quat{};
        pose.qRotation.w = q.w;
        pose.qRotation.x = q.x;
        pose.qRotation.y = q.y;
        pose.qRotation.z = q.z;
        trackers_[i]->Update(pose);
    }
    if (wrists_) {
        Wrists wrists;
        for (int side = 0; side < 2; ++side) {
            const TrackerPose wrist = session_.Wrist(side == 0, now);
            wrists.valid[side] = wrist.valid && !wrist.holding;
            wrists.position[side] = wrist.position;
            wrists.velocity[side] = wrist.velocity;
        }
        wrists_->Set(wrists);
    }
}

void BodyTracking::LogReport(double now)
{
    const BodySession::Report report = session_.TakeReport(now);
    for (const BodySession::CameraReport& c : report.cameras) {
        if (c.frames_per_second <= 0.0) {
            DriverLog("Body camera %04X: no frames (is tools/body_tracker.py running?)", ShortId(c.id));
            continue;
        }
        DriverLog("Body camera %04X: %.1f frames/s of %dx%d, taken %.0f ms before arriving, %.0f%% with another "
                  "camera's frame within 20 ms; a person in %.0f%%, the Move's bulb in %.0f%%; %s",
                  ShortId(c.id), c.frames_per_second, c.width, c.height, c.age_ms, c.paired_share * 100.0,
                  c.person_share * 100.0, c.ball_share * 100.0,
                  c.calibrated ? "calibrated" : "not calibrated (Ctrl+Alt+C)");
    }
    const BodyFilter::Stats& f = report.filter;
    if (f.updates + f.outliers > 0) {
        DriverLog("Body: %d points used, %d left out as wrong, %d frames with left and right swapped, %d joints "
                  "(re)started",
                  f.updates, f.outliers, f.swapped, f.started);
    }
    if (spreads_[0].count + spreads_[1].count + spreads_[2].count > 0) {
        static const char* const kNames[3] = {"hips", "left foot", "right foot"};
        std::string text = "Body trackers (height above the floor, lowest to highest, and how far from where it was "
                           "on average: its tremor when still):";
        for (int i = 0; i < 3; ++i) {
            const PoseSpread& s = spreads_[i];
            char part[128];
            if (s.count == 0) {
                std::snprintf(part, sizeof(part), "%s %s nowhere", i > 0 ? ";" : "", kNames[i]);
            } else {
                std::snprintf(part, sizeof(part), "%s %s %.2f m (%.2f to %.2f), %.1f cm, %.0f%% of the time",
                              i > 0 ? ";" : "", kNames[i], s.Mean().y, s.lowest, s.highest, s.Deviation() * 100.0,
                              100.0 * s.count / published_);
            }
            text += part;
        }
        DriverLog("%s", text.c_str());
    }
    for (PoseSpread& s : spreads_) {
        s = PoseSpread{};
    }
    published_ = 0;
    if (report.checks > 0) {
        // Two cameras close together see a point from nearly the same side: across their view they
        // place it well, but in depth several times worse.
        DriverLog("Body check: the cameras put the Move's bulb %.1f cm from where PSMoveServiceEx has it on average "
                  "(%.1f cm at most), %d times; %.1f cm of it in depth and %.1f cm across. They see it %.0f degrees "
                  "apart%s",
                  report.check_mean_m * 100.0, report.check_max_m * 100.0, report.checks,
                  report.check_depth_m * 100.0, report.check_across_m * 100.0, report.check_angle_degrees,
                  report.check_angle_degrees < kFewDegreesApart ? ": too few for depth; place the phones 60 to 90 "
                                                                  "degrees apart around where you play"
                                                                : "");
    } else if (report.check_ambiguous > 0) {
        DriverLog("Body check: none; both phones saw a bulb %d times, but more than one Move was on and either "
                  "could be it: turn off all but one",
                  report.check_ambiguous);
    }
}
