#pragma once

#include <atomic>
#include <cstdint>
#include <thread>

#include "body_session.h"

class HeadSightingQueue;
class PsmsClient;
class TrackerDevice;

// The body tracking in SteamVR: takes the pose program's frames on their own UDP port, the PS
// Moves from PSMoveServiceEx, Ctrl+Alt+C to calibrate the cameras, and gives the hip and foot
// trackers their poses. The calibrations are saved in SteamVR's settings, one per camera.
class BodyTracking {
public:
    // `psms` may be null (no calibrating then); the trackers stay ours. Where the cameras see the
    // head goes to `head` (may be null), for the headset.
    BodyTracking(PsmsClient* psms, TrackerDevice* waist, TrackerDevice* left_foot, TrackerDevice* right_foot,
                 HeadSightingQueue* head);
    ~BodyTracking();

    void Start();
    void Stop();

private:
    enum class Calibration { Off, Countdown, Collecting };

    void Run();
    void OnNewCamera(uint32_t id);
    void ToggleCalibration(double now);
    void FinishCalibration();
    void Publish(double now);
    void LogReport(double now);
    void LogProgress();

    int port_ = 0;
    double calibration_seconds_ = 60.0;
    PsmsClient* psms_;
    TrackerDevice* trackers_[3];
    HeadSightingQueue* head_;
    BodySession session_;  // only touched by the thread
    // Since the last report: each tracker's poses, and how many times all three were published.
    PoseSpread spreads_[3];
    int published_ = 0;

    Calibration calibration_ = Calibration::Off;
    double calibration_at_ = 0.0;  // when the countdown ends, or the collecting started
    double next_progress_ = 0.0;
    double next_report_ = 0.0;
    double next_publish_ = 0.0;
    bool hotkey_was_down_ = false;

    std::atomic<bool> running_{false};
    std::thread thread_;
};
