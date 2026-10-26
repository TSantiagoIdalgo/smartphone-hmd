#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "head_position.h"
#include "head_tracker.h"
#include "openvr_driver.h"
#include "psms_client.h"

class VideoStreamer;
class VirtualDisplay;

struct DisplayConfig {
    int32_t window_x = 0;
    int32_t window_y = 0;
    int32_t window_width = 1920;
    int32_t window_height = 1080;
    int32_t render_width = 1280;
    int32_t render_height = 1440;
    float fov_degrees = 90.0f;
    // False when frames go to the phone through VirtualDisplay instead of a desktop window.
    bool on_desktop = true;
};

// Side-by-side stereo display: left eye on the left half, right eye on the right half.
class DisplayComponent : public vr::IVRDisplayComponent {
public:
    explicit DisplayComponent(const DisplayConfig& config);

    void GetWindowBounds(int32_t* x, int32_t* y, uint32_t* width, uint32_t* height) override;
    bool IsDisplayOnDesktop() override;
    bool IsDisplayRealDisplay() override;
    void GetRecommendedRenderTargetSize(uint32_t* width, uint32_t* height) override;
    void GetEyeOutputViewport(vr::EVREye eye, uint32_t* x, uint32_t* y, uint32_t* width, uint32_t* height) override;
    void GetProjectionRaw(vr::EVREye eye, float* left, float* right, float* top, float* bottom) override;
    vr::DistortionCoordinates_t ComputeDistortion(vr::EVREye eye, float u, float v) override;
    bool ComputeInverseDistortion(vr::HmdVector2_t* result, vr::EVREye eye, uint32_t channel, float u, float v) override;

private:
    DisplayConfig config_;
};

class HmdDevice : public vr::ITrackedDeviceServerDriver {
public:
    // `psms` (shared with the body tracking, may be null) gives the head light's position, and
    // `sightings` (may be null) where the body tracking's cameras see the head.
    HmdDevice(PsmsClient* psms, HeadSightingQueue* sightings);
    ~HmdDevice();

    vr::EVRInitError Activate(uint32_t object_id) override;
    void Deactivate() override;
    void EnterStandby() override;
    void* GetComponent(const char* component_name_and_version) override;
    void DebugRequest(const char* request, char* response_buffer, uint32_t response_buffer_size) override;
    vr::DriverPose_t GetPose() override;

    const std::string& SerialNumber() const { return serial_number_; }

private:
    // Receives the phone's packets, updates the tracker and sends each new pose to SteamVR.
    void TrackingThread();
    // Feeds the head light's samples from PSMoveServiceEx, and the body cameras' views, to the
    // position estimator.
    void UpdateHeadPosition(std::vector<LightSample>* samples, std::vector<HeadSighting>* sightings);
    // Ctrl+Alt+L, and each light sample as PSMoveServiceEx gives it while measuring.
    void MeasureLightDistance(double now, const std::vector<LightSample>& samples);
    void RecenterHead(const char* source);
    void ToggleHeadHold();
    vr::DriverPose_t MakePose(bool receiving, double now) const;

    std::string serial_number_;
    std::string model_number_;
    float display_frequency_ = 60.0f;
    float head_height_ = 1.7f;
    int32_t udp_port_ = 9944;
    std::unique_ptr<DisplayComponent> display_;
#if defined(_WIN32)
    std::unique_ptr<VideoStreamer> video_;
    std::unique_ptr<VirtualDisplay> virtual_display_;
#endif
    uint64_t adapter_luid_ = 0;
    vr::VRInputComponentHandle_t proximity_ = vr::k_ulInvalidInputComponentHandle;

    HeadTracker tracker_;  // only touched by the tracking thread
    double prediction_seconds_ = 0.0;  // same

    Vec3 eyes_from_neck_{0.0, 0.075, -0.08};  // our neck model, head axes, m
    // 3DoF only (Ctrl+Alt+H): the eyes stay exactly still, for VDM's playspace calibration.
    bool head_held_ = false;

    // 6DoF: a light on the visor, tracked by PSMoveServiceEx, and the body cameras give the head a
    // position.
    bool position_enabled_ = false;
    bool apply_head_model_ = false;
    PsmsClient* psms_ = nullptr;
    HeadSightingQueue* sightings_ = nullptr;
    bool use_sightings_ = false;  // Ctrl+Alt+B turns the body cameras' views off and on
    bool sightings_hotkey_was_down_ = false;
    HeadPositionEstimator head_position_;  // only touched by the tracking thread, like the rest
    bool head_camera_valid_ = false;
    Vec3 head_camera_;
    Vec3 head_last_light_;  // newest light position as PSMoveServiceEx gave it, for the log
    int head_cameras_ = 0;
    bool head_frozen_ = false;
    // Undoes PSMoveServiceEx putting the light too near or far from the PS Eye (a wrong ball size).
    double light_distance_scale_ = 1.0;
    // Ctrl+Alt+L: measuring it against a Move's bulb held to the light, whose distance is right.
    bool light_hotkey_was_down_ = false;
    double light_measure_until_ = 0.0;  // 0: not measuring
    std::vector<double> light_ratios_;
    std::vector<MoveState> moves_;
    bool freeze_hotkey_was_down_ = false;
    HeadPositionEstimator::State logged_head_state_ = HeadPositionEstimator::State::NeverSeen;
    bool logged_light_followed_ = false;
    double next_head_stats_ = 0.0;
    std::mutex pose_mutex_;
    vr::DriverPose_t last_pose_{};

    std::atomic<bool> active_{false};
    std::atomic<uint32_t> device_index_{vr::k_unTrackedDeviceIndexInvalid};
    std::thread tracking_thread_;
};
