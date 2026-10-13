#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "head_tracker.h"
#include "openvr_driver.h"

struct DisplayConfig {
    int32_t window_x = 0;
    int32_t window_y = 0;
    int32_t window_width = 1920;
    int32_t window_height = 1080;
    int32_t render_width = 1280;
    int32_t render_height = 1440;
    float fov_degrees = 90.0f;
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
    HmdDevice();

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
    vr::DriverPose_t MakePose(bool receiving) const;

    std::string serial_number_;
    std::string model_number_;
    float display_frequency_ = 60.0f;
    float head_height_ = 1.7f;
    int32_t udp_port_ = 9944;
    std::unique_ptr<DisplayComponent> display_;

    HeadTracker tracker_;  // only touched by the tracking thread
    std::mutex pose_mutex_;
    vr::DriverPose_t last_pose_{};

    std::atomic<bool> active_{false};
    std::atomic<uint32_t> device_index_{vr::k_unTrackedDeviceIndexInvalid};
    std::thread tracking_thread_;
};
