#pragma once

#include <atomic>
#include <mutex>
#include <string>

#include "openvr_driver.h"

// A body tracker SteamVR sees as a Vive tracker with its role already set (hips, left foot,
// right foot), so VRChat and other games use it for full-body tracking without setting it up in
// SteamVR's "Manage Trackers". The body tracking thread gives it its poses.
class TrackerDevice : public vr::ITrackedDeviceServerDriver {
public:
    // role: SteamVR's name, as "TrackerRole_Waist"; type: as "vive_tracker_waist".
    TrackerDevice(std::string serial, std::string role, std::string type);

    vr::EVRInitError Activate(uint32_t object_id) override;
    void Deactivate() override;
    void EnterStandby() override {}
    void* GetComponent(const char*) override { return nullptr; }
    void DebugRequest(const char*, char* response_buffer, uint32_t response_buffer_size) override;
    vr::DriverPose_t GetPose() override;

    // Any thread.
    void Update(const vr::DriverPose_t& pose);
    const std::string& Serial() const { return serial_; }

private:
    std::string serial_;
    std::string role_;
    std::string type_;
    std::atomic<uint32_t> device_index_{vr::k_unTrackedDeviceIndexInvalid};
    std::mutex mutex_;
    vr::DriverPose_t pose_{};
};
