#pragma once

#include <memory>
#include <vector>

#include "hmd_device.h"
#include "openvr_driver.h"

class BodyTracking;
class MoveControllers;
class PsmsClient;
class SharedWrists;
class TrackerDevice;

// Entry object SteamVR talks to: creates the HMD (and, with body tracking, the hip and foot
// trackers; with controllers_enabled, the PS Moves as controllers) and pumps runtime events.
class DeviceProvider : public vr::IServerTrackedDeviceProvider {
public:
    DeviceProvider();
    ~DeviceProvider();

    vr::EVRInitError Init(vr::IVRDriverContext* driver_context) override;
    void Cleanup() override;
    const char* const* GetInterfaceVersions() override;
    void RunFrame() override;
    bool ShouldBlockStandbyMode() override;
    void EnterStandby() override;
    void LeaveStandby() override;

private:
    std::unique_ptr<PsmsClient> psms_;  // shared by the head light and the body tracking
    std::unique_ptr<HeadSightingQueue> head_sightings_;  // from the body tracking to the headset
    std::unique_ptr<HmdDevice> hmd_;
    std::vector<std::unique_ptr<TrackerDevice>> trackers_;
    std::unique_ptr<SharedWrists> wrists_;  // from the body tracking to the controllers
    std::unique_ptr<HeadSightingQueue> bulb_sightings_;  // the same
    std::unique_ptr<BodyTracking> body_;
    std::unique_ptr<MoveControllers> controllers_;
};
