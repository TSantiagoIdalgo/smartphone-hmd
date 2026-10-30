#pragma once

#include <memory>

#include "hmd_device.h"
#include "openvr_driver.h"

class MoveControllers;
class PsmsClient;

// Entry object SteamVR talks to: creates the HMD (and, with controllers_enabled, the PS Moves as
// controllers) and pumps runtime events.
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
    std::unique_ptr<PsmsClient> psms_;  // shared by the head light and the controllers
    std::unique_ptr<HmdDevice> hmd_;
    std::unique_ptr<MoveControllers> controllers_;
};
