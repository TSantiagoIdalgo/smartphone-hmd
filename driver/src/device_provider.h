#pragma once

#include <memory>

#include "hmd_device.h"
#include "openvr_driver.h"

// Entry object SteamVR talks to: creates the HMD and pumps runtime events.
class DeviceProvider : public vr::IServerTrackedDeviceProvider {
public:
    vr::EVRInitError Init(vr::IVRDriverContext* driver_context) override;
    void Cleanup() override;
    const char* const* GetInterfaceVersions() override;
    void RunFrame() override;
    bool ShouldBlockStandbyMode() override;
    void EnterStandby() override;
    void LeaveStandby() override;

private:
    std::unique_ptr<HmdDevice> hmd_;
};
