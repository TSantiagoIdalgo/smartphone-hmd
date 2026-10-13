#include "device_provider.h"

#include "driver_log.h"

vr::EVRInitError DeviceProvider::Init(vr::IVRDriverContext* driver_context)
{
    VR_INIT_SERVER_DRIVER_CONTEXT(driver_context);

    hmd_ = std::make_unique<HmdDevice>();
    if (!vr::VRServerDriverHost()->TrackedDeviceAdded(
            hmd_->SerialNumber().c_str(), vr::TrackedDeviceClass_HMD, hmd_.get())) {
        DriverLog("Failed to add the HMD device");
        return vr::VRInitError_Driver_Unknown;
    }
    return vr::VRInitError_None;
}

void DeviceProvider::Cleanup()
{
    hmd_ = nullptr;
}

const char* const* DeviceProvider::GetInterfaceVersions()
{
    return vr::k_InterfaceVersions;
}

void DeviceProvider::RunFrame()
{
    vr::VREvent_t event{};
    while (vr::VRServerDriverHost()->PollNextEvent(&event, sizeof(event))) {
    }
}

bool DeviceProvider::ShouldBlockStandbyMode()
{
    return false;
}

void DeviceProvider::EnterStandby() {}

void DeviceProvider::LeaveStandby() {}
