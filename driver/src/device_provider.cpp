#include "device_provider.h"

#include "body_tracking.h"
#include "driver_log.h"
#include "psms_client.h"
#include "settings.h"
#include "tracker_device.h"

DeviceProvider::DeviceProvider() = default;

DeviceProvider::~DeviceProvider() = default;

vr::EVRInitError DeviceProvider::Init(vr::IVRDriverContext* driver_context)
{
    VR_INIT_SERVER_DRIVER_CONTEXT(driver_context);

    const bool head = GetBoolSetting("head_position_enabled", false);
    const bool body = GetBoolSetting("body_tracking_enabled", false);
    PsmsConfig psms;
    if (head || body) {
        psms.dll_path = GetStringSetting("psms_client_dll");
        psms.host = GetStringSetting("psms_host", "127.0.0.1");
        psms.port = GetStringSetting("psms_port", "9512");
        psms.follow_head = head;
        psms.hmd_id = GetIntSetting("psms_hmd_id", -1);
        psms.controller_id = GetIntSetting("psms_controller_id", -1);
        psms.stream_controllers = body;
        psms_ = std::make_unique<PsmsClient>();
    }

    hmd_ = std::make_unique<HmdDevice>(head ? psms_.get() : nullptr);
    if (!vr::VRServerDriverHost()->TrackedDeviceAdded(
            hmd_->SerialNumber().c_str(), vr::TrackedDeviceClass_HMD, hmd_.get())) {
        DriverLog("Failed to add the HMD device");
        return vr::VRInitError_Driver_Unknown;
    }

    if (body) {
        struct Tracker {
            const char* serial;
            const char* role;
            const char* type;
        };
        static const Tracker kTrackers[] = {
            {"SMARTPHONE-HMD-WAIST", "TrackerRole_Waist", "vive_tracker_waist"},
            {"SMARTPHONE-HMD-LEFT-FOOT", "TrackerRole_LeftFoot", "vive_tracker_left_foot"},
            {"SMARTPHONE-HMD-RIGHT-FOOT", "TrackerRole_RightFoot", "vive_tracker_right_foot"},
        };
        for (const Tracker& t : kTrackers) {
            trackers_.push_back(std::make_unique<TrackerDevice>(t.serial, t.role, t.type));
            if (!vr::VRServerDriverHost()->TrackedDeviceAdded(t.serial, vr::TrackedDeviceClass_GenericTracker,
                                                              trackers_.back().get())) {
                DriverLog("Body tracking: could not add the %s tracker", t.serial);
            }
        }
        body_ = std::make_unique<BodyTracking>(psms_.get(), trackers_[0].get(), trackers_[1].get(), trackers_[2].get());
        body_->Start();
        DriverLog("Body tracking: on, with hip and foot trackers");
        if (!head) {
            DriverLog("Body tracking: head_position_enabled is off, so the head stays put while the hips and feet "
                      "move: turn it on for the body to line up with the head");
        }
    }
    if (psms_) {
        psms_->Start(psms);
    }
    return vr::VRInitError_None;
}

void DeviceProvider::Cleanup()
{
    // The threads go first, then what they use.
    body_ = nullptr;
    hmd_ = nullptr;
    psms_ = nullptr;
    trackers_.clear();
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
