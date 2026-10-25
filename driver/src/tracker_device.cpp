#include "tracker_device.h"

#include <utility>

#include "chaperone.h"

TrackerDevice::TrackerDevice(std::string serial, std::string role, std::string type)
    : serial_(std::move(serial)), role_(std::move(role)), type_(std::move(type))
{
    pose_.qWorldFromDriverRotation.w = 1.0;
    pose_.qDriverFromHeadRotation.w = 1.0;
    pose_.qRotation.w = 1.0;
    pose_.deviceIsConnected = true;
    pose_.poseIsValid = false;
    pose_.result = vr::TrackingResult_Running_OutOfRange;
}

vr::EVRInitError TrackerDevice::Activate(uint32_t object_id)
{
    device_index_ = object_id;
    const vr::PropertyContainerHandle_t container = vr::VRProperties()->TrackedDeviceToPropertyContainer(object_id);
    vr::VRProperties()->SetStringProperty(container, vr::Prop_ModelNumber_String, "Smartphone HMD body tracker");
    vr::VRProperties()->SetStringProperty(container, vr::Prop_ManufacturerName_String, "smartphone-hmd");
    // Looks and behaves as a Vive tracker, which every game knows.
    vr::VRProperties()->SetStringProperty(container, vr::Prop_RenderModelName_String, "{htc}vr_tracker_vive_1_0");
    vr::VRProperties()->SetStringProperty(container, vr::Prop_InputProfilePath_String,
                                          "{htc}/input/vive_tracker_profile.json");
    vr::VRProperties()->SetStringProperty(container, vr::Prop_ControllerType_String, type_.c_str());
    vr::VRProperties()->SetInt32Property(container, vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_OptOut);
    // The headset's space.
    vr::VRProperties()->SetUint64Property(container, vr::Prop_CurrentUniverseId_Uint64, kUniverseId);
    // The role SteamVR's "Manage Trackers" would set, so it needs no setting up.
    const std::string path = "/devices/smartphone_hmd/" + serial_;
    vr::VRSettings()->SetString(vr::k_pch_Trackers_Section, path.c_str(), role_.c_str());
    return vr::VRInitError_None;
}

void TrackerDevice::Deactivate()
{
    device_index_ = vr::k_unTrackedDeviceIndexInvalid;
}

void TrackerDevice::DebugRequest(const char*, char* response_buffer, uint32_t response_buffer_size)
{
    if (response_buffer_size >= 1) {
        response_buffer[0] = 0;
    }
}

vr::DriverPose_t TrackerDevice::GetPose()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return pose_;
}

void TrackerDevice::Update(const vr::DriverPose_t& pose)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pose_ = pose;
    }
    const uint32_t index = device_index_;
    if (index != vr::k_unTrackedDeviceIndexInvalid) {
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(index, pose, sizeof(vr::DriverPose_t));
    }
}
