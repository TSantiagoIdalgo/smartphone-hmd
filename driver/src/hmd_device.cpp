#include "hmd_device.h"

#include <chrono>
#include <cmath>
#include <cstring>

#include "driver_log.h"

namespace {

const char* const kSettingsSection = "driver_smartphone_hmd";

std::string GetStringSetting(const char* key)
{
    char buffer[256] = {};
    vr::VRSettings()->GetString(kSettingsSection, key, buffer, sizeof(buffer));
    return buffer;
}

}  // namespace

// ---------------------------------------------------------------------------
// DisplayComponent
// ---------------------------------------------------------------------------

DisplayComponent::DisplayComponent(const DisplayConfig& config) : config_(config) {}

void DisplayComponent::GetWindowBounds(int32_t* x, int32_t* y, uint32_t* width, uint32_t* height)
{
    *x = config_.window_x;
    *y = config_.window_y;
    *width = static_cast<uint32_t>(config_.window_width);
    *height = static_cast<uint32_t>(config_.window_height);
}

bool DisplayComponent::IsDisplayOnDesktop()
{
    return true;
}

bool DisplayComponent::IsDisplayRealDisplay()
{
    return false;
}

void DisplayComponent::GetRecommendedRenderTargetSize(uint32_t* width, uint32_t* height)
{
    *width = static_cast<uint32_t>(config_.render_width);
    *height = static_cast<uint32_t>(config_.render_height);
}

void DisplayComponent::GetEyeOutputViewport(vr::EVREye eye, uint32_t* x, uint32_t* y, uint32_t* width, uint32_t* height)
{
    const uint32_t half_width = static_cast<uint32_t>(config_.window_width) / 2;
    *x = eye == vr::Eye_Left ? 0 : half_width;
    *y = 0;
    *width = half_width;
    *height = static_cast<uint32_t>(config_.window_height);
}

void DisplayComponent::GetProjectionRaw(vr::EVREye, float* left, float* right, float* top, float* bottom)
{
    // Symmetric frustum per eye. Values are tangents of the half angles; top is negative in OpenVR.
    const float half_tan = std::tan(config_.fov_degrees * 0.5f * 3.14159265f / 180.0f);
    *left = -half_tan;
    *right = half_tan;
    *top = -half_tan;
    *bottom = half_tan;
}

vr::DistortionCoordinates_t DisplayComponent::ComputeDistortion(vr::EVREye, float u, float v)
{
    // No distortion here: the phone corrects its own lenses.
    vr::DistortionCoordinates_t coordinates{};
    coordinates.rfRed[0] = coordinates.rfGreen[0] = coordinates.rfBlue[0] = u;
    coordinates.rfRed[1] = coordinates.rfGreen[1] = coordinates.rfBlue[1] = v;
    return coordinates;
}

bool DisplayComponent::ComputeInverseDistortion(vr::HmdVector2_t* result, vr::EVREye, uint32_t, float u, float v)
{
    result->v[0] = u;
    result->v[1] = v;
    return true;
}

// ---------------------------------------------------------------------------
// HmdDevice
// ---------------------------------------------------------------------------

HmdDevice::HmdDevice()
{
    serial_number_ = GetStringSetting("serial_number");
    model_number_ = GetStringSetting("model_number");
    display_frequency_ = vr::VRSettings()->GetFloat(kSettingsSection, "display_frequency");
    head_height_ = vr::VRSettings()->GetFloat(kSettingsSection, "head_height_meters");

    DisplayConfig config;
    config.window_x = vr::VRSettings()->GetInt32(kSettingsSection, "window_x");
    config.window_y = vr::VRSettings()->GetInt32(kSettingsSection, "window_y");
    config.window_width = vr::VRSettings()->GetInt32(kSettingsSection, "window_width");
    config.window_height = vr::VRSettings()->GetInt32(kSettingsSection, "window_height");
    config.render_width = vr::VRSettings()->GetInt32(kSettingsSection, "render_width");
    config.render_height = vr::VRSettings()->GetInt32(kSettingsSection, "render_height");
    config.fov_degrees = vr::VRSettings()->GetFloat(kSettingsSection, "fov_degrees");
    display_ = std::make_unique<DisplayComponent>(config);

    DriverLog("Smartphone HMD created: %s (%s)", model_number_.c_str(), serial_number_.c_str());
}

vr::EVRInitError HmdDevice::Activate(uint32_t object_id)
{
    device_index_ = object_id;
    active_ = true;

    const vr::PropertyContainerHandle_t container =
        vr::VRProperties()->TrackedDeviceToPropertyContainer(object_id);
    vr::VRProperties()->SetStringProperty(container, vr::Prop_ModelNumber_String, model_number_.c_str());

    // The IPD gives each eye its own position, which is what produces the 3D effect.
    const float ipd = vr::VRSettings()->GetFloat(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_IPD_Float);
    vr::VRProperties()->SetFloatProperty(container, vr::Prop_UserIpdMeters_Float, ipd);
    vr::VRProperties()->SetFloatProperty(container, vr::Prop_DisplayFrequency_Float, display_frequency_);
    vr::VRProperties()->SetFloatProperty(container, vr::Prop_UserHeadToEyeDepthMeters_Float, 0.0f);
    vr::VRProperties()->SetFloatProperty(container, vr::Prop_SecondsFromVsyncToPhotons_Float, 0.011f);
    vr::VRProperties()->SetBoolProperty(container, vr::Prop_IsOnDesktop_Bool, false);
    vr::VRProperties()->SetBoolProperty(container, vr::Prop_DisplayDebugMode_Bool, true);

    pose_thread_ = std::thread(&HmdDevice::PoseThread, this);
    return vr::VRInitError_None;
}

void HmdDevice::Deactivate()
{
    if (active_.exchange(false) && pose_thread_.joinable()) {
        pose_thread_.join();
    }
    device_index_ = vr::k_unTrackedDeviceIndexInvalid;
}

void HmdDevice::EnterStandby() {}

void* HmdDevice::GetComponent(const char* component_name_and_version)
{
    if (std::strcmp(component_name_and_version, vr::IVRDisplayComponent_Version) == 0) {
        return display_.get();
    }
    return nullptr;
}

void HmdDevice::DebugRequest(const char*, char* response_buffer, uint32_t response_buffer_size)
{
    if (response_buffer_size >= 1) {
        response_buffer[0] = 0;
    }
}

vr::DriverPose_t HmdDevice::GetPose()
{
    vr::DriverPose_t pose{};
    pose.qWorldFromDriverRotation.w = 1.0;
    pose.qDriverFromHeadRotation.w = 1.0;
    // Fixed orientation for now; milestone 1 replaces it with the phone's sensors.
    pose.qRotation.w = 1.0;
    pose.vecPosition[1] = head_height_;
    pose.poseIsValid = true;
    pose.deviceIsConnected = true;
    pose.result = vr::TrackingResult_Running_OK;
    pose.shouldApplyHeadModel = true;
    return pose;
}

void HmdDevice::PoseThread()
{
    while (active_) {
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(device_index_, GetPose(), sizeof(vr::DriverPose_t));
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
