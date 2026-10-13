#include "hmd_device.h"

#include <chrono>
#include <cmath>
#include <cstring>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "driver_log.h"
#include "protocol.h"
#include "udp_socket.h"
#if defined(_WIN32)
#include "video_streamer.h"
#include "virtual_display.h"
#endif

namespace {

const char* const kSettingsSection = "driver_smartphone_hmd";

std::string GetStringSetting(const char* key)
{
    char buffer[256] = {};
    vr::VRSettings()->GetString(kSettingsSection, key, buffer, sizeof(buffer));
    return buffer;
}

// Ctrl+Alt+R on the PC keyboard recenters the view.
bool RecenterHotkeyDown()
{
#if defined(_WIN32)
    return (GetAsyncKeyState(VK_CONTROL) & 0x8000) && (GetAsyncKeyState(VK_MENU) & 0x8000) &&
           (GetAsyncKeyState('R') & 0x8000);
#else
    return false;
#endif
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
    return config_.on_desktop;
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
    udp_port_ = vr::VRSettings()->GetInt32(kSettingsSection, "udp_port");

    DisplayConfig config;
    config.window_x = vr::VRSettings()->GetInt32(kSettingsSection, "window_x");
    config.window_y = vr::VRSettings()->GetInt32(kSettingsSection, "window_y");
    config.window_width = vr::VRSettings()->GetInt32(kSettingsSection, "window_width");
    config.window_height = vr::VRSettings()->GetInt32(kSettingsSection, "window_height");
    config.render_width = vr::VRSettings()->GetInt32(kSettingsSection, "render_width");
    config.render_height = vr::VRSettings()->GetInt32(kSettingsSection, "render_height");
    config.fov_degrees = vr::VRSettings()->GetFloat(kSettingsSection, "fov_degrees");

#if defined(_WIN32)
    const int bitrate = vr::VRSettings()->GetInt32(kSettingsSection, "video_bitrate_mbps") * 1000000;
    video_ = std::make_unique<VideoStreamer>();
    if (video_->Start(static_cast<int>(display_frequency_), bitrate, &adapter_luid_)) {
        virtual_display_ = std::make_unique<VirtualDisplay>(video_.get(), display_frequency_);
        config.on_desktop = false;
    } else {
        DriverLog("Video streaming unavailable; showing the HMD in a desktop window instead");
        video_.reset();
        adapter_luid_ = 0;
    }
#endif
    display_ = std::make_unique<DisplayComponent>(config);

    DriverLog("Smartphone HMD created: %s (%s)", model_number_.c_str(), serial_number_.c_str());
}

HmdDevice::~HmdDevice() = default;

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
    if (adapter_luid_ != 0) {
        // Frames go to the phone: SteamVR must render on the same GPU we capture from.
        vr::VRProperties()->SetUint64Property(container, vr::Prop_GraphicsAdapterLuid_Uint64, adapter_luid_);
    } else {
        vr::VRProperties()->SetBoolProperty(container, vr::Prop_DisplayDebugMode_Bool, true);
    }

    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        last_pose_ = MakePose(false);
    }
    tracking_thread_ = std::thread(&HmdDevice::TrackingThread, this);
    return vr::VRInitError_None;
}

void HmdDevice::Deactivate()
{
    if (active_.exchange(false) && tracking_thread_.joinable()) {
        tracking_thread_.join();
    }
#if defined(_WIN32)
    if (video_) {
        video_->Stop();
    }
#endif
    device_index_ = vr::k_unTrackedDeviceIndexInvalid;
}

void HmdDevice::EnterStandby() {}

void* HmdDevice::GetComponent(const char* component_name_and_version)
{
    if (std::strcmp(component_name_and_version, vr::IVRDisplayComponent_Version) == 0) {
        return display_.get();
    }
#if defined(_WIN32)
    if (virtual_display_ && std::strcmp(component_name_and_version, vr::IVRVirtualDisplay_Version) == 0) {
        return virtual_display_.get();
    }
#endif
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
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return last_pose_;
}

vr::DriverPose_t HmdDevice::MakePose(bool receiving) const
{
    vr::DriverPose_t pose{};
    pose.qWorldFromDriverRotation.w = 1.0;
    pose.qDriverFromHeadRotation.w = 1.0;

    const Quat orientation = tracker_.Orientation();
    pose.qRotation.w = orientation.w;
    pose.qRotation.x = orientation.x;
    pose.qRotation.y = orientation.y;
    pose.qRotation.z = orientation.z;

    const Vec3 angular_velocity = tracker_.AngularVelocity();
    pose.vecAngularVelocity[0] = angular_velocity.x;
    pose.vecAngularVelocity[1] = angular_velocity.y;
    pose.vecAngularVelocity[2] = angular_velocity.z;

    // 3DoF: the head stays at a fixed height; SteamVR's head model adds the neck movement.
    pose.vecPosition[1] = head_height_;
    pose.poseIsValid = true;
    pose.deviceIsConnected = true;
    pose.result = receiving ? vr::TrackingResult_Running_OK : vr::TrackingResult_Running_OutOfRange;
    pose.shouldApplyHeadModel = true;
    return pose;
}

void HmdDevice::TrackingThread()
{
    UdpSocket socket;
    if (socket.Open(static_cast<uint16_t>(udp_port_), 100)) {
        DriverLog("Listening for the phone on UDP port %d", udp_port_);
    } else {
        DriverLog("Could not listen on UDP port %d", udp_port_);
    }

    using Clock = std::chrono::steady_clock;
    Clock::time_point last_sample = Clock::now() - std::chrono::hours(1);
    bool hotkey_was_down = false;
    uint8_t buffer[1500];

    while (active_) {
        uint32_t sender = 0;
        const int size = socket.Receive(buffer, sizeof(buffer), &sender);
        protocol::Packet packet;
        if (size > 0 && protocol::Parse(buffer, static_cast<size_t>(size), packet)) {
            if (packet.type == protocol::PacketType::Sensors) {
                if (tracker_.AddSample(packet)) {
                    last_sample = Clock::now();
#if defined(_WIN32)
                    if (video_) {
                        video_->OnPhonePacket(sender);
                    }
#endif
                }
            } else if (packet.type == protocol::PacketType::Recenter) {
                tracker_.Recenter();
                DriverLog("Recentered from the phone");
            } else if (packet.type == protocol::PacketType::KeyframeRequest) {
#if defined(_WIN32)
                if (video_) {
                    video_->RequestKeyframe();
                }
#endif
            }
        }

        const bool hotkey_down = RecenterHotkeyDown();
        if (hotkey_down && !hotkey_was_down) {
            tracker_.Recenter();
            DriverLog("Recentered from the keyboard");
        }
        hotkey_was_down = hotkey_down;

        const bool receiving = Clock::now() - last_sample < std::chrono::seconds(1);
        const vr::DriverPose_t pose = MakePose(receiving);
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            last_pose_ = pose;
        }
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(device_index_, pose, sizeof(vr::DriverPose_t));
    }
}
