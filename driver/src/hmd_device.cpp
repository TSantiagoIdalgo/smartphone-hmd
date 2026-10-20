#include "hmd_device.h"

#include <algorithm>
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

#include "chaperone.h"
#include "driver_log.h"
#include "mono_clock.h"
#include "protocol.h"
#include "udp_socket.h"
#if defined(_WIN32)
#include "video_streamer.h"
#include "virtual_display.h"
#endif

namespace {

const char* const kSettingsSection = "driver_smartphone_hmd";

// SteamVR already predicts the head this far past each vsync.
constexpr float kVsyncToPhotonsSeconds = 0.011f;
// Our own prediction covers the rest of the latency the phone measures, up to this much. Beyond
// it, stopping a fast turn overshoots too visibly.
constexpr double kMaxPredictionSeconds = 0.08;

// Recentering towards the camera needs the head at least this far from it, horizontally.
constexpr double kMinCameraDistance = 0.3;
constexpr double kHeadStatsSeconds = 10.0;

std::string GetStringSetting(const char* key, const char* fallback = "")
{
    char buffer[1024] = {};
    vr::EVRSettingsError error = vr::VRSettingsError_None;
    vr::VRSettings()->GetString(kSettingsSection, key, buffer, sizeof(buffer), &error);
    return error == vr::VRSettingsError_None ? buffer : fallback;
}

// The head position settings fall back to these defaults when a key is missing, so updating only
// the DLL (and keeping an older default.vrsettings) still works.
double GetDoubleSetting(const char* key, double fallback)
{
    vr::EVRSettingsError error = vr::VRSettingsError_None;
    const double value = vr::VRSettings()->GetFloat(kSettingsSection, key, &error);
    return error == vr::VRSettingsError_None ? value : fallback;
}

int GetIntSetting(const char* key, int fallback)
{
    vr::EVRSettingsError error = vr::VRSettingsError_None;
    const int value = vr::VRSettings()->GetInt32(kSettingsSection, key, &error);
    return error == vr::VRSettingsError_None ? value : fallback;
}

// Ctrl+Alt+<key> on the PC keyboard.
bool HotkeyDown(int key)
{
#if defined(_WIN32)
    return (GetAsyncKeyState(VK_CONTROL) & 0x8000) && (GetAsyncKeyState(VK_MENU) & 0x8000) &&
           (GetAsyncKeyState(key) & 0x8000);
#else
    (void)key;
    return false;
#endif
}

// Ctrl+Alt+R recenters the view.
bool RecenterHotkeyDown()
{
    return HotkeyDown('R');
}

// Ctrl+Alt+P stops following the head light, and starts again.
bool FreezeHotkeyDown()
{
    return HotkeyDown('P');
}

// Ctrl+Alt+H holds the eyes still, and lets them turn around the neck again.
bool HoldHotkeyDown()
{
    return HotkeyDown('H');
}

const char* HeadStateText(HeadPositionEstimator::State state)
{
    switch (state) {
    case HeadPositionEstimator::State::NeverSeen:
        return "not seen yet";
    case HeadPositionEstimator::State::Live:
        return "tracked";
    case HeadPositionEstimator::State::Holding:
        return "lost, holding the head where it was";
    case HeadPositionEstimator::State::Frozen:
        return "frozen (Ctrl+Alt+P)";
    }
    return "?";
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

    eyes_from_neck_ = Vec3{0.0, GetDoubleSetting("head_neck_to_eyes_up_cm", 7.5),
                           -GetDoubleSetting("head_neck_to_eyes_forward_cm", 8.0)} * 0.01;
    tracker_.SetMagnetometerGain(std::clamp(GetDoubleSetting("magnetometer_gain", 0.0), 0.0, 0.5));
    position_enabled_ = vr::VRSettings()->GetBool(kSettingsSection, "head_position_enabled");
    if (position_enabled_) {
        const double above = GetDoubleSetting("head_light_above_eyes_cm", 0.0);
        const double in_front = GetDoubleSetting("head_light_in_front_of_eyes_cm", 0.0);
        const double right = GetDoubleSetting("head_light_right_of_eyes_cm", 0.0);
        HeadPositionConfig head;
        head.eyes_from_light = Vec3{-right, -above, in_front} * 0.01;
        head.eyes_from_neck = eyes_from_neck_;
        head.default_eyes = Vec3{0.0, head_height_, 0.0};
        head.latency_seconds = std::clamp(GetDoubleSetting("head_light_latency_ms", 40.0), 0.0, 200.0) / 1000.0;
        head.camera_period_seconds = 1.0 / std::max(GetDoubleSetting("head_light_camera_fps", 60.0), 1.0);
        head.lateral_noise_m = std::max(GetDoubleSetting("head_position_lateral_noise_mm", 2.0), 0.1) / 1000.0;
        head.depth_noise_m = std::max(GetDoubleSetting("head_position_depth_noise_cm", 3.0), 0.01) / 100.0;
        head.agility = std::max(GetDoubleSetting("head_position_agility", 1.0), 0.001);
        head.prediction_gain = std::clamp(GetDoubleSetting("head_position_prediction", 0.8), 0.0, 1.0);
        head_position_ = HeadPositionEstimator(head);
        apply_head_model_ = vr::VRSettings()->GetBool(kSettingsSection, "head_position_apply_head_model");

        psms_config_.dll_path = GetStringSetting("psms_client_dll");
        psms_config_.host = GetStringSetting("psms_host", "127.0.0.1");
        psms_config_.port = GetStringSetting("psms_port", "9512");
        psms_config_.hmd_id = GetIntSetting("psms_hmd_id", -1);
        psms_config_.controller_id = GetIntSetting("psms_controller_id", -1);
        psms_ = std::make_unique<PsmsClient>();
        DriverLog("Head position from PSMoveServiceEx: on. Light %.1f cm above, %.1f cm in front and %.1f cm to the "
                  "right of the eyes; camera latency %.0f ms",
                  above, in_front, right, head.latency_seconds * 1000.0);
        if (above == 0.0 && in_front == 0.0 && right == 0.0) {
            DriverLog("Head position: head_light_*_cm are all 0; measure where the light sits on the visor");
        }
    }

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

    DriverLog("Smartphone HMD created: %s (%s), driver build %s", model_number_.c_str(), serial_number_.c_str(),
              SMARTPHONE_HMD_BUILD);
}

HmdDevice::~HmdDevice()
{
    // In case SteamVR cleans up without deactivating: a running std::thread would abort.
    active_ = false;
    if (tracking_thread_.joinable()) {
        tracking_thread_.join();
    }
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
    vr::VRProperties()->SetFloatProperty(container, vr::Prop_SecondsFromVsyncToPhotons_Float, kVsyncToPhotonsSeconds);
    vr::VRProperties()->SetBoolProperty(container, vr::Prop_IsOnDesktop_Bool, false);
    // Our own universe and chaperone, so the standing and seated spaces games ask for exist
    // without a room setup. The bounds stay hidden: nothing measures the room's walls.
    vr::VRProperties()->SetUint64Property(container, vr::Prop_CurrentUniverseId_Uint64, kUniverseId);
    vr::VRProperties()->SetStringProperty(container, vr::Prop_DriverProvidedChaperoneJson_String,
                                          ChaperoneJson(kUniverseId, head_height_).c_str());
    vr::VRProperties()->SetBoolProperty(container, vr::Prop_DriverProvidedChaperoneVisibility_Bool, false);
    // Reports the headset as worn while the phone sends data. Otherwise SteamVR puts it in standby
    // after 5 s without head motion and the compositor stops drawing.
    vr::VRDriverInput()->CreateBooleanComponent(container, "/proximity", &proximity_);
    if (adapter_luid_ != 0) {
        // Frames go to the phone: SteamVR must render on the same GPU we capture from.
        vr::VRProperties()->SetUint64Property(container, vr::Prop_GraphicsAdapterLuid_Uint64, adapter_luid_);
    } else {
        vr::VRProperties()->SetBoolProperty(container, vr::Prop_DisplayDebugMode_Bool, true);
    }

    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        last_pose_ = MakePose(false, MonoSeconds());
    }
    if (psms_) {
        psms_->Start(psms_config_);
    }
    tracking_thread_ = std::thread(&HmdDevice::TrackingThread, this);
    return vr::VRInitError_None;
}

void HmdDevice::Deactivate()
{
    if (active_.exchange(false) && tracking_thread_.joinable()) {
        tracking_thread_.join();
    }
    if (psms_) {
        psms_->Stop();
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

vr::DriverPose_t HmdDevice::MakePose(bool receiving, double now) const
{
    vr::DriverPose_t pose{};
    pose.qWorldFromDriverRotation.w = 1.0;
    pose.qDriverFromHeadRotation.w = 1.0;

    const Vec3 angular_velocity = tracker_.AngularVelocity();
    // The phone shows each frame a while after its sensors moved: render it where the head will be.
    const Quat orientation = Predict(tracker_.Orientation(), angular_velocity, receiving ? prediction_seconds_ : 0.0);
    pose.qRotation.w = orientation.w;
    pose.qRotation.x = orientation.x;
    pose.qRotation.y = orientation.y;
    pose.qRotation.z = orientation.z;

    pose.vecAngularVelocity[0] = angular_velocity.x;
    pose.vecAngularVelocity[1] = angular_velocity.y;
    pose.vecAngularVelocity[2] = angular_velocity.z;

    // Our own neck model, not SteamVR's head model: VDM reads this position to line the PS Move
    // hands up with the head, and SteamVR's model would move it in ways we cannot hold still.
    const Vec3 straight_ahead{0.0, head_height_, 0.0};
    HeadPoseOut head;
    if (position_enabled_) {
        // 6DoF: the eyes, from the head light.
        head = head_position_.Output(now, orientation, angular_velocity);
        pose.shouldApplyHeadModel = apply_head_model_;
    } else if (head_held_) {
        // 3DoF, held: VDM's playspace calibration only treats the visor as fixed in place if its
        // position does not change at all, and the neck model moves it with every small turn.
        head.position = straight_ahead;
    } else {
        // 3DoF: the neck stays put and the eyes turn around it.
        head = NeckModel(straight_ahead, eyes_from_neck_, orientation, angular_velocity);
    }
    pose.vecPosition[0] = head.position.x;
    pose.vecPosition[1] = head.position.y;
    pose.vecPosition[2] = head.position.z;
    pose.vecVelocity[0] = head.velocity.x;
    pose.vecVelocity[1] = head.velocity.y;
    pose.vecVelocity[2] = head.velocity.z;
    pose.poseIsValid = true;
    pose.deviceIsConnected = true;
    // Never out of range because of the head light: that would grey out the view.
    pose.result = receiving ? vr::TrackingResult_Running_OK : vr::TrackingResult_Running_OutOfRange;
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
    bool hold_hotkey_was_down = false;
    bool was_receiving = false;
    uint32_t last_discoverer = 0;
    double logged_prediction = -1.0;
    uint8_t buffer[1500];
    std::vector<LightSample> light_samples;

    while (active_) {
        uint32_t sender = 0;
        uint16_t sender_port = 0;
        const int size = socket.Receive(buffer, sizeof(buffer), &sender, &sender_port);
        protocol::Packet packet;
        if (size > 0 && protocol::Parse(buffer, static_cast<size_t>(size), packet)) {
            if (packet.type == protocol::PacketType::Sensors) {
                if (tracker_.AddSample(packet)) {
                    last_sample = Clock::now();
                    if (position_enabled_) {
                        head_position_.AddOrientation(MonoSeconds(), tracker_.Orientation());
                    }
#if defined(_WIN32)
                    if (video_) {
                        video_->OnPhonePacket(sender, packet.seq);
                    }
#endif
                }
            } else if (packet.type == protocol::PacketType::Discover) {
                uint8_t reply[protocol::kHeaderSize];
                socket.SendTo(sender, sender_port, reply, protocol::WritePcHere(reply));
                if (sender != last_discoverer) {
                    const uint8_t* ip = reinterpret_cast<const uint8_t*>(&sender);
                    DriverLog("A phone at %u.%u.%u.%u is looking for the PC; answered", ip[0], ip[1], ip[2], ip[3]);
                    last_discoverer = sender;
                }
            } else if (packet.type == protocol::PacketType::Latency) {
                prediction_seconds_ =
                    packet.latency_ms > 0
                        ? std::clamp(packet.latency_ms / 1000.0 - kVsyncToPhotonsSeconds, 0.0, kMaxPredictionSeconds)
                        : 0.0;
                if (std::fabs(prediction_seconds_ - logged_prediction) > 0.005) {
                    DriverLog("Phone latency %.0f ms: predicting the head %.0f ms ahead", packet.latency_ms,
                              prediction_seconds_ * 1000.0);
                    logged_prediction = prediction_seconds_;
                }
            } else if (packet.type == protocol::PacketType::Recenter) {
                RecenterHead("the phone");
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
            RecenterHead("the keyboard");
        }
        hotkey_was_down = hotkey_down;
        const bool hold_down = HoldHotkeyDown();
        if (hold_down && !hold_hotkey_was_down) {
            ToggleHeadHold();
        }
        hold_hotkey_was_down = hold_down;
        if (position_enabled_) {
            UpdateHeadPosition(&light_samples);
        }

        const bool receiving = Clock::now() - last_sample < std::chrono::seconds(1);
        if (receiving != was_receiving) {
            vr::VRDriverInput()->UpdateBooleanComponent(proximity_, receiving, 0.0);
            was_receiving = receiving;
        }
        head_position_.SetHorizon(receiving ? prediction_seconds_ : 0.0);
        const vr::DriverPose_t pose = MakePose(receiving, MonoSeconds());
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            last_pose_ = pose;
        }
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(device_index_, pose, sizeof(vr::DriverPose_t));
    }
}

void HmdDevice::UpdateHeadPosition(std::vector<LightSample>* samples)
{
    const double now = MonoSeconds();
    const bool freeze_down = FreezeHotkeyDown();
    if (freeze_down && !freeze_hotkey_was_down_) {
        head_frozen_ = !head_frozen_;
        head_position_.SetFrozen(head_frozen_, now);
    }
    freeze_hotkey_was_down_ = freeze_down;

    const bool streaming = psms_->Drain(samples, &head_camera_valid_, &head_camera_, &head_cameras_);
    head_position_.SetCamera(head_camera_valid_, head_camera_, head_cameras_);
    const Vec3 angular_velocity = tracker_.AngularVelocity();
    for (const LightSample& sample : *samples) {
        head_position_.AddLight(sample.time, sample.position, sample.tracking, angular_velocity);
    }
    head_position_.Tick(now, streaming);

    const HeadPositionEstimator::State state = head_position_.state();
    if (state != logged_head_state_) {
        DriverLog("Head light: %s", HeadStateText(state));
        logged_head_state_ = state;
    }
    if (now >= next_head_stats_) {
        next_head_stats_ = now + kHeadStatsSeconds;
        const HeadPositionEstimator::Stats stats = head_position_.TakeStats();
        if (stats.accepted + stats.rejected + stats.implausible > 0) {
            DriverLog("Head light: %d samples, %d outliers, %d implausible, %d resets, %d losses; innovation %.0f mm "
                      "along the camera ray and %.0f mm across; eyes %.2f m high, light %.2f m from the camera",
                      stats.accepted, stats.rejected, stats.implausible, stats.resets, stats.losses,
                      stats.depth_innovation_rms_m * 1000.0, stats.lateral_innovation_rms_m * 1000.0,
                      stats.eye_height_m, stats.camera_distance_m);
        }
    }
}

// Makes the current direction "forward". With the head light, forward means towards the PS Eye
// from wherever the user stands, which lines the view up with the hands; the user faces the camera
// when recentering. Without it, forward is -z, as in Part 1.
void HmdDevice::RecenterHead(const char* source)
{
    if (position_enabled_ && head_camera_valid_ && head_cameras_ == 1) {
        const double now = MonoSeconds();
        bool turned = false;
        // The eyes move a little with the yaw (the light is not between them), so repeat.
        for (int i = 0; i < 3; ++i) {
            const Vec3 eyes = head_position_.Output(now, tracker_.Orientation(), Vec3{}).position;
            Vec3 to_camera = head_camera_ - eyes;
            to_camera.y = 0.0;
            const double distance = Length(to_camera);
            if (distance < kMinCameraDistance) {
                break;
            }
            const Quat before = tracker_.Orientation();
            head_position_.OnYawChange(tracker_.RecenterTowards(to_camera * (1.0 / distance)), before);
            turned = true;
        }
        if (turned) {
            DriverLog("Recentered from %s, facing the camera", source);
            return;
        }
    }
    const Quat before = tracker_.Orientation();
    const Quat delta = tracker_.RecenterTowards(Vec3{0.0, 0.0, -1.0});
    if (position_enabled_) {
        head_position_.OnYawChange(delta, before);
    }
    DriverLog("Recentered from %s", source);
}

void HmdDevice::ToggleHeadHold()
{
    if (position_enabled_) {
        DriverLog("Ctrl+Alt+H only works without head_position_enabled; Ctrl+Alt+P freezes the head light");
        return;
    }
    head_held_ = !head_held_;
    DriverLog(head_held_ ? "Head held still for VDM's playspace calibration; Ctrl+Alt+H again when it is done"
                         : "Head turns around the neck again");
}
