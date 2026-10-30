#include "device_provider.h"

#include <algorithm>

#include "driver_log.h"
#include "move_controllers.h"
#include "psms_client.h"
#include "settings.h"

DeviceProvider::DeviceProvider() = default;

DeviceProvider::~DeviceProvider() = default;

vr::EVRInitError DeviceProvider::Init(vr::IVRDriverContext* driver_context)
{
    VR_INIT_SERVER_DRIVER_CONTEXT(driver_context);

    const bool head = GetBoolSetting("head_position_enabled", false);
    const bool controllers = GetBoolSetting("controllers_enabled", false);
    PsmsConfig psms;
    if (head || controllers) {
        psms.dll_path = GetStringSetting("psms_client_dll");
        psms.host = GetStringSetting("psms_host", "127.0.0.1");
        psms.port = GetStringSetting("psms_port", "9512");
        psms.follow_head = head;
        psms.hmd_id = GetIntSetting("psms_hmd_id", -1);
        psms.controller_id = GetIntSetting("psms_controller_id", -1);
        psms.stream_controllers = controllers;
        psms_ = std::make_unique<PsmsClient>();
    }

    hmd_ = std::make_unique<HmdDevice>(head ? psms_.get() : nullptr);
    if (!vr::VRServerDriverHost()->TrackedDeviceAdded(
            hmd_->SerialNumber().c_str(), vr::TrackedDeviceClass_HMD, hmd_.get())) {
        DriverLog("Failed to add the HMD device");
        return vr::VRInitError_Driver_Unknown;
    }

    if (controllers) {
        // A Move's bulb through the head's light filter: no offsets, a hand's speed and heights.
        HeadPositionConfig bulb;
        bulb.eyes_from_light = bulb.eyes_from_neck = Vec3{};
        bulb.latency_seconds = std::clamp(GetDoubleSetting("head_light_latency_ms", 40.0), 0.0, 200.0) / 1000.0;
        bulb.lateral_noise_m = std::max(GetDoubleSetting("controller_lateral_noise_mm", 2.0), 0.1) / 1000.0;
        bulb.depth_noise_m = std::max(GetDoubleSetting("controller_depth_noise_cm", 4.0), 0.01) / 100.0;
        bulb.agility = std::max(GetDoubleSetting("controller_agility", 5.0), 0.01);
        bulb.prediction_gain = std::clamp(GetDoubleSetting("controller_prediction", 1.0), 0.0, 1.0);
        bulb.max_speed = 10.0;
        bulb.min_eye_height = -1.0;
        bulb.max_eye_height = 3.5;
        bulb.distrust_jumps = true;
        HandConfig hand;
        hand.pitch_degrees = GetDoubleSetting("controller_pitch_degrees", 0.0);
        hand.rest_enabled = GetBoolSetting("controller_rest_enabled", true);
        hand.rest_below_m = GetDoubleSetting("controller_rest_below_cm", 50.0) / 100.0;
        hand.rest_forward_m = GetDoubleSetting("controller_rest_forward_cm", 30.0) / 100.0;
        hand.rest_side_m = GetDoubleSetting("controller_rest_side_cm", 20.0) / 100.0;
        // Without the head light, the headset's position says nothing about where the hands can be.
        hand.reach_m = head ? std::max(GetDoubleSetting("controller_reach_cm", 120.0), 0.0) / 100.0 : 0.0;
        hand.found_seconds = std::clamp(GetDoubleSetting("controller_found_ms", 300.0), 0.0, 2000.0) / 1000.0;
        hand.still_mm = std::clamp(GetDoubleSetting("controller_still_mm", 15.0), 0.0, 100.0);
        hand.still_degrees = std::clamp(GetDoubleSetting("controller_still_degrees", 0.6), 0.0, 10.0);
        controllers_ = std::make_unique<MoveControllers>(psms_.get(), bulb, hand);
        controllers_->Start();
        DriverLog("Hands: the PS Moves are controllers here; turn off VDM's controllers in SteamVR");
        if (hand.rest_enabled) {
            DriverLog("Hands: a Move lost a moment goes %.0f cm under the eyes, %.0f cm in front and %.0f cm to its "
                      "side, turning with the headset (controller_rest_*); found again, back to its bulb once the PS "
                      "Eye has seen it for %.0f ms (controller_found_ms)",
                      hand.rest_below_m * 100.0, hand.rest_forward_m * 100.0, hand.rest_side_m * 100.0,
                      hand.found_seconds * 1000.0);
        }
        if (hand.reach_m > 0.0) {
            DriverLog("Hands: a bulb seen more than %.0f cm from the headset, across the floor, is not the hand's "
                      "(controller_reach_cm)",
                      hand.reach_m * 100.0);
        }
        DriverLog("Hands: held still, a hand stays put through %.0f mm and %.1f degrees of shaking "
                  "(controller_still_mm, controller_still_degrees)",
                  hand.still_mm, hand.still_degrees);
    }
    if (psms_) {
        psms_->Start(psms);
    }
    return vr::VRInitError_None;
}

void DeviceProvider::Cleanup()
{
    // The threads go first, then what they use.
    controllers_ = nullptr;
    hmd_ = nullptr;
    psms_ = nullptr;
}

const char* const* DeviceProvider::GetInterfaceVersions()
{
    return vr::k_InterfaceVersions;
}

void DeviceProvider::RunFrame()
{
    vr::VREvent_t event{};
    while (vr::VRServerDriverHost()->PollNextEvent(&event, sizeof(event))) {
        if (controllers_) {
            controllers_->OnEvent(event);
        }
    }
}

bool DeviceProvider::ShouldBlockStandbyMode()
{
    return false;
}

void DeviceProvider::EnterStandby() {}

void DeviceProvider::LeaveStandby() {}
