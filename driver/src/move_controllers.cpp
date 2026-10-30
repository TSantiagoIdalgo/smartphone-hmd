#include "move_controllers.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <utility>

#include "chaperone.h"
#include "driver_log.h"
#include "mono_clock.h"
#include "psms_client.h"

namespace {

constexpr double kMinPulseSeconds = 0.03;  // the Move's motor needs this long to be felt
constexpr float kTriggerClick = 0.9f;
constexpr double kReportSeconds = 10.0;
constexpr int kLoopMilliseconds = 2;
constexpr double kRecenterHoldSeconds = 1.0;
constexpr double kFullTiltRadians = 25.0 * 3.14159265358979 / 180.0;  // the trackpad's edge

const char* const kHandNames[2] = {"left", "right"};

}  // namespace

// ---------------------------------------------------------------------------
// MoveController
// ---------------------------------------------------------------------------

MoveController::MoveController(std::string serial, bool left) : serial_(std::move(serial)), left_(left)
{
    pose_.qWorldFromDriverRotation.w = 1.0;
    pose_.qDriverFromHeadRotation.w = 1.0;
    pose_.qRotation.w = 1.0;
    pose_.deviceIsConnected = true;
    pose_.poseIsValid = false;
    pose_.result = vr::TrackingResult_Running_OutOfRange;
}

vr::EVRInitError MoveController::Activate(uint32_t object_id)
{
    const vr::PropertyContainerHandle_t container = vr::VRProperties()->TrackedDeviceToPropertyContainer(object_id);
    vr::VRProperties()->SetStringProperty(container, vr::Prop_ModelNumber_String, "PS Move");
    vr::VRProperties()->SetStringProperty(container, vr::Prop_SerialNumber_String, serial_.c_str());
    vr::VRProperties()->SetStringProperty(container, vr::Prop_ManufacturerName_String, "smartphone-hmd");
    // Looks and behaves as a Vive wand, which every game has bindings for.
    vr::VRProperties()->SetStringProperty(container, vr::Prop_RenderModelName_String, "vr_controller_vive_1_5");
    vr::VRProperties()->SetStringProperty(container, vr::Prop_ControllerType_String, "vive_controller");
    vr::VRProperties()->SetStringProperty(container, vr::Prop_InputProfilePath_String,
                                          "{htc}/input/vive_controller_profile.json");
    vr::VRProperties()->SetInt32Property(container, vr::Prop_ControllerRoleHint_Int32,
                                         left_ ? vr::TrackedControllerRole_LeftHand
                                               : vr::TrackedControllerRole_RightHand);
    // For games still on the old input system.
    vr::VRProperties()->SetInt32Property(container, vr::Prop_Axis0Type_Int32, vr::k_eControllerAxis_TrackPad);
    vr::VRProperties()->SetInt32Property(container, vr::Prop_Axis1Type_Int32, vr::k_eControllerAxis_Trigger);
    vr::VRProperties()->SetUint64Property(
        container, vr::Prop_SupportedButtons_Uint64,
        vr::ButtonMaskFromId(vr::k_EButton_System) | vr::ButtonMaskFromId(vr::k_EButton_ApplicationMenu) |
            vr::ButtonMaskFromId(vr::k_EButton_Grip) | vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Touchpad) |
            vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Trigger));
    // The headset's space.
    vr::VRProperties()->SetUint64Property(container, vr::Prop_CurrentUniverseId_Uint64, kUniverseId);

    vr::IVRDriverInput* input = vr::VRDriverInput();
    input->CreateBooleanComponent(container, "/input/system/click", &system_);
    input->CreateBooleanComponent(container, "/input/application_menu/click", &menu_);
    input->CreateBooleanComponent(container, "/input/grip/click", &grip_);
    input->CreateBooleanComponent(container, "/input/trigger/click", &trigger_click_);
    input->CreateScalarComponent(container, "/input/trigger/value", &trigger_value_, vr::VRScalarType_Absolute,
                                 vr::VRScalarUnits_NormalizedOneSided);
    input->CreateScalarComponent(container, "/input/trackpad/x", &pad_x_, vr::VRScalarType_Absolute,
                                 vr::VRScalarUnits_NormalizedTwoSided);
    input->CreateScalarComponent(container, "/input/trackpad/y", &pad_y_, vr::VRScalarType_Absolute,
                                 vr::VRScalarUnits_NormalizedTwoSided);
    input->CreateBooleanComponent(container, "/input/trackpad/click", &pad_click_);
    input->CreateBooleanComponent(container, "/input/trackpad/touch", &pad_touch_);
    input->CreateHapticComponent(container, "/output/haptic", &haptic_);
    inputs_ready_ = true;
    device_index_ = object_id;
    return vr::VRInitError_None;
}

void MoveController::Deactivate()
{
    device_index_ = vr::k_unTrackedDeviceIndexInvalid;
}

void MoveController::DebugRequest(const char*, char* response_buffer, uint32_t response_buffer_size)
{
    if (response_buffer_size >= 1) {
        response_buffer[0] = 0;
    }
}

vr::DriverPose_t MoveController::GetPose()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return pose_;
}

void MoveController::Update(bool connected, const HandPose& hand, uint32_t buttons, float trigger)
{
    vr::DriverPose_t pose{};
    pose.qWorldFromDriverRotation.w = 1.0;
    pose.qDriverFromHeadRotation.w = 1.0;
    pose.deviceIsConnected = connected;
    pose.poseIsValid = hand.valid;
    // Held where it was, it is still shown and still turns, as VDM's are.
    pose.result = hand.valid ? vr::TrackingResult_Running_OK : vr::TrackingResult_Running_OutOfRange;
    pose.vecPosition[0] = hand.position.x;
    pose.vecPosition[1] = hand.position.y;
    pose.vecPosition[2] = hand.position.z;
    pose.vecVelocity[0] = hand.velocity.x;
    pose.vecVelocity[1] = hand.velocity.y;
    pose.vecVelocity[2] = hand.velocity.z;
    const Quat q = hand.valid ? hand.orientation : Quat{};
    pose.qRotation.w = q.w;
    pose.qRotation.x = q.x;
    pose.qRotation.y = q.y;
    pose.qRotation.z = q.z;
    pose.vecAngularVelocity[0] = hand.angular_velocity.x;
    pose.vecAngularVelocity[1] = hand.angular_velocity.y;
    pose.vecAngularVelocity[2] = hand.angular_velocity.z;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pose_ = pose;
    }
    const uint32_t index = device_index_;
    if (index != vr::k_unTrackedDeviceIndexInvalid) {
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(index, pose, sizeof(vr::DriverPose_t));
        UpdateInputs(buttons, trigger, q);
    }
}

void MoveController::UpdateInputs(uint32_t buttons, float trigger, Quat orientation)
{
    const auto down = [buttons](uint32_t bits) { return (buttons & bits) != 0; };
    const bool tilting = down(kMoveMove);
    if (tilting && !(have_inputs_ && (last_buttons_ & kMoveMove))) {
        tilt_from_ = orientation;
    }
    if (!inputs_ready_ || (have_inputs_ && buttons == last_buttons_ && trigger == last_trigger_ && !tilting)) {
        return;
    }
    have_inputs_ = true;
    last_buttons_ = buttons;
    last_trigger_ = trigger;
    vr::IVRDriverInput* input = vr::VRDriverInput();
    input->UpdateBooleanComponent(system_, down(kMovePS), 0.0);
    input->UpdateBooleanComponent(menu_, down(kMoveSelect), 0.0);
    input->UpdateBooleanComponent(grip_, down(kMoveStart), 0.0);
    input->UpdateBooleanComponent(trigger_click_, down(kMoveTrigger) || trigger >= kTriggerClick, 0.0);
    input->UpdateScalarComponent(trigger_value_, trigger, 0.0);
    // The face buttons as the trackpad's edges, clicked; two together, its diagonal.
    double x = (down(kMoveCircle) ? 1.0 : 0.0) - (down(kMoveSquare) ? 1.0 : 0.0);
    double y = (down(kMoveTriangle) ? 1.0 : 0.0) - (down(kMoveCross) ? 1.0 : 0.0);
    const bool face = x != 0.0 || y != 0.0;
    if (!face && tilting) {
        // Where it points now, seen from where it pointed: right and up move the touch.
        const Vec3 forward = Rotate(Conjugate(tilt_from_) * orientation, Vec3{0.0, 0.0, -1.0});
        x = std::atan2(forward.x, -forward.z) / kFullTiltRadians;
        y = std::atan2(forward.y, std::hypot(forward.x, forward.z)) / kFullTiltRadians;
    }
    const double length = std::hypot(x, y);
    if (length > 1.0) {
        x /= length;
        y /= length;
    }
    input->UpdateScalarComponent(pad_x_, static_cast<float>(x), 0.0);
    input->UpdateScalarComponent(pad_y_, static_cast<float>(y), 0.0);
    input->UpdateBooleanComponent(pad_touch_, face || tilting, 0.0);
    input->UpdateBooleanComponent(pad_click_, face, 0.0);
}

bool MoveController::OnHaptic(const vr::VREvent_HapticVibration_t& haptic, double now)
{
    if (!inputs_ready_ || haptic.componentHandle != haptic_) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    rumble_until_ = now + std::max(static_cast<double>(haptic.fDurationSeconds), kMinPulseSeconds);
    rumble_amplitude_ = std::clamp(haptic.fAmplitude, 0.0f, 1.0f);
    return true;
}

float MoveController::Rumble(double now)
{
    std::lock_guard<std::mutex> lock(mutex_);
    return now < rumble_until_ ? rumble_amplitude_ : 0.0f;
}

// ---------------------------------------------------------------------------
// MoveControllers
// ---------------------------------------------------------------------------

MoveControllers::MoveControllers(PsmsClient* psms, const HeadPositionConfig& bulb, const HandConfig& hand)
    : psms_(psms), bulb_config_(bulb), hand_config_(hand),
      bulbs_{HeadPositionEstimator(bulb), HeadPositionEstimator(bulb)}, hands_{MoveHand(true, hand), MoveHand(false, hand)}
{
    devices_[0] = std::make_unique<MoveController>("SMARTPHONE-HMD-MOVE-LEFT", true);
    devices_[1] = std::make_unique<MoveController>("SMARTPHONE-HMD-MOVE-RIGHT", false);
}

MoveControllers::~MoveControllers()
{
    Stop();
}

void MoveControllers::Start()
{
    for (int h = 0; h < 2; ++h) {
        added_[h] = vr::VRServerDriverHost()->TrackedDeviceAdded(devices_[h]->Serial().c_str(),
                                                                 vr::TrackedDeviceClass_Controller, devices_[h].get());
        if (!added_[h]) {
            DriverLog("Hands: could not add the %s controller", kHandNames[h]);
        }
    }
    running_ = true;
    thread_ = std::thread(&MoveControllers::Run, this);
}

void MoveControllers::Stop()
{
    running_ = false;
    if (thread_.joinable()) {
        thread_.join();
    }
}

void MoveControllers::OnEvent(const vr::VREvent_t& event)
{
    if (event.eventType != vr::VREvent_Input_HapticVibration) {
        return;
    }
    const double now = MonoSeconds();
    for (const auto& device : devices_) {
        if (device->OnHaptic(event.data.hapticVibration, now)) {
            return;
        }
    }
}

void MoveControllers::Run()
{
    std::vector<MoveState> moves;
    double next_report = MonoSeconds() + kReportSeconds;
    while (running_) {
        const double now = MonoSeconds();
        psms_->LatestMoves(&moves);
        std::sort(moves.begin(), moves.end(), [](const MoveState& a, const MoveState& b) { return a.id < b.id; });
        // Which Move in which hand.
        int ids[2] = {-1, -1};
        for (const MoveState& move : moves) {
            if ((move.hand == 1 || move.hand == 2) && ids[move.hand - 1] < 0) {
                ids[move.hand - 1] = move.id;
            }
        }
        for (const MoveState& move : moves) {
            if (move.id != ids[0] && move.id != ids[1] && (ids[1] < 0 || ids[0] < 0)) {
                ids[ids[1] < 0 ? 1 : 0] = move.id;
            }
        }

        const Headset headset = ReadHeadset();
        Vec3 camera;
        int cameras = 0;
        const bool camera_valid = psms_->Camera(&camera, &cameras);
        for (int h = 0; h < 2; ++h) {
            if (ids[h] != ids_[h]) {
                if (ids_[h] >= 0 && rumble_[h] > 0.0f) {
                    psms_->SetRumble(ids_[h], 0.0f);
                }
                if (ids[h] >= 0) {
                    DriverLog("Hands: PSMoveServiceEx's Move %d is the %s hand", ids[h], kHandNames[h]);
                } else {
                    DriverLog("Hands: no Move for the %s hand", kHandNames[h]);
                }
                ids_[h] = ids[h];
                hands_[h] = MoveHand(h == 0, hand_config_);
                bulbs_[h] = HeadPositionEstimator(bulb_config_);
                rumble_[h] = 0.0f;
                recenter_since_[h] = -1.0;
            }
            const auto move = std::find_if(moves.begin(), moves.end(),
                                           [&](const MoveState& m) { return m.id == ids[h]; });
            if (move == moves.end()) {
                devices_[h]->Update(false, HandPose{}, 0, 0.0f);
                continue;
            }
            // The bulb, filtered; seen while the filter follows it.
            HeadPositionEstimator& bulb = bulbs_[h];
            bulb.SetCamera(camera_valid, camera, cameras);
            bulb.AddLight(move->time, move->position, move->tracking, Vec3{});
            bulb.Tick(now, true);
            MoveState filtered = *move;
            filtered.tracking = bulb.state() == HeadPositionEstimator::State::Live;
            if (filtered.tracking) {
                const HeadPoseOut out = bulb.Output(now, Quat{}, Vec3{});
                filtered.position = out.position;
                filtered.velocity = out.velocity;
            }
            hands_[h].Update(filtered, now, headset);
            devices_[h]->Update(true, hands_[h].Pose(), move->buttons, move->trigger);
            CheckRecenter(h, *move, now);
            const float rumble = devices_[h]->Rumble(now);
            if (rumble != rumble_[h]) {
                psms_->SetRumble(move->id, rumble);
                rumble_[h] = rumble;
            }
        }

        if (now >= next_report) {
            next_report = now + kReportSeconds;
            LogReport();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kLoopMilliseconds));
    }
}

Headset MoveControllers::ReadHeadset()
{
    Headset headset;
    vr::TrackedDevicePose_t head{};
    vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0.0f, &head, 1);
    if (!head.bPoseIsValid) {
        return headset;
    }
    const vr::HmdMatrix34_t& m = head.mDeviceToAbsoluteTracking;
    const Vec3 looks{-m.m[0][2], 0.0, -m.m[2][2]};
    if (Length(looks) > 0.2) {  // not looking straight up or down
        forward_ = Unit(looks);
    }
    headset.valid = true;
    headset.eyes = Vec3{m.m[0][3], m.m[1][3], m.m[2][3]};
    headset.forward = forward_;
    return headset;
}

void MoveControllers::CheckRecenter(int hand, const MoveState& move, double now)
{
    if ((move.buttons & (kMoveSelect | kMoveStart)) != (kMoveSelect | kMoveStart)) {
        recenter_since_[hand] = -1.0;
        return;
    }
    if (recenter_since_[hand] < 0.0) {
        recenter_since_[hand] = now;
    }
    if (now - recenter_since_[hand] < kRecenterHoldSeconds) {
        return;
    }
    recenter_since_[hand] = now + 1e9;  // once per hold
    // Pointing at the PS Eye, level, its top up; straight ahead if the camera is not known.
    Vec3 camera, toward{0.0, 0.0, -1.0};
    if (psms_->Camera(&camera) && Length(camera - move.position) > 0.3) {
        toward = Unit(camera - move.position);
    }
    const double yaw = std::atan2(-toward.x, -toward.z);
    const double pitch = std::asin(std::clamp(toward.y, -1.0, 1.0));
    const Quat controller = AxisAngle(Vec3{0, 1, 0}, yaw) * AxisAngle(Vec3{1, 0, 0}, pitch);
    psms_->ResetOrientation(move.id, hands_[hand].MoveOrientation(controller));
    DriverLog("Hands: the %s Move's rotation reset, pointing at the PS Eye (Select and Start held)", kHandNames[hand]);
}

void MoveControllers::LogReport()
{
    std::string text = "Hands:";
    for (int h = 0; h < 2; ++h) {
        const MoveHand::Stats stats = hands_[h].TakeStats();
        const HeadPositionEstimator::Stats bulb = bulbs_[h].TakeStats();
        const int total = stats.tracking + stats.at_rest + stats.held;
        char part[640];
        if (ids_[h] < 0 || total == 0) {
            std::snprintf(part, sizeof(part), "%s %s no Move", h > 0 ? ";" : "", kHandNames[h]);
        } else {
            char still[200] = "never held still";
            if (stats.still > 0) {
                const double seconds = std::max(stats.still_seconds, 1e-3);
                std::snprintf(still, sizeof(still),
                              "held still %.0f%% of the time: its bulb shook up to %.0f mm and its rotation %.1f "
                              "degrees, the controller drifted %.1f mm and %.2f degrees a second",
                              100.0 * stats.still / total, stats.shake_m * 1000.0, stats.shake_degrees,
                              stats.drift_m * 1000.0 / seconds, stats.drift_degrees / seconds);
            }
            std::snprintf(part, sizeof(part),
                          "%s %s Move %d, %.0f%% of the time on its bulb, %.0f%% in front of the headset, %.0f%% held "
                          "where it was (%d jumps not believed, %d glimpses of it left out, %d bulb positions out of "
                          "reach, %d bulb readings left out, %d resets); on its bulb, on average %.0f cm to the right "
                          "of the eyes, %.0f cm in front and %.0f cm below, and across the floor up to %.0f cm from "
                          "them; %s; the PS Eye's depth of it shakes %.0f mm and sideways %.0f, agility %.2f",
                          h > 0 ? ";" : "", kHandNames[h], ids_[h], 100.0 * stats.tracking / total,
                          100.0 * stats.at_rest / total, 100.0 * stats.held / total, stats.jumps, stats.glimpses,
                          stats.out_of_reach, bulb.rejected + bulb.implausible, bulb.resets, stats.offset.x * 100.0,
                          stats.offset.z * 100.0, -stats.offset.y * 100.0, stats.farthest * 100.0, still,
                          bulb.depth_innovation_rms_m * 1000.0, bulb.lateral_innovation_rms_m * 1000.0, bulb.agility);
        }
        text += part;
        if (stats.out_of_reach > stats.tracking && stats.out_of_reach > 100) {
            DriverLog("Hands: the PS Eye mostly puts the %s bulb farther from the headset than an arm reaches: is the "
                      "head light's distance right (Ctrl+Alt+L)? Otherwise raise controller_reach_cm",
                      kHandNames[h]);
        }
    }
    DriverLog("%s", text.c_str());
}
