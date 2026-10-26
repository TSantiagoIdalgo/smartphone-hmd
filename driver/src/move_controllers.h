#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "head_position.h"
#include "move_hand.h"
#include "openvr_driver.h"

class PsmsClient;

// A PS Move as SteamVR sees it: a Vive wand, whose bindings every game has, in the left or the
// right hand.
//   trigger: trigger                    Start: grip
//   Move (the big button) held: the trackpad touched where the Move tilts to from where it
//     pointed when pressed, as in VDM (VRChat's action menu, gestures)
//   Triangle, Cross, Square, Circle: the trackpad up, down, left, right, touched and clicked
//   Select: menu                        PS: system (SteamVR's dashboard)
// The haptic pulses games send become the Move's rumble. Select and Start held together for a
// second, pointing the Move at the PS Eye, undo the drift of its rotation.
class MoveController : public vr::ITrackedDeviceServerDriver {
public:
    MoveController(std::string serial, bool left);

    vr::EVRInitError Activate(uint32_t object_id) override;
    void Deactivate() override;
    void EnterStandby() override {}
    void* GetComponent(const char*) override { return nullptr; }
    void DebugRequest(const char*, char* response_buffer, uint32_t response_buffer_size) override;
    vr::DriverPose_t GetPose() override;

    // The controllers' thread. Not `connected`: no Move for this hand.
    void Update(bool connected, const HandPose& hand, uint32_t buttons, float trigger);
    // SteamVR's thread: a haptic pulse, if it is for this controller.
    bool OnHaptic(const vr::VREvent_HapticVibration_t& haptic, double now);
    // How strongly to rumble at `now`, 0 to 1.
    float Rumble(double now);
    const std::string& Serial() const { return serial_; }

private:
    void UpdateInputs(uint32_t buttons, float trigger, Quat orientation);

    std::string serial_;
    bool left_;
    std::atomic<uint32_t> device_index_{vr::k_unTrackedDeviceIndexInvalid};
    std::mutex mutex_;
    vr::DriverPose_t pose_{};
    double rumble_until_ = 0.0;
    float rumble_amplitude_ = 0.0f;

    // Set once in Activate, then read by the controllers' thread.
    std::atomic<bool> inputs_ready_{false};
    vr::VRInputComponentHandle_t system_ = vr::k_ulInvalidInputComponentHandle;
    vr::VRInputComponentHandle_t menu_ = vr::k_ulInvalidInputComponentHandle;
    vr::VRInputComponentHandle_t grip_ = vr::k_ulInvalidInputComponentHandle;
    vr::VRInputComponentHandle_t trigger_click_ = vr::k_ulInvalidInputComponentHandle;
    vr::VRInputComponentHandle_t trigger_value_ = vr::k_ulInvalidInputComponentHandle;
    vr::VRInputComponentHandle_t pad_x_ = vr::k_ulInvalidInputComponentHandle;
    vr::VRInputComponentHandle_t pad_y_ = vr::k_ulInvalidInputComponentHandle;
    vr::VRInputComponentHandle_t pad_click_ = vr::k_ulInvalidInputComponentHandle;
    vr::VRInputComponentHandle_t pad_touch_ = vr::k_ulInvalidInputComponentHandle;
    vr::VRInputComponentHandle_t haptic_ = vr::k_ulInvalidInputComponentHandle;
    // The controllers' thread only.
    bool have_inputs_ = false;
    uint32_t last_buttons_ = 0;
    float last_trigger_ = 0.0f;
    Quat tilt_from_;  // where the Move pointed when its button was pressed
};

// Both hands: follows the PS Moves from PSMoveServiceEx on its own thread and gives the
// controllers their poses, falling back on the wrists the body tracking sees (`wrists`, may be
// null). A Move set to the left or right hand in PSMoveServiceEx goes there; the others fill the
// right hand first, then the left.
//
// Each bulb goes through the head's light filter first: like the visor's light, the PS Eye sees it
// well sideways and poorly in depth, worse the farther it is, so it shakes; the filter smooths it
// more while the hand keeps still, and leaves out the jumps of a bulb half hidden. The body
// cameras' views of the bulbs (`sightings`, may be null) go into it as they do the head's: they
// give its depth, and follow it while the PS Eye does not see it. A view is the bulb of the hand
// nearest its line of sight, when the other hand is clearly farther.
class MoveControllers {
public:
    // `pitch_degrees`: see MoveHand. `bulb`: the filter's settings for a bulb.
    MoveControllers(PsmsClient* psms, SharedWrists* wrists, HeadSightingQueue* sightings, double pitch_degrees,
                    const HeadPositionConfig& bulb);
    ~MoveControllers();

    // Adds the controllers to SteamVR and starts following the Moves.
    void Start();
    void Stop();
    // SteamVR's events, from RunFrame.
    void OnEvent(const vr::VREvent_t& event);

private:
    void Run();
    // The hand whose bulb a camera saw, or -1.
    int Owner(const HeadSighting& sighting, double now) const;
    // Select and Start held: tells PSMoveServiceEx the Move points at the PS Eye.
    void CheckRecenter(int hand, const MoveState& move, double now);
    void LogReport();

    PsmsClient* psms_;
    SharedWrists* wrists_;
    HeadSightingQueue* sightings_;
    double pitch_degrees_;
    std::unique_ptr<MoveController> devices_[2];  // left, right
    bool added_[2] = {false, false};
    // The thread only.
    HeadPositionConfig bulb_config_;
    HeadPositionEstimator bulbs_[2];
    MoveHand hands_[2]{MoveHand(true), MoveHand(false)};
    int ids_[2] = {-1, -1};
    float rumble_[2] = {0.0f, 0.0f};
    double recenter_since_[2] = {-1.0, -1.0};

    std::atomic<bool> running_{false};
    std::thread thread_;
};
