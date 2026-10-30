#pragma once

#include <cstdint>

#include "math3d.h"

// A PS Move held in one hand, as SteamVR should see it: where PSMoveServiceEx sees its bulb
// while the PS Eye sees it; while it does not (behind the body, out of the camera's view, hidden
// by the other hand), a moment where it was, then where it rests in front of the headset. The
// rotation always comes from the Move itself.
//
// What the PS Eye sees is not always the bulb's place:
// - A bulb partly hidden (behind the other hand, at the edge of the picture) looks smaller, so
//   PSMoveServiceEx puts it much farther away while still calling it tracked: a jump faster than
//   any hand is not believed until the bulb stays there a moment.
// - Turned away from the PS Eye, it glimpses a sliver of a bulb now and then: a lost Move goes
//   back to its bulb only once the PS Eye has seen it for a moment, and never to one farther from
//   the headset, across the floor, than an arm reaches. Only across the floor: how high the headset
//   is depends on how PSMoveServiceEx has the PS Eye tilted, and on where the light sits on it.
//
// Held still, the PS Eye's depth and PSMoveServiceEx's rotation still shake by some mm and a
// fraction of a degree, which a pointer at a menu makes plain. While the Move's gyro says the hand
// keeps still, the controller stays put until the bulb or the rotation goes farther than that
// shaking, and then follows slowly; as soon as the hand moves, it follows as it is.
//
// World: the head's, SteamVR raw space in metres. Times: MonoSeconds().

enum MoveButton : uint32_t {
    kMoveSquare = 1u << 0,
    kMoveTriangle = 1u << 1,
    kMoveCross = 1u << 2,
    kMoveCircle = 1u << 3,
    kMoveMove = 1u << 4,
    kMovePS = 1u << 5,
    kMoveStart = 1u << 6,
    kMoveSelect = 1u << 7,
    kMoveTrigger = 1u << 8,
};

// What PSMoveServiceEx says about one Move.
struct MoveState {
    int id = -1;
    int hand = 0;           // as set in PSMoveServiceEx: 0 either, 1 left, 2 right
    double time = 0.0;      // when it arrived
    bool tracking = false;  // the PS Eye sees the bulb
    Vec3 position;          // the bulb's centre: PSMoveServiceEx space / 100
    Quat orientation;       // PSMoveServiceEx's: identity pointing along -z, buttons up
    Vec3 velocity;          // m/s
    Vec3 angular_velocity;  // rad/s, world
    // The Move's own sensors, in its axes, as PSMoveServiceEx calibrates them: the accelerometer
    // (gravity included) and the gyroscope (rad/s), and when PSMoveServiceEx read them, s.
    Vec3 accel;
    Vec3 gyro;
    double sensor_time = 0.0;
    uint32_t buttons = 0;   // MoveButton bits
    float trigger = 0.0f;   // 0 to 1
};

struct HandConfig {
    // Tilts PSMoveServiceEx's rotation about the Move's crosswise axis, should it not point where
    // it is held.
    double pitch_degrees = 0.0;
    // Where a lost Move's hand goes: in front of the headset, turning with it but not tilting.
    bool rest_enabled = true;
    double rest_below_m = 0.5;    // under the eyes
    double rest_forward_m = 0.3;  // in front of them
    double rest_side_m = 0.2;     // each hand to its side
    double reach_m = 1.2;         // a bulb farther from the eyes, across the floor, is not the hand's; 0: any
    double found_seconds = 0.3;   // lost a while, seen this long again before it is believed
    double still_mm = 15.0;       // held still, shaking this small is left out
    double still_degrees = 0.6;
    bool yaw_from_motion = true;  // undo the drift of the Move's rotation about the vertical (MoveYaw)
};

// The headset as SteamVR has it now.
struct Headset {
    bool valid = false;
    Vec3 eyes;
    Vec3 forward;  // where it looks, level: unit
};

struct HandPose {
    bool valid = false;
    bool tracking = false;  // from the bulb
    bool at_rest = false;   // in front of the headset; neither: held where it was
    Vec3 position;
    Quat orientation;  // SteamVR's controller axes: -z where it points, +y its top
    Vec3 velocity;
    Vec3 angular_velocity;
};

class MoveHand {
public:
    struct Stats {
        int tracking = 0;  // updates from the bulb
        int at_rest = 0;
        int held = 0;
        int jumps = 0;        // bulb positions not believed: farther than a hand moves
        int glimpses = 0;     // the bulb seen too briefly, after it was lost, to be believed
        int out_of_reach = 0;  // bulb positions farther from the headset than an arm reaches
        double farthest = 0.0;  // the farthest believed bulb from the eyes across the floor, m
        int placed = 0;         // updates on the bulb with the headset known, and on average in them
        Vec3 offset;            // the bulb from the eyes: x to their right, y up, z forward, m
        int still = 0;  // updates with the hand held still; meanwhile:
        double still_seconds = 0.0;
        double shake_m = 0.0;  // the most the bulb went from the controller
        double shake_degrees = 0.0;
        double drift_m = 0.0;  // how far the controller went all the same
        double drift_degrees = 0.0;
    };

    explicit MoveHand(bool left, const HandConfig& config = HandConfig{});

    // The rotation PSMoveServiceEx would give a Move whose controller has `controller` (SteamVR's).
    Quat MoveOrientation(Quat controller) const { return Normalize(controller * Conjugate(move_from_controller_)); }

    // Each new state of the Move, with the headset as it is now.
    void Update(const MoveState& move, double now, const Headset& headset = Headset{});
    // As SteamVR should have it, after the newest Update.
    const HandPose& Pose() const { return out_; }
    // Counters since the last call.
    Stats TakeStats();

private:
    bool left_;
    HandConfig config_;
    Quat move_from_controller_;
    HandPose pose_;  // before the blend and keeping still
    HandPose out_;
    double last_update_ = -1.0;
    // The bulb's last believed position, and when: a jump from it no hand makes is not believed.
    bool have_bulb_ = false;
    Vec3 bulb_;
    double bulb_time_ = 0.0;
    bool jumping_ = false;
    Vec3 jump_last_;
    double jump_since_ = 0.0;
    double followed_at_ = -1e9;  // when the bulb last said where it is
    double seen_since_ = -1.0;   // when the PS Eye started seeing it again, after losing it
    Vec3 blend_;  // from where it was shown, when it changed: gliding away to nothing
    double blend_time_ = -1e9;
    double blend_seconds_ = 0.2;
    Vec3 spin_;  // the Move's angular velocity, smoothed
    Stats stats_;
};
