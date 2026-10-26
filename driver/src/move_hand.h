#pragma once

#include <cstdint>
#include <mutex>

#include "math3d.h"

// A PS Move held in one hand, as SteamVR should see it: where PSMoveServiceEx sees its bulb
// while the PS Eye sees it, and while it does not (behind the body, out of the camera's view,
// hidden by the other hand) where the body cameras see the wrist holding it, with the bulb where
// it has been from that wrist. The rotation always comes from the Move itself.
//
// Which wrist holds it, and where the bulb sits from that wrist in the Move's own axes, are
// learned while both are seen. Changing from one to the other glides over a moment.
//
// A bulb partly hidden (behind the other hand, at the edge of the picture) looks smaller to the
// PS Eye, so PSMoveServiceEx puts it much farther away while still calling it tracked: a jump
// faster than any hand is not believed until the bulb stays there a moment.
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
    uint32_t buttons = 0;   // MoveButton bits
    float trigger = 0.0f;   // 0 to 1
};

// The wrists as the body cameras see them, now.
struct Wrists {
    bool valid[2] = {false, false};  // the person's left, right
    Vec3 position[2];
    Vec3 velocity[2];
};

// Hands the newest wrists from the body tracking's thread to the controllers'.
class SharedWrists {
public:
    void Set(const Wrists& wrists)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        wrists_ = wrists;
    }
    Wrists Get()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return wrists_;
    }

private:
    std::mutex mutex_;
    Wrists wrists_;
};

struct HandPose {
    bool valid = false;
    bool tracking = false;    // from the bulb
    bool from_wrist = false;  // from the wrist; neither: held where it was
    Vec3 position;
    Quat orientation;  // SteamVR's controller axes: -z where it points, +y its top
    Vec3 velocity;
    Vec3 angular_velocity;
};

class MoveHand {
public:
    struct Stats {
        int tracking = 0;  // updates from the bulb
        int from_wrist = 0;
        int held = 0;
        int jumps = 0;     // bulb positions not believed: farther than a hand moves
    };

    // `left`: the hand it starts with, and the wrist it guesses holds it. `pitch_degrees` tilts
    // PSMoveServiceEx's rotation about the Move's crosswise axis, should it not point where it is
    // held.
    explicit MoveHand(bool left, double pitch_degrees = 0.0);

    // The rotation PSMoveServiceEx would give a Move whose controller has `controller` (SteamVR's).
    Quat MoveOrientation(Quat controller) const { return Normalize(controller * Conjugate(move_from_controller_)); }

    // Each new state of the Move, with the wrists of the same moment.
    void Update(const MoveState& move, const Wrists& wrists, double now);
    HandPose Pose(double now) const;

    // The person's wrist that holds it: 0 left, 1 right.
    int wrist() const { return wrist_; }
    // Where the bulb sits from it, in the controller's axes.
    Vec3 BulbFromWrist() const { return offset_[wrist_]; }
    // Counters since the last call.
    Stats TakeStats();

private:
    Quat move_from_controller_;
    HandPose pose_;
    int wrist_;
    Vec3 offset_[2];
    double distance_[2] = {0.0, 0.0};  // between the bulb and each wrist, on average
    int samples_[2] = {0, 0};
    double learned_at_ = -1.0;
    // The bulb's last believed position, and when: a jump from it no hand makes is not believed.
    bool have_bulb_ = false;
    Vec3 bulb_;
    double bulb_time_ = 0.0;
    bool jumping_ = false;
    Vec3 jump_last_;
    double jump_since_ = 0.0;
    Vec3 blend_;
    double blend_time_ = 0.0;
    Stats stats_;
};
