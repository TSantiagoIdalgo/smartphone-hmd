#include "move_hand.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kLearnSeconds = 1.0;        // how long the learned offset takes to follow
constexpr double kMaxLearnSpeed = 0.3;       // m/s: faster, the cameras' later wrist would show
constexpr int kMinSamples = 30;              // before a wrist's offset counts
constexpr double kMaxBulbFromWristM = 0.35;  // farther on average, that wrist is not holding it
constexpr double kSwitchMarginM = 0.08;      // the other wrist must be this much closer to take over
constexpr double kBlendSeconds = 0.15;
constexpr double kHalfTurn = 3.14159265358979;
constexpr double kMaxBulbSpeed = 12.0;      // m/s; a hand swung hard reaches about half that
constexpr double kMinFrameSeconds = 1.0 / 30.0;  // PSMoveServiceEx's states repeat between camera frames
constexpr double kJumpSpreadM = 0.15;       // positions after a jump this close together: one place
constexpr double kJumpTrustSeconds = 0.3;   // staying there this long, it is believed

// Along the Move from the wrist to the bulb, in the controller's axes, until learned.
const Vec3 kDefaultBulbFromWrist{0.0, 0.0, -0.16};

}  // namespace

// PSMoveServiceEx's Move points along -z at identity with its buttons up, as SteamVR's controllers.
MoveHand::MoveHand(bool left, double pitch_degrees)
    : move_from_controller_(AxisAngle(Vec3{1.0, 0.0, 0.0}, pitch_degrees * kHalfTurn / 180.0)), wrist_(left ? 0 : 1)
{
    offset_[0] = offset_[1] = kDefaultBulbFromWrist;
}

void MoveHand::Update(const MoveState& move, const Wrists& wrists, double now)
{
    const Quat orientation = Normalize(move.orientation * move_from_controller_);
    const HandPose before = Pose(now);
    HandPose pose;
    pose.orientation = orientation;
    pose.angular_velocity = move.angular_velocity;

    bool seen = move.tracking;
    if (seen && have_bulb_ &&
        Length(move.position - bulb_) > kMaxBulbSpeed * std::max(move.time - bulb_time_, kMinFrameSeconds)) {
        if (!jumping_ || Length(move.position - jump_last_) > kJumpSpreadM) {
            jumping_ = true;
            jump_since_ = now;
        }
        jump_last_ = move.position;
        seen = now - jump_since_ >= kJumpTrustSeconds;
        stats_.jumps += !seen;
    }
    if (seen) {
        jumping_ = false;
        bulb_ = move.position;
        bulb_time_ = move.time;
        have_bulb_ = true;
    }

    if (seen) {
        const double dt = learned_at_ >= 0.0 ? std::clamp(now - learned_at_, 0.0, 0.1) : 0.0;
        learned_at_ = now;
        if (Length(move.velocity) < kMaxLearnSpeed) {
            for (int w = 0; w < 2; ++w) {
                if (!wrists.valid[w]) {
                    continue;
                }
                const Vec3 bulb_from_wrist = move.position - wrists.position[w];
                const double rate = std::max(1.0 / (samples_[w] + 1), 1.0 - std::exp(-dt / kLearnSeconds));
                offset_[w] = offset_[w] + (Rotate(Conjugate(orientation), bulb_from_wrist) - offset_[w]) * rate;
                distance_[w] += (Length(bulb_from_wrist) - distance_[w]) * rate;
                ++samples_[w];
            }
            const int other = 1 - wrist_;
            if (samples_[other] >= kMinSamples &&
                (samples_[wrist_] < kMinSamples || distance_[other] + kSwitchMarginM < distance_[wrist_])) {
                wrist_ = other;
            }
        }
        pose.valid = true;
        pose.tracking = true;
        pose.position = move.position;
        pose.velocity = move.velocity;
        ++stats_.tracking;
    } else if (wrists.valid[wrist_] && (samples_[wrist_] < kMinSamples || distance_[wrist_] < kMaxBulbFromWristM)) {
        pose.valid = true;
        pose.from_wrist = true;
        pose.position = wrists.position[wrist_] + Rotate(orientation, offset_[wrist_]);
        pose.velocity = wrists.velocity[wrist_];
        ++stats_.from_wrist;
    } else {
        // Where it was, turning in place.
        pose.valid = pose_.valid;
        pose.position = pose_.position;
        ++stats_.held;
    }

    if (pose_.valid && (pose.tracking != pose_.tracking || pose.from_wrist != pose_.from_wrist)) {
        blend_ = before.position - pose.position;
        blend_time_ = now;
    }
    pose_ = pose;
}

HandPose MoveHand::Pose(double now) const
{
    HandPose pose = pose_;
    pose.position = pose.position + blend_ * std::exp(-std::max(now - blend_time_, 0.0) / kBlendSeconds);
    return pose;
}

MoveHand::Stats MoveHand::TakeStats()
{
    const Stats stats = stats_;
    stats_ = Stats{};
    return stats;
}
