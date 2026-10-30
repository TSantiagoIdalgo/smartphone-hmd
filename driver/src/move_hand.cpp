#include "move_hand.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kBlendSeconds = 0.2;
constexpr double kRestBlendSeconds = 0.4;   // to and from the rest in front of the headset, farther
constexpr double kHalfTurn = 3.14159265358979;
constexpr double kMaxBulbSpeed = 12.0;      // m/s; a hand swung hard reaches about half that
constexpr double kMinFrameSeconds = 1.0 / 30.0;  // PSMoveServiceEx's states repeat between camera frames
constexpr double kJumpSpreadM = 0.15;       // positions after a jump this close together: one place
constexpr double kJumpTrustSeconds = 0.3;   // staying there this long, it is believed
constexpr double kRestAfterSeconds = 0.3;   // lost this long (not just hidden a moment), it goes to rest
// How much the hand moves: the Move's gyro over the last moment, which the hand's own tremor (about
// 10 Hz) and the gyro's noise average out of, and the bulb's speed, which the PS Eye's depth noise
// still shakes a few cm/s. Below the first value of each it keeps still, above the second it moves.
constexpr double kSpinSeconds = 0.05;
constexpr double kStillSpin = 0.05;  // rad/s: 3 degrees a second
constexpr double kMovingSpin = 0.5;
constexpr double kStillSpeed = 0.05;  // m/s
constexpr double kMovingSpeed = 0.3;
// Kept still, what shaking still goes beyond the radius is smoothed over this long.
constexpr double kStillSeconds = 0.5;

double Angle(Quat a, Quat b)
{
    const double cosine = std::fabs(a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z);
    return 2.0 * std::acos(std::min(cosine, 1.0));
}

}  // namespace

// PSMoveServiceEx's Move points along -z at identity with its buttons up, as SteamVR's controllers.
MoveHand::MoveHand(bool left, const HandConfig& config)
    : left_(left), config_(config),
      move_from_controller_(AxisAngle(Vec3{1.0, 0.0, 0.0}, config.pitch_degrees * kHalfTurn / 180.0))
{
}

void MoveHand::Update(const MoveState& move, double now, const Headset& headset)
{
    const double dt = last_update_ >= 0.0 ? std::clamp(now - last_update_, 0.0, 0.1) : 0.0;
    last_update_ = now;
    const Quat orientation = Normalize(move.orientation * move_from_controller_);
    HandPose pose;
    pose.orientation = orientation;

    bool seen = move.tracking;
    if (seen && config_.reach_m > 0.0 && headset.valid && Length(move.position - headset.eyes) > config_.reach_m) {
        seen = false;
        ++stats_.out_of_reach;
    }
    bool jumped = false;  // to where it stayed long enough to be believed
    if (seen && have_bulb_ &&
        Length(move.position - bulb_) > kMaxBulbSpeed * std::max(move.time - bulb_time_, kMinFrameSeconds)) {
        if (!jumping_ || Length(move.position - jump_last_) > kJumpSpreadM) {
            jumping_ = true;
            jump_since_ = now;
        }
        jump_last_ = move.position;
        seen = jumped = now - jump_since_ >= kJumpTrustSeconds;
        stats_.jumps += !seen;
    }
    // Lost a while, it goes back to the bulb only once the PS Eye has seen it for a moment.
    if (seen && now - followed_at_ >= kRestAfterSeconds) {
        if (seen_since_ < 0.0) {
            seen_since_ = now;
        }
        seen = now - seen_since_ >= config_.found_seconds;
    } else if (!seen && seen_since_ >= 0.0) {
        ++stats_.glimpses;
        seen_since_ = -1.0;
    }

    if (seen) {
        seen_since_ = -1.0;
        jumping_ = false;
        bulb_ = move.position;
        bulb_time_ = move.time;
        have_bulb_ = true;
        followed_at_ = now;
        pose.valid = true;
        pose.tracking = true;
        pose.position = move.position;
        pose.velocity = move.velocity;
        ++stats_.tracking;
        if (headset.valid) {
            stats_.farthest = std::max(stats_.farthest, Length(move.position - headset.eyes));
        }
    } else if (config_.rest_enabled && headset.valid && now - followed_at_ >= kRestAfterSeconds) {
        const Vec3 right = Cross(headset.forward, Vec3{0.0, 1.0, 0.0});
        pose.valid = true;
        pose.at_rest = true;
        pose.position = headset.eyes + Vec3{0.0, -config_.rest_below_m, 0.0} +
                        headset.forward * config_.rest_forward_m + right * (left_ ? -config_.rest_side_m : config_.rest_side_m);
        ++stats_.at_rest;
    } else {
        // Where it was, turning in place.
        pose.valid = pose_.valid;
        pose.position = pose_.position;
        ++stats_.held;
    }

    // Changing from one to the other, or to where the bulb jumped, glides from where the controller
    // was shown.
    if (out_.valid && pose.valid && (pose.tracking != pose_.tracking || pose.at_rest != pose_.at_rest || jumped)) {
        blend_ = out_.position - pose.position;
        blend_time_ = now;
        blend_seconds_ = pose.at_rest || pose_.at_rest ? kRestBlendSeconds : kBlendSeconds;
    }
    pose_ = pose;
    const double glided = std::clamp((now - blend_time_) / blend_seconds_, 0.0, 1.0);
    const Vec3 target = pose.position + blend_ * (1.0 - glided * glided * (3.0 - 2.0 * glided));

    // Keeping still: within the radius, the controller stays where it is; beyond, it is dragged
    // along at that distance, slowly. Moving, the radius and the delay shrink to nothing and the
    // controller catches up. Gliding, or at rest with the headset, it goes as it is.
    spin_ = spin_ + (move.angular_velocity - spin_) * (1.0 - std::exp(-dt / kSpinSeconds));
    const double turning = std::clamp((Length(spin_) - kStillSpin) / (kMovingSpin - kStillSpin), 0.0, 1.0);
    const double moving = pose.at_rest || glided < 1.0
                              ? 1.0
                              : std::max(turning, std::clamp((Length(pose.velocity) - kStillSpeed) /
                                                                 (kMovingSpeed - kStillSpeed),
                                                             0.0, 1.0));
    // How much of the way to where it is dragged it stays behind, this update.
    const auto stay = [dt](double factor) {
        return factor < 1.0 ? std::exp(-dt / (kStillSeconds * (1.0 - factor))) : 0.0;
    };
    Vec3 position = target;
    Quat rotation = orientation;
    if (out_.valid && pose.valid) {
        const double radius = config_.still_mm / 1000.0 * (1.0 - moving);
        const double off = Length(out_.position - target);
        const Vec3 dragged = off > radius ? target + (out_.position - target) * (radius / off) : out_.position;
        position = dragged + (out_.position - dragged) * stay(moving);
        const double turn_radius = config_.still_degrees * kHalfTurn / 180.0 * (1.0 - turning);
        const double turn = Angle(out_.orientation, orientation);
        const Quat turned =
            turn > turn_radius ? Slerp(orientation, out_.orientation, turn_radius / turn) : out_.orientation;
        rotation = Slerp(turned, out_.orientation, stay(turning));
        if (moving == 0.0) {
            ++stats_.still;
            stats_.still_seconds += dt;
            stats_.shake_m = std::max(stats_.shake_m, off);
            stats_.shake_degrees = std::max(stats_.shake_degrees, turn * 180.0 / kHalfTurn);
            stats_.drift_m += Length(position - out_.position);
            stats_.drift_degrees += Angle(rotation, out_.orientation) * 180.0 / kHalfTurn;
        }
    }
    out_ = pose;
    out_.position = position;
    out_.orientation = rotation;
    // SteamVR carries the controller on by these until the next update: still, it stays.
    out_.velocity = pose.velocity * moving;
    out_.angular_velocity = move.angular_velocity * turning;
}

MoveHand::Stats MoveHand::TakeStats()
{
    const Stats stats = stats_;
    stats_ = Stats{};
    return stats;
}
