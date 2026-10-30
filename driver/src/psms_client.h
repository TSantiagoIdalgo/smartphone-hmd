#pragma once

#include <atomic>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "math3d.h"
#include "move_hand.h"

// Reads the head light's position, and the PS Moves' for the hands, from PSMoveServiceEx, the
// service that tracks PS Move controllers with the PS Eye. It loads the
// PSMoveClient_CAPI.dll that ships with PSMoveServiceEx at runtime, so the driver still starts
// without it. Every call into the DLL happens on this class's own thread, because the DLL keeps a
// single global client. Windows only.
struct PsmsConfig {
    std::string dll_path;       // empty: look next to the running PSMoveServiceEx or VDM
    std::string host = "127.0.0.1";
    std::string port = "9512";
    bool follow_head = true;    // the head light (head_position_enabled)
    int hmd_id = -1;            // -1: the first virtual HMD
    int controller_id = -1;     // 0 or more: follow this controller (a Move on the visor) instead
    bool stream_controllers = false;  // every other Move (the hands)
};

struct LightSample {
    double time;    // MonoSeconds() when it arrived
    Vec3 position;  // metres: PSMoveServiceEx space / 100
    bool tracking;  // false when the camera lost it
};

class PsmsClient {
public:
    PsmsClient() = default;
    ~PsmsClient();
    PsmsClient(const PsmsClient&) = delete;
    PsmsClient& operator=(const PsmsClient&) = delete;

    void Start(const PsmsConfig& config);
    void Stop();

    // Moves the samples received since the last call into `samples`. Returns whether the light is
    // being streamed. The camera outputs describe the PS Eyes, in the same space as the samples.
    bool Drain(std::vector<LightSample>* samples, bool* camera_valid, Vec3* camera, int* cameras);
    // Each streamed Move as it is now, with stream_controllers.
    void LatestMoves(std::vector<MoveState>* moves);
    // Sets a Move's rumble, 0 to 1, until changed.
    void SetRumble(int id, float fraction);
    // Tells PSMoveServiceEx the rotation a Move is in now, undoing its drift.
    void ResetOrientation(int id, Quat orientation);
    // Where the first PS Eye is, if known, and how many there are.
    bool Camera(Vec3* position, int* cameras = nullptr);

private:
    struct Api;  // the DLL's functions

    void Run();
    void Stream(const Api& api);
    void Push(const LightSample& sample);
    void SetStreaming(bool streaming);
    void SetMoves(const std::vector<MoveState>& moves);
    float Rumble(int id);
    bool TakeOrientationReset(int id, Quat* orientation);
    void SetCameras(bool valid, Vec3 camera, int cameras);
    void LogOnce(const std::string& message);
    void SleepWhileRunning(int milliseconds);

    PsmsConfig config_;
    std::string prefix_;  // what its log lines start with
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::string last_message_;  // client thread only

    std::mutex mutex_;
    std::vector<LightSample> samples_;
    std::vector<MoveState> moves_;
    std::map<int, float> rumble_;
    std::map<int, Quat> orientation_resets_;
    bool streaming_ = false;
    bool camera_valid_ = false;
    Vec3 camera_;
    int cameras_ = 0;
};
