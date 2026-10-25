#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "math3d.h"

// Reads the head light's position, and the PS Moves' for calibrating the body cameras, from
// PSMoveServiceEx, the service that tracks PS Move controllers with the PS Eye. It loads the
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
    bool stream_controllers = false;  // every other controller's position (body tracking)
};

struct LightSample {
    double time;    // MonoSeconds() when it arrived
    Vec3 position;  // metres: PSMoveServiceEx space / 100
    bool tracking;  // false when the camera lost it
};

struct ControllerSample {
    double time;    // MonoSeconds() when it arrived
    int id;         // PSMoveServiceEx's controller number
    Vec3 position;  // metres, as LightSample: the centre of the Move's bulb
    bool tracking;
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
    // The same for the controllers, with stream_controllers. Returns whether any is streamed.
    bool DrainControllers(std::vector<ControllerSample>* samples);

private:
    struct Api;  // the DLL's functions

    void Run();
    void Stream(const Api& api);
    void Push(const LightSample& sample);
    void PushController(const ControllerSample& sample);
    void SetStreaming(bool streaming);
    void SetControllersStreaming(bool streaming);
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
    std::vector<ControllerSample> controller_samples_;
    bool streaming_ = false;
    bool controllers_streaming_ = false;
    bool camera_valid_ = false;
    Vec3 camera_;
    int cameras_ = 0;
};
