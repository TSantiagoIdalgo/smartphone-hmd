#include "psms_client.h"

#include <chrono>
#include <cstring>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <mmsystem.h>
#include <tlhelp32.h>
#include <winsock2.h>
#include <ws2tcpip.h>

// Types only: the functions come from the DLL at runtime.
#define PSMoveClient_STATIC
#include "PSMoveClient_CAPI.h"
#endif

#include "driver_log.h"
#include "mono_clock.h"

namespace {

constexpr size_t kMaxQueuedSamples = 256;
constexpr size_t kMaxQueuedControllerSamples = 1024;
constexpr int kRetryMilliseconds = 2000;
constexpr double kDeviceRetrySeconds = 5.0;
constexpr double kCameraRetrySeconds = 1.0;
constexpr int kRequestTimeoutMs = 500;

}  // namespace

PsmsClient::~PsmsClient()
{
    Stop();
}

void PsmsClient::Start(const PsmsConfig& config)
{
    if (running_) {
        return;
    }
    config_ = config;
    prefix_ = config.follow_head ? "Head position: " : "Body tracking: ";
    running_ = true;
    thread_ = std::thread(&PsmsClient::Run, this);
}

void PsmsClient::Stop()
{
    running_ = false;
    if (thread_.joinable()) {
        thread_.join();
    }
}

bool PsmsClient::Drain(std::vector<LightSample>* samples, bool* camera_valid, Vec3* camera, int* cameras)
{
    std::lock_guard<std::mutex> lock(mutex_);
    samples->swap(samples_);
    samples_.clear();
    *camera_valid = camera_valid_;
    *camera = camera_;
    *cameras = cameras_;
    return streaming_;
}

bool PsmsClient::DrainControllers(std::vector<ControllerSample>* samples)
{
    std::lock_guard<std::mutex> lock(mutex_);
    samples->swap(controller_samples_);
    controller_samples_.clear();
    return controllers_streaming_;
}

void PsmsClient::PushController(const ControllerSample& sample)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (controller_samples_.size() >= kMaxQueuedControllerSamples) {
        controller_samples_.erase(controller_samples_.begin());
    }
    controller_samples_.push_back(sample);
}

void PsmsClient::SetControllersStreaming(bool streaming)
{
    std::lock_guard<std::mutex> lock(mutex_);
    controllers_streaming_ = streaming;
}

void PsmsClient::Push(const LightSample& sample)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (samples_.size() >= kMaxQueuedSamples) {
        samples_.erase(samples_.begin());
    }
    samples_.push_back(sample);
}

void PsmsClient::SetStreaming(bool streaming)
{
    std::lock_guard<std::mutex> lock(mutex_);
    streaming_ = streaming;
}

void PsmsClient::SetCameras(bool valid, Vec3 camera, int cameras)
{
    std::lock_guard<std::mutex> lock(mutex_);
    camera_valid_ = valid;
    camera_ = camera;
    cameras_ = cameras;
}

void PsmsClient::LogOnce(const std::string& message)
{
    if (message != last_message_) {
        DriverLog("%s", message.c_str());
        last_message_ = message;
    }
}

void PsmsClient::SleepWhileRunning(int milliseconds)
{
    for (int waited = 0; running_ && waited < milliseconds; waited += 50) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

#if defined(_WIN32)

struct PsmsClient::Api {
    decltype(&PSM_Initialize) Initialize;
    decltype(&PSM_Shutdown) Shutdown;
    decltype(&PSM_Update) Update;
    decltype(&PSM_GetIsInitialized) GetIsInitialized;
    decltype(&PSM_GetIsConnected) GetIsConnected;
    decltype(&PSM_GetClientVersionString) GetClientVersionString;
    decltype(&PSM_GetServiceVersionString) GetServiceVersionString;
    decltype(&PSM_HasHMDListChanged) HasHMDListChanged;
    decltype(&PSM_HasControllerListChanged) HasControllerListChanged;
    decltype(&PSM_HasTrackerListChanged) HasTrackerListChanged;
    decltype(&PSM_HasPlayspaceOffsetChanged) HasPlayspaceOffsetChanged;
    decltype(&PSM_GetTrackerList) GetTrackerList;
    decltype(&PSM_GetHmdList) GetHmdList;
    decltype(&PSM_AllocateHmdListener) AllocateHmdListener;
    decltype(&PSM_FreeHmdListener) FreeHmdListener;
    decltype(&PSM_StartHmdDataStream) StartHmdDataStream;
    decltype(&PSM_StopHmdDataStream) StopHmdDataStream;
    decltype(&PSM_GetIsHmdTracking) GetIsHmdTracking;
    decltype(&PSM_GetHmdPosition) GetHmdPosition;
    decltype(&PSM_GetControllerList) GetControllerList;
    decltype(&PSM_AllocateControllerListener) AllocateControllerListener;
    decltype(&PSM_FreeControllerListener) FreeControllerListener;
    decltype(&PSM_StartControllerDataStream) StartControllerDataStream;
    decltype(&PSM_StopControllerDataStream) StopControllerDataStream;
    decltype(&PSM_GetIsControllerTracking) GetIsControllerTracking;
    decltype(&PSM_GetControllerPosition) GetControllerPosition;
};

namespace {

std::wstring Widen(const std::string& utf8)
{
    if (utf8.empty()) {
        return {};
    }
    const int length = MultiByteToWideChar(CP_UTF8, 0, utf8.c_str(), -1, nullptr, 0);
    std::wstring wide(static_cast<size_t>(length > 0 ? length : 1), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, utf8.c_str(), -1, &wide[0], length);
    wide.resize(wcslen(wide.c_str()));
    return wide;
}

std::string Narrow(const std::wstring& wide)
{
    if (wide.empty()) {
        return {};
    }
    const int length = WideCharToMultiByte(CP_UTF8, 0, wide.c_str(), -1, nullptr, 0, nullptr, nullptr);
    std::string utf8(static_cast<size_t>(length > 0 ? length : 1), '\0');
    WideCharToMultiByte(CP_UTF8, 0, wide.c_str(), -1, &utf8[0], length, nullptr, nullptr);
    utf8.resize(std::strlen(utf8.c_str()));
    return utf8;
}

// Folders (with a trailing backslash) of the running processes with this executable name.
std::vector<std::wstring> ProcessFolders(const wchar_t* exe_name)
{
    std::vector<std::wstring> folders;
    const HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
    if (snapshot == INVALID_HANDLE_VALUE) {
        return folders;
    }
    PROCESSENTRY32W entry{};
    entry.dwSize = sizeof(entry);
    for (BOOL more = Process32FirstW(snapshot, &entry); more; more = Process32NextW(snapshot, &entry)) {
        if (_wcsicmp(entry.szExeFile, exe_name) != 0) {
            continue;
        }
        const HANDLE process = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, entry.th32ProcessID);
        if (!process) {
            continue;
        }
        wchar_t path[MAX_PATH * 2];
        DWORD size = static_cast<DWORD>(sizeof(path) / sizeof(path[0]));
        if (QueryFullProcessImageNameW(process, 0, path, &size)) {
            std::wstring folder(path, size);
            folder.resize(folder.find_last_of(L'\\') + 1);
            folders.push_back(folder);
        }
        CloseHandle(process);
    }
    CloseHandle(snapshot);
    return folders;
}

// Prefers the copy next to the running service: it is the build that matches it. VDM unpacks the
// service into its own "PSMoveServiceEx" subfolder and also carries an older copy of the DLL.
std::vector<std::wstring> DllCandidates(const std::string& configured)
{
    if (!configured.empty()) {
        return {Widen(configured)};
    }
    const wchar_t* const kDll = L"PSMoveClient_CAPI.dll";
    std::vector<std::wstring> candidates;
    for (const wchar_t* service : {L"PSMoveService.exe", L"PSMoveServiceAdmin.exe"}) {
        for (const std::wstring& folder : ProcessFolders(service)) {
            candidates.push_back(folder + kDll);
        }
    }
    for (const std::wstring& folder : ProcessFolders(L"PSMSVirtualDeviceManager.exe")) {
        candidates.push_back(folder + L"PSMoveServiceEx\\" + kDll);
        candidates.push_back(folder + kDll);
    }
    return candidates;
}

template <typename T>
void Resolve(HMODULE module, const char* name, T* function, std::string* missing)
{
    *function = reinterpret_cast<T>(reinterpret_cast<void*>(GetProcAddress(module, name)));
    if (!*function && missing->empty()) {
        *missing = name;
    }
}

// Returns the name of the first missing function, or an empty string.
template <typename Api>
std::string ResolveAll(HMODULE module, Api* api)
{
    std::string missing;
    Resolve(module, "PSM_Initialize", &api->Initialize, &missing);
    Resolve(module, "PSM_Shutdown", &api->Shutdown, &missing);
    Resolve(module, "PSM_Update", &api->Update, &missing);
    Resolve(module, "PSM_GetIsInitialized", &api->GetIsInitialized, &missing);
    Resolve(module, "PSM_GetIsConnected", &api->GetIsConnected, &missing);
    Resolve(module, "PSM_GetClientVersionString", &api->GetClientVersionString, &missing);
    Resolve(module, "PSM_GetServiceVersionString", &api->GetServiceVersionString, &missing);
    Resolve(module, "PSM_HasHMDListChanged", &api->HasHMDListChanged, &missing);
    Resolve(module, "PSM_HasControllerListChanged", &api->HasControllerListChanged, &missing);
    Resolve(module, "PSM_HasTrackerListChanged", &api->HasTrackerListChanged, &missing);
    Resolve(module, "PSM_HasPlayspaceOffsetChanged", &api->HasPlayspaceOffsetChanged, &missing);
    Resolve(module, "PSM_GetTrackerList", &api->GetTrackerList, &missing);
    Resolve(module, "PSM_GetHmdList", &api->GetHmdList, &missing);
    Resolve(module, "PSM_AllocateHmdListener", &api->AllocateHmdListener, &missing);
    Resolve(module, "PSM_FreeHmdListener", &api->FreeHmdListener, &missing);
    Resolve(module, "PSM_StartHmdDataStream", &api->StartHmdDataStream, &missing);
    Resolve(module, "PSM_StopHmdDataStream", &api->StopHmdDataStream, &missing);
    Resolve(module, "PSM_GetIsHmdTracking", &api->GetIsHmdTracking, &missing);
    Resolve(module, "PSM_GetHmdPosition", &api->GetHmdPosition, &missing);
    Resolve(module, "PSM_GetControllerList", &api->GetControllerList, &missing);
    Resolve(module, "PSM_AllocateControllerListener", &api->AllocateControllerListener, &missing);
    Resolve(module, "PSM_FreeControllerListener", &api->FreeControllerListener, &missing);
    Resolve(module, "PSM_StartControllerDataStream", &api->StartControllerDataStream, &missing);
    Resolve(module, "PSM_StopControllerDataStream", &api->StopControllerDataStream, &missing);
    Resolve(module, "PSM_GetIsControllerTracking", &api->GetIsControllerTracking, &missing);
    Resolve(module, "PSM_GetControllerPosition", &api->GetControllerPosition, &missing);
    return missing;
}

// The DLL resolves the host name with a call that throws on failure, which would take SteamVR
// down with it, so only plain IPv4 addresses and port numbers are passed in.
bool IsAddressValid(const PsmsConfig& config)
{
    in_addr address{};
    if (inet_pton(AF_INET, config.host.c_str(), &address) != 1) {
        return false;
    }
    if (config.port.empty() || config.port.size() > 5) {
        return false;
    }
    for (const char c : config.port) {
        if (c < '0' || c > '9') {
            return false;
        }
    }
    return true;
}

}  // namespace

void PsmsClient::Run()
{
    if (!IsAddressValid(config_)) {
        DriverLog("%spsms_host must be an IPv4 address and psms_port a number (got \"%s\", \"%s\")",
                  prefix_.c_str(), config_.host.c_str(), config_.port.c_str());
        return;
    }
    timeBeginPeriod(1);
    Api api{};
    HMODULE module = nullptr;
    while (running_ && !module) {
        std::string searched;
        for (const std::wstring& candidate : DllCandidates(config_.dll_path)) {
            searched += (searched.empty() ? "" : ", ") + Narrow(candidate);
            if (GetFileAttributesW(candidate.c_str()) == INVALID_FILE_ATTRIBUTES) {
                continue;
            }
            module = LoadLibraryExW(candidate.c_str(), nullptr, LOAD_WITH_ALTERED_SEARCH_PATH);
            if (module) {
                DriverLog("%susing %s", prefix_.c_str(), Narrow(candidate).c_str());
                break;
            }
        }
        if (!module) {
            LogOnce(searched.empty()
                        ? prefix_ + "waiting for PSMoveServiceEx to run (start it from VDM), or set psms_client_dll"
                        : prefix_ + "could not load PSMoveClient_CAPI.dll from " + searched);
            SleepWhileRunning(kRetryMilliseconds);
        }
    }
    if (module) {
        const std::string missing = ResolveAll(module, &api);
        if (!missing.empty()) {
            DriverLog("%sPSMoveClient_CAPI.dll has no %s; update PSMoveServiceEx", prefix_.c_str(), missing.c_str());
            running_ = false;
        }
        // The DLL stays loaded: unloading it while its network threads wind down is not safe.
    }

    while (running_) {
        // A failed connection leaves a half-started client behind; the official tools shut it
        // down before trying again.
        if (api.GetIsInitialized()) {
            api.Shutdown();
        }
        if (api.Initialize(config_.host.c_str(), config_.port.c_str(), kRequestTimeoutMs) != PSMResult_Success) {
            if (api.GetIsInitialized()) {
                api.Shutdown();
            }
            LogOnce(prefix_ + "waiting for PSMoveServiceEx at " + config_.host + ":" + config_.port);
            SleepWhileRunning(kRetryMilliseconds);
            continue;
        }
        char version[64] = {};
        api.GetServiceVersionString(version, sizeof(version) - 1, kRequestTimeoutMs);
        const char* client_version = api.GetClientVersionString();
        LogOnce(prefix_ + "connected to PSMoveServiceEx " + version + " (client " +
                (client_version ? client_version : "?") + ")");
        Stream(api);
        SetStreaming(false);
        SetControllersStreaming(false);
        SetCameras(false, Vec3{}, 0);
        if (api.GetIsInitialized()) {
            api.Shutdown();
        }
        if (running_) {
            LogOnce(prefix_ + "lost PSMoveServiceEx; reconnecting");
            SleepWhileRunning(kRetryMilliseconds);
        }
    }
    timeEndPeriod(1);
}

void PsmsClient::Stream(const Api& api)
{
    const bool use_controller = config_.controller_id >= 0;
    int device = -1;
    double next_device_check = 0.0;
    bool read_cameras = true;
    double next_camera_check = 0.0;
    bool last_tracking = false;
    PSMVector3f last_position{};

    // Every other PS Move, for calibrating the body cameras.
    struct Followed {
        int id;
        bool tracking;
        PSMVector3f position;
    };
    std::vector<Followed> followed;
    bool list_controllers = config_.stream_controllers;
    double next_controllers_check = 0.0;
    const auto stop_controllers = [&]() {
        const bool connected = api.GetIsConnected();
        for (const Followed& f : followed) {
            if (connected) {
                api.StopControllerDataStream(f.id, kRequestTimeoutMs);
            }
            api.FreeControllerListener(f.id);
        }
        followed.clear();
        SetControllersStreaming(false);
    };

    const auto stop_device = [&]() {
        if (device < 0) {
            return;
        }
        SetStreaming(false);
        // On a dead connection the stop request would only wait out its timeout.
        const bool connected = api.GetIsConnected();
        if (use_controller) {
            if (connected) {
                api.StopControllerDataStream(device, kRequestTimeoutMs);
            }
            api.FreeControllerListener(device);
        } else {
            if (connected) {
                api.StopHmdDataStream(device, kRequestTimeoutMs);
            }
            api.FreeHmdListener(device);
        }
        device = -1;
    };

    while (running_) {
        if (api.Update() != PSMResult_Success || !api.GetIsConnected()) {
            break;
        }
        // Read every flag each time so a stale one does not fire later.
        const bool hmds_changed = api.HasHMDListChanged();
        const bool controllers_changed = api.HasControllerListChanged();
        const bool devices_changed = use_controller ? controllers_changed : hmds_changed;
        const bool trackers_changed = api.HasTrackerListChanged();
        const bool offsets_changed = api.HasPlayspaceOffsetChanged();
        if (devices_changed) {
            stop_device();
            next_device_check = 0.0;
        }
        if (controllers_changed && config_.stream_controllers) {
            stop_controllers();
            list_controllers = true;
            next_controllers_check = 0.0;
        }

        if (trackers_changed || offsets_changed) {
            read_cameras = true;
            next_camera_check = 0.0;
        }
        if (read_cameras && MonoSeconds() >= next_camera_check) {
            read_cameras = false;
            PSMTrackerList trackers{};
            if (api.GetTrackerList(&trackers, kRequestTimeoutMs) == PSMResult_Success) {
                // The camera poses include the playspace position and yaw offsets, like the light.
                // Only the playspace scale (kept at 1) and per-HMD offsets would set them apart.
                const PSMVector3f p = trackers.count > 0 ? trackers.trackers[0].tracker_pose.Position : PSMVector3f{};
                SetCameras(trackers.count > 0, Vec3{p.x * 0.01, p.y * 0.01, p.z * 0.01}, trackers.count);
                LogOnce(prefix_ + std::to_string(trackers.count) + " camera(s), the first at (" +
                        std::to_string(static_cast<int>(p.x)) + ", " + std::to_string(static_cast<int>(p.y)) + ", " +
                        std::to_string(static_cast<int>(p.z)) + ") cm");
            } else {
                read_cameras = true;
                next_camera_check = MonoSeconds() + kCameraRetrySeconds;
            }
        }

        if (config_.follow_head && device < 0 && MonoSeconds() >= next_device_check) {
            next_device_check = MonoSeconds() + kDeviceRetrySeconds;
            int found = -1;
            if (use_controller) {
                PSMControllerList list{};
                if (api.GetControllerList(&list, kRequestTimeoutMs) == PSMResult_Success) {
                    for (int i = 0; i < list.count && i < PSMOVESERVICE_MAX_CONTROLLER_COUNT; ++i) {
                        if (list.controller_id[i] == config_.controller_id) {
                            found = config_.controller_id;
                        }
                    }
                }
            } else {
                PSMHmdList list{};
                if (api.GetHmdList(&list, kRequestTimeoutMs) == PSMResult_Success) {
                    for (int i = 0; i < list.count && i < PSMOVESERVICE_MAX_HMD_COUNT && found < 0; ++i) {
                        const bool wanted = config_.hmd_id >= 0 ? list.hmd_id[i] == config_.hmd_id
                                                                : list.hmd_type[i] == PSMHmd_Virtual;
                        if (wanted) {
                            found = list.hmd_id[i];
                        }
                    }
                }
            }
            const std::string name = (use_controller ? "controller " : "virtual HMD ") +
                                     std::to_string(found >= 0 ? found : (use_controller ? config_.controller_id
                                                                                         : config_.hmd_id));
            if (found < 0) {
                LogOnce(use_controller ? "Head position: PSMoveServiceEx has no " + name
                                       : "Head position: PSMoveServiceEx has no virtual HMD; set \"Virtual HMD count\" "
                                         "to 1 in VDM");
            } else {
                const bool started =
                    use_controller
                        ? api.AllocateControllerListener(found) == PSMResult_Success &&
                              api.StartControllerDataStream(found, PSMStreamFlags_includePositionData,
                                                            kRequestTimeoutMs) == PSMResult_Success
                        : api.AllocateHmdListener(found) == PSMResult_Success &&
                              api.StartHmdDataStream(found, PSMStreamFlags_includePositionData, kRequestTimeoutMs) ==
                                  PSMResult_Success;
                if (started) {
                    device = found;
                    last_tracking = false;
                    SetStreaming(true);
                    LogOnce("Head position: following " + name);
                } else {
                    if (use_controller) {
                        api.FreeControllerListener(found);
                    } else {
                        api.FreeHmdListener(found);
                    }
                    LogOnce("Head position: could not start streaming " + name);
                }
            }
        }

        if (device >= 0) {
            bool tracking = false;
            PSMVector3f position{};
            const bool tracked =
                use_controller ? api.GetIsControllerTracking(device, &tracking) == PSMResult_Success
                               : api.GetIsHmdTracking(device, &tracking) == PSMResult_Success;
            const bool valid = use_controller ? api.GetControllerPosition(device, &position) == PSMResult_Success
                                              : api.GetHmdPosition(device, &position) == PSMResult_Success;
            tracking = tracked && tracking && valid;
            if (tracking != last_tracking ||
                (tracking && std::memcmp(&position, &last_position, sizeof(position)) != 0)) {
                Push({MonoSeconds(), Vec3{position.x * 0.01, position.y * 0.01, position.z * 0.01}, tracking});
                last_tracking = tracking;
                last_position = position;
            }
        }
        if (list_controllers && MonoSeconds() >= next_controllers_check) {
            next_controllers_check = MonoSeconds() + kDeviceRetrySeconds;
            PSMControllerList list{};
            if (api.GetControllerList(&list, kRequestTimeoutMs) == PSMResult_Success) {
                for (int i = 0; i < list.count && i < PSMOVESERVICE_MAX_CONTROLLER_COUNT; ++i) {
                    const int id = list.controller_id[i];
                    if (list.controller_type[i] != PSMController_Move || (use_controller && id == config_.controller_id)) {
                        continue;  // only Moves have a bulb; the one on the visor is the head's
                    }
                    if (api.AllocateControllerListener(id) == PSMResult_Success &&
                        api.StartControllerDataStream(id, PSMStreamFlags_includePositionData, kRequestTimeoutMs) ==
                            PSMResult_Success) {
                        followed.push_back({id, false, PSMVector3f{}});
                    } else {
                        api.FreeControllerListener(id);
                    }
                }
                // Until one is connected, look again now and then.
                list_controllers = followed.empty();
                SetControllersStreaming(!followed.empty());
                LogOnce("Body tracking: following " + std::to_string(followed.size()) +
                        " PS Move(s), for calibrating the cameras");
            }
        }
        for (Followed& f : followed) {
            bool tracking = false;
            PSMVector3f position{};
            const bool tracked = api.GetIsControllerTracking(f.id, &tracking) == PSMResult_Success;
            const bool valid = api.GetControllerPosition(f.id, &position) == PSMResult_Success;
            tracking = tracked && tracking && valid;
            if (tracking != f.tracking || (tracking && std::memcmp(&position, &f.position, sizeof(position)) != 0)) {
                PushController({MonoSeconds(), f.id, Vec3{position.x * 0.01, position.y * 0.01, position.z * 0.01},
                                tracking});
                f.tracking = tracking;
                f.position = position;
            }
        }
        Sleep(1);
    }
    stop_device();
    stop_controllers();
}

#else

void PsmsClient::Run()
{
    DriverLog("Head position from PSMoveServiceEx is only available on Windows");
}

#endif
