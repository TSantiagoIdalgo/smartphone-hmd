#include <cstring>

#include "device_provider.h"
#include "openvr_driver.h"

#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec(dllexport)
#else
#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))
#endif

static DeviceProvider g_device_provider;

// Entry point SteamVR calls after loading the DLL.
HMD_DLL_EXPORT void* HmdDriverFactory(const char* interface_name, int* return_code)
{
    if (std::strcmp(vr::IServerTrackedDeviceProvider_Version, interface_name) == 0) {
        return &g_device_provider;
    }
    if (return_code) {
        *return_code = vr::VRInitError_Init_InterfaceNotFound;
    }
    return nullptr;
}
