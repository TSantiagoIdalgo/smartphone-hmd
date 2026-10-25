#pragma once

#include <string>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "openvr_driver.h"

// The driver's settings (resources/settings/default.vrsettings, and what SteamVR saved over them
// in steamvr.vrsettings) and its keyboard shortcuts.

inline const char* const kSettingsSection = "driver_smartphone_hmd";

inline std::string GetStringSetting(const char* key, const char* fallback = "")
{
    char buffer[1024] = {};
    vr::EVRSettingsError error = vr::VRSettingsError_None;
    vr::VRSettings()->GetString(kSettingsSection, key, buffer, sizeof(buffer), &error);
    return error == vr::VRSettingsError_None ? buffer : fallback;
}

// Settings added after the first release fall back to their defaults when a key is missing, so
// updating only the DLL (and keeping an older default.vrsettings) still works.
inline double GetDoubleSetting(const char* key, double fallback)
{
    vr::EVRSettingsError error = vr::VRSettingsError_None;
    const double value = vr::VRSettings()->GetFloat(kSettingsSection, key, &error);
    return error == vr::VRSettingsError_None ? value : fallback;
}

inline int GetIntSetting(const char* key, int fallback)
{
    vr::EVRSettingsError error = vr::VRSettingsError_None;
    const int value = vr::VRSettings()->GetInt32(kSettingsSection, key, &error);
    return error == vr::VRSettingsError_None ? value : fallback;
}

inline bool GetBoolSetting(const char* key, bool fallback)
{
    vr::EVRSettingsError error = vr::VRSettingsError_None;
    const bool value = vr::VRSettings()->GetBool(kSettingsSection, key, &error);
    return error == vr::VRSettingsError_None ? value : fallback;
}

// Ctrl+Alt+<key> on the PC keyboard.
inline bool HotkeyDown(int key)
{
#if defined(_WIN32)
    return (GetAsyncKeyState(VK_CONTROL) & 0x8000) && (GetAsyncKeyState(VK_MENU) & 0x8000) &&
           (GetAsyncKeyState(key) & 0x8000);
#else
    (void)key;
    return false;
#endif
}
