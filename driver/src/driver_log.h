#pragma once

#include <cstdarg>
#include <cstdio>

#include "openvr_driver.h"

// Writes to the SteamVR web console log, prefixed with "smartphone_hmd:".
inline void DriverLog(const char* format, ...)
{
    char buffer[1024];
    va_list args;
    va_start(args, format);
    std::vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (vr::VRDriverLog()) {
        vr::VRDriverLog()->Log(buffer);
    }
}
