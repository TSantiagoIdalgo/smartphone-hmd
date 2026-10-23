#pragma once

#include <chrono>

// Seconds on the steady clock: the time base shared by the phone and PSMoveServiceEx samples.
inline double MonoSeconds()
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
