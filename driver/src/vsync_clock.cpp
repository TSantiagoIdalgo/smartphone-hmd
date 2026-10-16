#include "vsync_clock.h"

#include <algorithm>
#include <cmath>

int64_t VsyncClock::VsyncsBefore(double now) const
{
    return static_cast<int64_t>(std::floor(now / frame_seconds_));
}

void VsyncClock::FramePresented(double now)
{
    const int64_t next = VsyncsBefore(now) + 1;
    presented_vsync_ = presented_vsync_ >= next ? next + 1 : next;
}

void VsyncClock::SinceLastVsync(double now, double* seconds, uint64_t* counter) const
{
    // Between presents the display keeps refreshing, so the count keeps going.
    const int64_t vsync = std::max(VsyncsBefore(now), presented_vsync_);
    *seconds = now - static_cast<double>(vsync) * frame_seconds_;
    *counter = static_cast<uint64_t>(vsync);
}
