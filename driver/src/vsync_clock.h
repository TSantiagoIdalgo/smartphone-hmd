#pragma once

#include <cstdint>

// The refresh of our virtual display, as SteamVR's compositor sees it through
// IVRVirtualDisplay::GetTimeSinceLastVsync. Vsyncs fall every `frame_seconds` from time 0.
//
// A presented frame shows at the next vsync, and that vsync is reported as soon as the frame is
// presented, so the time since it is negative until it comes. This is what Valve's
// virtual_display sample does: it makes the compositor wait for that vsync before rendering
// again. Without it the compositor rendered two or three frames per vsync, the likely cause of
// DirectX games showing only gray.
class VsyncClock {
public:
    explicit VsyncClock(double frame_seconds) : frame_seconds_(frame_seconds) {}

    // A frame was presented at `now` seconds. If a frame is already waiting for the next vsync,
    // this one goes to the vsync after it, and never further.
    void FramePresented(double now);

    // The latest vsync reported so far and the seconds since it at `now` (negative while a
    // presented frame waits for it).
    void SinceLastVsync(double now, double* seconds, uint64_t* counter) const;

private:
    int64_t VsyncsBefore(double now) const;

    double frame_seconds_;
    int64_t presented_vsync_ = 0;
};
