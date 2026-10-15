#pragma once

#include <cstdint>
#include <string>

// SteamVR universe of our headset. 0 means "no universe" and 1 is Oculus's.
constexpr uint32_t kUniverseId = 0x53484D44;  // "SHMD"

// The chaperone setup SteamVR asks every driver with its own tracking to provide
// (Prop_DriverProvidedChaperoneJson_String). Without it SteamVR has no standing or seated
// universe, keeps asking for room setup, and some games only show gray.
//
// Our raw tracking space already has the floor at y = 0 and the head straight above the origin,
// so the standing universe is the raw space, and the seated one is centred on the head.
std::string ChaperoneJson(uint32_t universe_id, float head_height_meters);
