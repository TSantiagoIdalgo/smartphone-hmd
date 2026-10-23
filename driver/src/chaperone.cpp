#include "chaperone.h"

#include <cstdio>

std::string ChaperoneJson(uint32_t universe_id, float head_height_meters)
{
    // A 4 x 4 m play area: the head only moves a few centimetres (neck model), so its walls never
    // come close enough to show. "jsonid" is what SteamVR's own file uses; the docs say "json_id".
    char json[1024];
    std::snprintf(json, sizeof(json),
                  "{\n"
                  "  \"jsonid\": \"chaperone_info\",\n"
                  "  \"json_id\": \"chaperone_info\",\n"
                  "  \"version\": 5,\n"
                  "  \"time\": \"2026-10-15T00:00:00\",\n"
                  "  \"universes\": [\n"
                  "    {\n"
                  "      \"collision_bounds\": [\n"
                  "        [[-2, 0, -2], [-2, 2.4, -2], [2, 2.4, -2], [2, 0, -2]],\n"
                  "        [[2, 0, -2], [2, 2.4, -2], [2, 2.4, 2], [2, 0, 2]],\n"
                  "        [[2, 0, 2], [2, 2.4, 2], [-2, 2.4, 2], [-2, 0, 2]],\n"
                  "        [[-2, 0, 2], [-2, 2.4, 2], [-2, 2.4, -2], [-2, 0, -2]]\n"
                  "      ],\n"
                  "      \"play_area\": [4, 4],\n"
                  "      \"seated\": {\"translation\": [0, %.3f, 0], \"yaw\": 0},\n"
                  "      \"standing\": {\"translation\": [0, 0, 0], \"yaw\": 0},\n"
                  "      \"universeID\": \"%u\"\n"
                  "    }\n"
                  "  ]\n"
                  "}\n",
                  -static_cast<double>(head_height_meters), static_cast<unsigned>(universe_id));
    return json;
}
