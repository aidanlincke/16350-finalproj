#include <emscripten.h>
#include "json.hpp"
using json = nlohmann::json;

extern "C" {

    EMSCRIPTEN_KEEPALIVE
    int astar(int start_x, int start_y, int goal_x, int goal_y) {
        return 0;
    }
    
}
