#include <emscripten.h>

extern "C" {

    EMSCRIPTEN_KEEPALIVE
    int astar(int start_x, int start_y, int goal_x, int goal_y) {
        // dummy return for now
        return (goal_x - start_x) + (goal_y - start_y);
    }
    
}
