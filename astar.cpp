#include <emscripten.h>
#include <iostream>
#include <unordered_set>

struct Position {
    int row, col;

    Position operator+(const Position& other) const
    {
        return { row + other.row, col + other.col };
    }
    bool operator==(const Position& other) const
    {
        return row == other.row && col == other.col;
    }
    bool operator!=(const Position& other) const
    {
        return !(*this == other);
    }
};
struct PositionHash {
    std::size_t operator()(const Position& p) const
    {
        return (std::hash<int>()(p.row) << 1) ^ (std::hash<int>()(p.col));
    }
};

struct Params {
    int width;
    int height;
    std::vector<std::vector<int>> walking;
    std::vector<std::vector<int>> biking;
    std::unordered_set<Position, PositionHash> bike_racks;
};
static Params params;

enum Mode {
    WALKING,
    BIKING
};

inline bool isValid(const Position& position)
{
    return 0 <= position.row && position.row < params.height && 0 <= position.col && position.col < params.width;
}

struct State {
    Position position;
    Mode mode;

    bool operator==(const State& other) const
    {
        return position == other.position && mode == other.mode;
    }
    bool operator!=(const State& other) const
    {
        return !(*this == other);
    }
};
struct StateHash {
    std::size_t operator()(const State& state) const
    {
        std::size_t position_hash = PositionHash {}(state.position);
        std::size_t mode_hash = std::hash<int>()(static_cast<int>(state.mode));
        return position_hash ^ (mode_hash << 1);
    }
};

inline bool isValid(const State& state)
{
    switch (state.mode) {
    case WALKING:
        return params.walking[state.position.row][state.position.col] == 0;
    case BIKING:
        return params.biking[state.position.row][state.position.col] == 0;
    }
}

static const std::unordered_map<Position, float, PositionHash> moves = {
    { { 1, 0 }, 1.0f },
    { { -1, 0 }, 1.0f },
    { { 0, 1 }, 1.0f },
    { { 0, -1 }, 1.0f },
    { { 1, 1 }, std::sqrt(2.0f) },
    { { 1, -1 }, std::sqrt(2.0f) },
    { { -1, 1 }, std::sqrt(2.0f) },
    { { -1, -1 }, std::sqrt(2.0f) },
};

float eightConnectedDistance(Position p1, Position p2)
{
    int absDeltaR = std::abs(p1.row - p2.row);
    int absDeltaC = std::abs(p1.col - p2.col);

    int minDelta = std::min(absDeltaR, absDeltaC);
    int maxDelta = std::max(absDeltaR, absDeltaC);

    return (std::sqrt(2) - 1) * minDelta + maxDelta;
}

static std::vector<int> pathFlat;

struct Node {
    State state;
    double f; // f = g + h

    bool operator>(const Node& other) const
    {
        return f > other.f;
    }
};

int getHeuristic(State state, State goal)
{
    return eightConnectedDistance(state.position, goal.position);
}

extern "C" {

/**
 * @param mode The initial mode of the planner. Use `0` for walking and `1` for biking.
 *
 */
int* aStar(int start_r, int start_c, int goal_r, int goal_c, int mode)
{
    pathFlat.clear();
    Position startPosition = { start_r, start_c };
    Position goalPosition = { goal_r, goal_c };
    State start = { startPosition, static_cast<Mode>(mode) };
    State goal = { goalPosition, WALKING };

    std::unordered_set<State, StateHash> closed;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    std::unordered_map<State, State, StateHash> prev;
    std::unordered_map<State, float, StateHash> dist;

    open.push({ start, 0 });
    while (open.size() > 0) {
        Node node = open.top();
        open.pop();

        if (closed.find(node.state) != closed.end()) {
            continue;
        }
        closed.insert(node.state);

        if (node.state == goal) {
            std::vector<State> path;
            for (State at = goal; at != start; at = prev[at]) {
                path.push_back(at);
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());

            for (const auto& state : path) {
                pathFlat.push_back(state.position.row);
                pathFlat.push_back(state.position.col);
                pathFlat.push_back(state.mode);
            }
            printf("Found path of size %zu.\n", path.size());
            return pathFlat.data();
        }

        for (const auto& [move, moveCost] : moves) {
            Position newPosition = node.state.position + move;
            if (!isValid(newPosition)) {
                continue;
            }

            Mode newMode = node.state.mode;
            std::unordered_set<State, StateHash> newStates;
            newStates.insert({ newPosition, newMode });
            if (params.bike_racks.find(newPosition) != params.bike_racks.end()) {
                newStates.insert({ newPosition, WALKING });
            }

            for (const State& newState : newStates) {
                if (!isValid(newState) || closed.find(newState) != closed.end()) {
                    
                    continue;
                }

                float newCost = node.f + moveCost + params.walking[newPosition.row][newPosition.col] + (getHeuristic(newState, goal));
                if (dist.count(newState) == 0 || newCost < dist[newState]) {
                    dist[newState] = newCost;
                    prev[newState] = node.state;
                    open.push({ { newPosition, newMode }, newCost });
                }
            }
        }
    }

    printf("No path found!\n");
    return NULL;
}

int aStarPathLength()
{
    return pathFlat.size() / 3;
}

void transfer_params(int width, int height,
    int* walking_flat,
    int* biking_flat,
    int* bike_racks_ptr, int bike_racks_len)
{
    params.width = width;
    params.height = height;
    params.walking.resize(height, std::vector<int>(width));
    params.biking.resize(height, std::vector<int>(width));

    for (int r = 0; r < height; r++) {
        for (int c = 0; c < width; c++) {
            params.walking[r][c] = walking_flat[r * width + c];
            params.biking[r][c] = biking_flat[r * width + c];
        }
    }
    for (int i = 0; i < bike_racks_len; ++i) {
        int row = bike_racks_ptr[2 * i];
        int col = bike_racks_ptr[2 * i + 1];
        params.bike_racks.insert({ row, col });
    }
}
}
