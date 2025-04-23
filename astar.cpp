#include <emscripten.h>
#include <iostream>
#include <unordered_set>

#define BIKING_TO_WALKING 5
#define H_WEIGHT 2

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

static const std::unordered_map<Position, float, PositionHash> walkingMoves = {
    { { 1, 0 }, BIKING_TO_WALKING * 1.0f },
    { { -1, 0 }, BIKING_TO_WALKING * 1.0f },
    { { 0, 1 }, BIKING_TO_WALKING * 1.0f },
    { { 0, -1 }, BIKING_TO_WALKING * 1.0f },
    { { 1, 1 }, BIKING_TO_WALKING * std::sqrt(2.0f) },
    { { 1, -1 }, BIKING_TO_WALKING * std::sqrt(2.0f) },
    { { -1, 1 }, BIKING_TO_WALKING * std::sqrt(2.0f) },
    { { -1, -1 }, BIKING_TO_WALKING * std::sqrt(2.0f) },
};

static const std::unordered_map<Position, float, PositionHash> bikingMoves = {
    { { 1, 0 }, 1.0f },
    { { -1, 0 }, 1.0f },
    { { 0, 1 }, 1.0f },
    { { 0, -1 }, 1.0f },
    { { 1, 1 }, std::sqrt(2.0f) },
    { { 1, -1 }, std::sqrt(2.0f) },
    { { -1, 1 }, std::sqrt(2.0f) },
    { { -1, -1 }, std::sqrt(2.0f) },
};

std::unordered_map<Position, float, PositionHash> getMoves(const Mode& mode)
{
    switch (mode) {
    case WALKING:
        return walkingMoves;
    case BIKING:
        return bikingMoves;
    }
}

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
    double gCost;
    double hCost;
    int fCost() const { return gCost + hCost; }

    bool operator>(const Node& other) const
    {
        return fCost() > other.fCost();
    }
};

float getBikeHeuristic(Position position, Position goal) {
    float min = std::numeric_limits<float>::max();
    for (const Position& bike_rack : params.bike_racks) {
        float positionToBikeRack = eightConnectedDistance(position, bike_rack);
        float bikeRackToGoal = BIKING_TO_WALKING * eightConnectedDistance(bike_rack, goal);
        float heuristic = positionToBikeRack + bikeRackToGoal;
        min = std::min(min, heuristic);
    }
    return min;
}


float getHeuristic(State state, State goal)
{
    switch (state.mode) {
        case BIKING:
        return getBikeHeuristic(state.position, goal.position);
        case WALKING:
        return BIKING_TO_WALKING * eightConnectedDistance(state.position, goal.position);
    }
    
}

State findNearestValidPosition(const State& state)
{
    std::queue<Position> queue;
    std::unordered_set<Position, PositionHash> visited;

    queue.push(state.position);
    visited.insert(state.position);

    const std::unordered_map<Position, float, PositionHash>& moves = getMoves(state.mode);

    while (!queue.empty()) {
        Position current = queue.front();
        queue.pop();

        State testState = { current, state.mode };
        if (isValid(testState)) {
            return testState;
        }

        for (const auto& [move, _] : moves) {
            Position neighbor = current + move;
            if (isValid(neighbor) && visited.find(neighbor) == visited.end()) {
                visited.insert(neighbor);
                queue.push(neighbor);
            }
        }
    }

    return { -1, -1 };
}


extern "C" {

/**
 * @param mode The initial mode of the planner. Use `0` for walking and `1` for biking.
 *
 */
int* aStar(int start_r, int start_c, int goal_r, int goal_c, int mode)
{
    printf("Starting A* from (%d, %d) to (%d, %d) with mode %d.\n", start_r, start_c, goal_r, goal_c, mode);
    auto startTime = std::chrono::high_resolution_clock::now();

    pathFlat.clear();
    Position startPosition = { start_r, start_c };
    Position goalPosition = { goal_r, goal_c };

    State start = { startPosition, static_cast<Mode>(mode) };
    State goal = { goalPosition, WALKING };
    if (!isValid(start)) {
        start = findNearestValidPosition(start);
    }
    if (!isValid(goal)) {
        goal = findNearestValidPosition(goal);
    }


    std::unordered_set<State, StateHash> closed;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    std::unordered_map<State, State, StateHash> prev;
    std::unordered_map<State, float, StateHash> dist;

    open.push({ start, 0, 0 });
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
            auto endTime = std::chrono::high_resolution_clock::now();

            std::chrono::duration<double> duration = endTime - startTime;
            printf("Found path of size %zu in %f seconds.\n", path.size(), duration.count());
            return pathFlat.data();
        }

        for (const auto& [move, moveCost] : getMoves(node.state.mode)) {
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

                float newCost = node.gCost + moveCost;
                if (dist.count(newState) == 0 || newCost < dist[newState]) {
                    dist[newState] = newCost;
                    prev[newState] = node.state;
                    open.push({ newState, newCost, H_WEIGHT * getHeuristic(newState, goal) });
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
