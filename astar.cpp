#include <emscripten.h>
#include <iostream>
#include <unordered_set>

struct Position {
    int row, col;

    Position operator+(const Position& other) const {
        return {row + other.row, col + other.col};
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
    std::unordered_set<Position, PositionHash> bike_racks_rowcol;
};
static Params params;
static std::vector<int> pathFlat;

inline bool isValid(const Position& position) {
    return 0 <= position.row && position.row < params.height &&
           0 <= position.col && position.col < params.width;
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

int eightConnectedDistance(Position p1, Position p2)
{
    int absDeltaR = std::abs(p1.row - p2.row);
    int absDeltaC = std::abs(p1.col - p2.col);

    int minDelta = std::min(absDeltaR, absDeltaC);
    int maxDelta = std::max(absDeltaR, absDeltaC);

    return (std::sqrt(2) - 1) * minDelta + maxDelta;
}

enum Mode {
    WALKING, BIKING
};

struct Node {
    Position position;
    Mode mode;
    double f; // f = g + h

    bool operator>(const Node& other) const
    {
        return f > other.f;
    }
};

int getHeuristic(Position position, Position goal) {
    return eightConnectedDistance(position, goal);
}

extern "C" {

/**
 * @param mode The initial mode of the planner. Use `0` for walking and `1` for biking.
 *
 */
int* aStar(int start_r, int start_c, int goal_r, int goal_c, int mode)
{
    Position start = { start_r, start_c };
    Position goal = { goal_r, goal_c };
    pathFlat.clear();

    std::unordered_set<Position, PositionHash> closed;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    std::unordered_map<Position, Position, PositionHash> prev;
    std::unordered_map<Position, float, PositionHash> dist;

    open.push({ start, static_cast<Mode>(mode), 0 });
    while (open.size() > 0) {
        Node node = open.top();
        open.pop();

        if (closed.find(node.position) != closed.end()) {
            continue;
        }
        closed.insert(node.position);

        if (node.position == goal) {
            std::vector<Position> path;
            for (Position at = goal; at != start; at = prev[at]) {
                path.push_back(at);
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());

            for (const auto& [r, c] : path) {
                pathFlat.push_back(r);
                pathFlat.push_back(c);
            }
            printf("Found path of size %zu.\n", path.size());
            return pathFlat.data();
        }

        for (const auto& [move, moveCost] : moves) {
            Position newPosition = node.position + move;
            if (!isValid(newPosition) || closed.find(newPosition) != closed.end()) {
                continue; 
            }

            float newCost = node.f + moveCost + params.walking[newPosition.row][newPosition.col] + getHeuristic(newPosition, goal);
            if (dist.count(newPosition) == 0 || newCost < dist[newPosition]) {
                dist[newPosition] = newCost;
                prev[newPosition] = node.position;
                open.push({newPosition, WALKING, newCost});
            }
        }
    }

    printf("No path found!\n");
    return NULL;
}

int aStarPathLength()
{
    return pathFlat.size() / 2;
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

            if (params.walking[r][c] == 255) {
                params.walking[r][c] = 9999;
            }
        }
    }
    for (int i = 0; i < bike_racks_len; ++i) {
        int row = bike_racks_ptr[2 * i];
        int col = bike_racks_ptr[2 * i + 1];
        params.bike_racks_rowcol.insert({ row, col });
    }
}
}
