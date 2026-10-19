#include "LowLevel.h"

#include <algorithm>
#include <cstdlib>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>
#include <tuple>

bool isTraversable(const Map& map, Position p) {
    return p.x >= 0 && p.y >= 0 && static_cast<std::size_t>(p.x) < map.cells.size() &&
           static_cast<std::size_t>(p.y) < map.cells[static_cast<std::size_t>(p.x)].size() &&
           !map.cells[static_cast<std::size_t>(p.x)][static_cast<std::size_t>(p.y)].isObstacle;
}

void validateProblem(const Map& map, const std::vector<Constraint>& constraints) {
    if (map.cells.empty() || map.cells.front().empty())
        throw std::invalid_argument("The grid must not be empty.");
    const auto height = map.cells.front().size();
    const auto maxInt = static_cast<std::size_t>(std::numeric_limits<int>::max());
    if (map.cells.size() > maxInt / height)
        throw std::invalid_argument("The grid exceeds supported dimensions.");
    for (const auto& column : map.cells)
        if (column.size() != height) throw std::invalid_argument("The grid must be rectangular.");
    std::set<int> ids;
    for (const auto& agent : map.agents) {
        if (!ids.insert(agent.id).second) throw std::invalid_argument("Agent IDs must be unique.");
        if (!isTraversable(map, agent.start.position) || !isTraversable(map, agent.goal.position))
            throw std::invalid_argument("Agent endpoints must be traversable grid cells.");
    }
    const int cells = static_cast<int>(map.cells.size() * height);
    for (const auto& c : constraints) {
        if (ids.count(c.agentID) == 0 || c.time < 0 ||
            c.time > std::numeric_limits<int>::max() - cells || !isTraversable(map, c.position))
            throw std::invalid_argument("Invalid constraint agent, time, or position.");
        if (c.isEdge && (c.time == 0 || !isTraversable(map, c.from) ||
                        LowLevel::calculateHValue(c.from, c.position) > 1))
            throw std::invalid_argument("An edge constraint requires a legal transition and positive arrival time.");
    }
}

int LowLevel::calculateHValue(Position p, Position q) {
    return std::abs(p.x - q.x) + std::abs(p.y - q.y);
}

bool LowLevel::isConstraint(const Constraint& constraint, const std::vector<Constraint>& constraints) {
    return std::find(constraints.begin(), constraints.end(), constraint) != constraints.end();
}

namespace {
const int moves[5][2] = {{0, 0}, {1, 0}, {0, 1}, {-1, 0}, {0, -1}};

bool forbidden(const std::vector<Constraint>& constraints, int id, Position from, Position to, int time) {
    for (const auto& c : constraints)
        if (c.agentID == id && c.time == time && c.position == to && (!c.isEdge || c.from == from))
            return true;
    return false;
}

bool canStay(const std::vector<Constraint>& constraints, int id, Position goal, int time) {
    for (const auto& c : constraints)
        if (c.agentID == id && c.time > time && c.position == goal && (!c.isEdge || c.from == goal))
            return false;
    return true;
}

bool connected(const Map& map, Position start, Position goal) {
    std::queue<Position> queue;
    std::set<std::pair<int, int>> visited;
    queue.push(start);
    visited.emplace(start.x, start.y);
    while (!queue.empty()) {
        auto p = queue.front();
        queue.pop();
        if (p == goal) return true;
        for (int i = 1; i < 5; ++i) {
            Position next(p.x + moves[i][0], p.y + moves[i][1]);
            if (isTraversable(map, next) && visited.emplace(next.x, next.y).second) queue.push(next);
        }
    }
    return false;
}

struct Record { Position position; int time; std::size_t parent; };
struct Entry {
    long long f;
    int h;
    std::size_t index;
    bool operator<(const Entry& other) const {
        return std::tie(f, h, index) > std::tie(other.f, other.h, other.index);
    }
};
}

std::vector<Cell> LowLevel::aStarSearch(const Map& map, int agentID,
                                       const std::vector<Constraint>& constraints) {
    status_ = SearchStatus::NotStarted;
    validateProblem(map, constraints);
    const auto agent = std::find_if(map.agents.begin(), map.agents.end(),
                                   [agentID](const Agent& a) { return a.id == agentID; });
    if (agent == map.agents.end()) throw std::invalid_argument("Unknown agent ID.");
    const auto start = agent->start.position;
    const auto goal = agent->goal.position;
    status_ = SearchStatus::NoSolution;
    if (forbidden(constraints, agentID, start, start, 0) || !connected(map, start, goal)) return {};
    int lastConstraint = 0;
    for (const auto& c : constraints)
        if (c.agentID == agentID) lastConstraint = std::max(lastConstraint, c.time);
    // After the final restriction, a simple static path needs fewer than |V| moves.
    const int horizon = lastConstraint + static_cast<int>(map.cells.size() * map.cells.front().size());
    if (limits_.maxLowLevelStates == 0) { status_ = SearchStatus::ResourceLimit; return {}; }
    const auto noParent = std::numeric_limits<std::size_t>::max();
    std::vector<Record> records{{start, 0, noParent}};
    std::set<std::tuple<int, int, int>> visited;
    visited.emplace(start.x, start.y, 0);
    std::priority_queue<Entry> open;
    const int h = calculateHValue(start, goal);
    open.push({h, h, 0});
    while (!open.empty()) {
        const auto index = open.top().index;
        open.pop();
        // Copy before appending records, which may reallocate the container.
        const auto current = records[index];
        if (current.position == goal && canStay(constraints, agentID, goal, current.time)) {
            std::vector<Cell> path;
            for (auto i = index; i != noParent; i = records[i].parent)
                path.emplace_back(records[i].position, records[i].time);
            std::reverse(path.begin(), path.end());
            status_ = SearchStatus::Success;
            return path;
        }
        if (current.time == horizon) continue;
        for (const auto& move : moves) {
            Position next(current.position.x + move[0], current.position.y + move[1]);
            const int time = current.time + 1;
            const auto key = std::make_tuple(next.x, next.y, time);
            if (!isTraversable(map, next) || forbidden(constraints, agentID, current.position, next, time) ||
                visited.count(key) != 0) continue;
            if (records.size() >= limits_.maxLowLevelStates) {
                status_ = SearchStatus::ResourceLimit;
                return {};
            }
            visited.insert(key);
            records.push_back({next, time, index});
            const int heuristic = calculateHValue(next, goal);
            open.push({static_cast<long long>(time) + heuristic, heuristic, records.size() - 1});
        }
    }
    return {};
}

std::vector<std::vector<Cell>> LowLevel::getPaths(const std::vector<Constraint>& constraints, const Map& map) {
    validateProblem(map, constraints);
    std::vector<std::vector<Cell>> paths;
    for (const auto& agent : map.agents) {
        auto path = aStarSearch(map, agent.id, constraints);
        if (status_ != SearchStatus::Success) return {};
        paths.push_back(std::move(path));
    }
    status_ = SearchStatus::Success;
    return paths;
}
