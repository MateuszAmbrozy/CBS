#pragma once

#include <cstddef>
#include <vector>

struct Position {
    int x;
    int y;
    Position(int x = 0, int y = 0) : x(x), y(y) {}
    bool operator==(const Position& other) const { return x == other.x && y == other.y; }
    bool operator!=(const Position& other) const { return !(*this == other); }
};

struct Cell {
    Position position;
    bool isObstacle = false;
    int time = 0;
    explicit Cell(Position position = Position(), int time = 0) : position(position), time(time) {}
    bool operator==(const Cell& other) const { return position == other.position; }
};

struct Agent {
    int id;
    Cell start;
    Cell goal;
    Agent(int id, Position start, Position goal) : id(id), start(start), goal(goal) {}
};

// Edge constraints forbid from -> position at the specified arrival time.
struct Constraint {
    int agentID;
    Position position;
    int time;
    bool isEdge = false;
    Position from;

    Constraint(int agentID, Position position, int time)
        : agentID(agentID), position(position), time(time) {}
    static Constraint edge(int agentID, Position from, Position to, int arrivalTime) {
        Constraint result(agentID, to, arrivalTime);
        result.isEdge = true;
        result.from = from;
        return result;
    }
    bool operator==(const Constraint& other) const {
        return agentID == other.agentID && position == other.position && time == other.time &&
               isEdge == other.isEdge && (!isEdge || from == other.from);
    }
};

struct Conflict {
    // Agent fields are path indices; constraints use Agent::id instead.
    std::size_t agent1 = 0;
    std::size_t agent2 = 0;
    int time = 0;
    Position pos1;
    Position pos2;
    bool isEdge = false;
};

struct Map {
    // Rectangular grid indexed by cells[x][y].
    std::vector<std::vector<Cell>> cells;
    std::vector<Agent> agents;
};

enum class SearchStatus { NotStarted, Success, NoSolution, ResourceLimit };

struct SearchLimits {
    std::size_t maxHighLevelNodes = 10000;
    std::size_t maxLowLevelStates = 1000000;
};

// Invalid maps and constraints throw std::invalid_argument.
void validateProblem(const Map& map, const std::vector<Constraint>& constraints);
bool isTraversable(const Map& map, Position position);
