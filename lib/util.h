#pragma once
#include <vector>
#include <map>
#include <cfloat>


class Position
{
public:
    int x, y;

    Position(int x = 0, int y = 0) : x(x), y(y) {}
    bool operator==(const Position& other) const {
        return other.x == x && other.y == y;
    }

};
struct Cell { 
    bool isObstacle = false;
    Position position;
    double g = FLT_MAX, h = FLT_MAX, f = FLT_MAX; // Costs for A*
    Cell* parent = nullptr;
    int time = 1;

    // Constructor with position and parent initialization
    Cell(Position pos, double g = FLT_MAX, double h = FLT_MAX, double f = FLT_MAX, int time = 1, Cell* parent = nullptr)
        : position(pos), g(g), h(h), f(f), parent(parent), time(time) {}


    // Comparison operator for priority queue
    bool operator<(const Cell& other) const {
        return f > other.f; // Used by priority queue in A*
    }

    // Equality operator to compare cells based on position
    bool operator==(const Cell& other) const {
        return other.position == position;
    }
};


struct CompareCells {
    bool operator()(const Cell a, const Cell b) const {
        return a.f > b.f;
    }
};
struct Agent
{
    int id;
    Cell start;
    Cell goal;
    std::vector<std::vector<Cell>> path;

    Agent(int id, Position start, Position goal)
         : id(id), start(start), goal(goal) {

    }
    bool move();
};




struct Conflict{

    int agent1, agent2;
    int time;
    Cell pos1, pos2;

    Conflict(int agent1, int agent2, int time, Cell pos1, Cell pos2)
        :agent1(agent1), agent2(agent2), time(time), pos1(pos1), pos2(pos2)
    {}
    
};




struct Constraint {
    Position position;
    int time;
    int agentID;

    Constraint(int agentID, Position position, int time)
        : agentID(agentID), position(position), time(time) {}

    bool operator==(const Constraint& c) const {
        return c.agentID == agentID && c.time == time && position == c.position;
    }
};



struct Map {
    std::vector<std::vector<Cell>> cells;
    std::vector<Agent> agents;
};

