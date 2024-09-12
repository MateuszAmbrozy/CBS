#pragma once
#include <algorithm>
#include "util.h"
#include <cfloat>
class LowLevel
{
private:
    std::vector<Cell> tracePath(Map& map, Cell* destCell);
    bool isValid(const Map& map, Position pos, int currentTime, const std::vector<Constraint>& constraints);
    bool isBlocked(const Map& map, Position pos, const std::vector<Constraint>& constraints, int currentTime) ;
public:
    LowLevel();
    ~LowLevel();


    bool isConstraint(Constraint constraint, const std::vector<Constraint> &constraints);
    bool isDestination(Position pos, Position dest);
    double calculateHValue(Position pos, Position dest);
    std::vector<Cell> aStarSearch(Map& map, int agentID, const std::vector<Constraint>& constraints);
    std::vector<std::vector<Cell>> getPaths(const std::vector<Constraint> &constraints, Map &map);
};