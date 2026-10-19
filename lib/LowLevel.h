#pragma once

#include "util.h"

class LowLevel {
public:
    explicit LowLevel(SearchLimits limits = SearchLimits()) : limits_(limits) {}
    std::vector<Cell> aStarSearch(const Map& map, int agentID,
                                  const std::vector<Constraint>& constraints);
    std::vector<std::vector<Cell>> getPaths(const std::vector<Constraint>& constraints,
                                           const Map& map);
    SearchStatus status() const { return status_; }
    static bool isConstraint(const Constraint& constraint, const std::vector<Constraint>& constraints);
    static bool isDestination(Position position, Position destination) { return position == destination; }
    static int calculateHValue(Position position, Position destination);

private:
    SearchLimits limits_;
    SearchStatus status_ = SearchStatus::NotStarted;
};
