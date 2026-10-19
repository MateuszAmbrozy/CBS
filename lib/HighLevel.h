#pragma once

#include "ConstraintTree.h"

class HighLevel {
public:
    HighLevel(const Map& map, const std::vector<Constraint>& constraints = {},
              SearchLimits limits = SearchLimits());
    std::vector<std::vector<Cell>> solve();
    SearchStatus status() const { return status_; }

private:
    Map map_;
    std::vector<Constraint> constraints_;
    SearchLimits limits_;
    SearchStatus status_ = SearchStatus::NotStarted;
};
