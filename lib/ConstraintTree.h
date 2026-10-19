#pragma once

#include "util.h"
#include <memory>
#include <vector>

struct TreeNode {
    std::vector<Constraint> constraints;
    std::vector<std::vector<Cell>> solution;
    std::size_t cost = 0;
    void updateCost();
};

struct CompareNodes {
    bool operator()(const std::shared_ptr<TreeNode>& a, const std::shared_ptr<TreeNode>& b) const {
        return a->cost > b->cost;
    }
};

// Stateless collision checks: completed paths remain at their final positions.
class ConstraintTree {
public:
    static bool detectConflicts(const std::vector<Cell>& route1, const std::vector<Cell>& route2);
    static bool detectConflicts(const TreeNode& node);
    static bool hasEdgeConflict(const std::vector<Cell>& route1, const std::vector<Cell>& route2);
    static bool hasEdgeConflict(const TreeNode& node);
    static bool getFirstConflict(const TreeNode& node, Conflict& conflict);
};
