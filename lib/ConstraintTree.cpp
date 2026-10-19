#include "ConstraintTree.h"

#include <algorithm>
#include <stdexcept>

namespace {
Position at(const std::vector<Cell>& path, std::size_t time) {
    return path[std::min(time, path.size() - 1)].position;
}
}

void TreeNode::updateCost() {
    cost = 0;
    for (const auto& path : solution) {
        if (path.empty()) throw std::logic_error("A solution must contain nonempty paths.");
        cost += path.size() - 1;
    }
}

bool ConstraintTree::detectConflicts(const std::vector<Cell>& a, const std::vector<Cell>& b) {
    if (a.empty() || b.empty()) return false;
    for (std::size_t t = 0; t < std::max(a.size(), b.size()); ++t)
        if (at(a, t) == at(b, t)) return true;
    return false;
}

bool ConstraintTree::hasEdgeConflict(const std::vector<Cell>& a, const std::vector<Cell>& b) {
    if (a.empty() || b.empty()) return false;
    for (std::size_t t = 1; t < std::max(a.size(), b.size()); ++t)
        if (at(a, t - 1) != at(a, t) && at(a, t - 1) == at(b, t) && at(a, t) == at(b, t - 1))
            return true;
    return false;
}

bool ConstraintTree::detectConflicts(const TreeNode& node) {
    for (std::size_t i = 0; i < node.solution.size(); ++i)
        for (std::size_t j = i + 1; j < node.solution.size(); ++j)
            if (detectConflicts(node.solution[i], node.solution[j])) return true;
    return false;
}

bool ConstraintTree::hasEdgeConflict(const TreeNode& node) {
    for (std::size_t i = 0; i < node.solution.size(); ++i)
        for (std::size_t j = i + 1; j < node.solution.size(); ++j)
            if (hasEdgeConflict(node.solution[i], node.solution[j])) return true;
    return false;
}

bool ConstraintTree::getFirstConflict(const TreeNode& node, Conflict& conflict) {
    std::size_t horizon = 0;
    for (const auto& path : node.solution) {
        if (path.empty()) throw std::logic_error("Cannot inspect an incomplete solution.");
        horizon = std::max(horizon, path.size());
    }
    for (std::size_t t = 0; t < horizon; ++t) {
        for (std::size_t i = 0; i < node.solution.size(); ++i) {
            for (std::size_t j = i + 1; j < node.solution.size(); ++j) {
                auto a = at(node.solution[i], t);
                auto b = at(node.solution[j], t);
                if (a == b) {
                    conflict = {i, j, static_cast<int>(t), a, b, false};
                    return true;
                }
                if (t > 0 && at(node.solution[i], t - 1) == b && at(node.solution[j], t - 1) == a) {
                    conflict = {i, j, static_cast<int>(t), b, a, true};
                    return true;
                }
            }
        }
    }
    return false;
}
