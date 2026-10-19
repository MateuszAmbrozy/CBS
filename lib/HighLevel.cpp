#include "HighLevel.h"
#include "LowLevel.h"

#include <algorithm>
#include <queue>
#include <set>
#include <tuple>

namespace {
using ConstraintKey = std::tuple<int, int, int, int, bool, int, int>;
std::vector<ConstraintKey> signature(const std::vector<Constraint>& constraints) {
    std::vector<ConstraintKey> result;
    for (const auto& c : constraints)
        result.emplace_back(c.agentID, c.time, c.position.x, c.position.y, c.isEdge,
                            c.isEdge ? c.from.x : 0, c.isEdge ? c.from.y : 0);
    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end()), result.end());
    return result;
}
}

HighLevel::HighLevel(const Map& map, const std::vector<Constraint>& constraints, SearchLimits limits)
    : map_(map), constraints_(constraints), limits_(limits) {
    validateProblem(map_, constraints_);
}

std::vector<std::vector<Cell>> HighLevel::solve() {
    status_ = SearchStatus::NoSolution;
    // These collisions are unavoidable under stay-at-goal semantics.
    for (std::size_t i = 0; i < map_.agents.size(); ++i)
        for (std::size_t j = i + 1; j < map_.agents.size(); ++j)
            if (map_.agents[i].start == map_.agents[j].start || map_.agents[i].goal == map_.agents[j].goal)
                return {};
    LowLevel planner(limits_);
    auto root = std::make_shared<TreeNode>();
    root->constraints = constraints_;
    root->solution = planner.getPaths(constraints_, map_);
    if (planner.status() != SearchStatus::Success) { status_ = planner.status(); return {}; }
    root->updateCost();
    std::priority_queue<std::shared_ptr<TreeNode>, std::vector<std::shared_ptr<TreeNode>>, CompareNodes> open;
    open.push(root);
    std::set<std::vector<ConstraintKey>> seen;
    seen.insert(signature(constraints_));
    std::size_t expanded = 0;
    while (!open.empty()) {
        if (expanded >= limits_.maxHighLevelNodes) { status_ = SearchStatus::ResourceLimit; return {}; }
        ++expanded;
        auto node = open.top();
        open.pop();
        Conflict conflict;
        if (!ConstraintTree::getFirstConflict(*node, conflict)) {
            status_ = SearchStatus::Success;
            return node->solution;
        }
        for (int branch = 0; branch < 2; ++branch) {
            const auto index = branch == 0 ? conflict.agent1 : conflict.agent2;
            const int id = map_.agents[index].id;
            auto constraint = conflict.isEdge
                ? Constraint::edge(id, branch == 0 ? conflict.pos1 : conflict.pos2,
                                   branch == 0 ? conflict.pos2 : conflict.pos1, conflict.time)
                : Constraint(id, conflict.pos1, conflict.time);
            auto child = std::make_shared<TreeNode>(*node);
            child->constraints.push_back(constraint);
            if (!seen.insert(signature(child->constraints)).second) continue;
            auto path = planner.aStarSearch(map_, id, child->constraints);
            // An unresolved bounded search cannot be discarded as proven infeasible.
            if (planner.status() == SearchStatus::ResourceLimit) {
                status_ = SearchStatus::ResourceLimit;
                return {};
            }
            if (planner.status() != SearchStatus::Success) continue;
            child->solution[index] = std::move(path);
            child->updateCost();
            open.push(std::move(child));
        }
    }
    return {};
}
