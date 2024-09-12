#pragma once
#include "util.h"
#include "LowLevel.h"
#include <vector>
#include <unordered_map>
#include <memory>
#include <queue>

class TreeNode: public std::enable_shared_from_this<TreeNode> {
public:
    std::vector<Constraint> constraints;
    std::vector<std::vector<Cell>> solution; 
    double cost;
    std::shared_ptr<TreeNode> parent;

    TreeNode(std::shared_ptr<TreeNode> parent = nullptr)
        : cost(DBL_MAX), parent(parent) {}

    // Function to add a constraint and create a new node
    std::shared_ptr<TreeNode> addConstraint(const Constraint& constraint) {
        auto child = std::make_shared<TreeNode>(shared_from_this());
        child->constraints = this->constraints;
        child->constraints.push_back(constraint);
        return child;
    }

    void updateSolution(Map& map,const std::vector<std::vector<Cell>>& solution, double cost) {
        LowLevel solver;
        this->solution = solver.getPaths(constraints, map);
        this->cost = cost;
    }
    void updateCost()
    {
        auto cost = 0;
        for (auto route : solution) {
            cost += route.size();
        }
        this->cost = cost;
    }
};

struct CompareNodes //comaprition to priority_queue
{
    bool operator()(const std::shared_ptr<TreeNode>& a, const std::shared_ptr<TreeNode>& b) const {
        return a->cost > b->cost;
    }
};

class ConstraintTree {


private:
    static Map map;
public:
    ConstraintTree(Map& map)
        {
            this->map = map;
        }

    static std::vector<std::vector<Cell>> findInitialSolution(const std::vector<Constraint>& constraints) {
        LowLevel solver;
        return solver.getPaths(constraints, map);
    }
    static bool detectConflicts(const std::vector<Cell> &route1, const std::vector<Cell> &route2) {
        auto min_index = std::min(route1.size(), route2.size());
        for (int i = 0; i < min_index; i++) {
            if (route1[i] == route2[i])
                return true;
        }
        return false;
    }

    static bool detectConflicts( const TreeNode& node) {
        auto solutions = node.solution;
        for (int i = 0; i < solutions.size(); i++) {
            for (int j = i + 1; j < solutions.size(); j++) {
                if (detectConflicts(solutions[i], solutions[j]))
                    return true;
            }
        }
        return false;
    }

    static bool hasEdgeConflict(const std::vector<Cell> &route1, const std::vector<Cell> &route2) {
        auto min_route_size = std::min(route1.size(), route2.size()) - 1;
        for (int i = 0; i < min_route_size; i++) {
            if (route1[i] == route2[i + 1] && route1[i + 1] == route2[i])
                return true;
        }
        return false;
    }

    static bool hasEdgeConflict(const TreeNode &node) {
        auto solutions = node.solution;

        for (int i = 0; i < solutions.size(); i++) {
            for (int j = i + 1; j < solutions.size(); j++) {
                if (hasEdgeConflict(solutions[i], solutions[j]))
                    return true;
            }
        }
        return false;
    }

    static Conflict getFirstConflict(const TreeNode &P){
	auto solutions = P.solution;

	// Look for normal conflicts first
	for (int i = 0; i < solutions.size(); i++) {
		for (int j = i + 1; j < solutions.size(); j++) {
			auto min_index = std::min(solutions[i].size(), solutions[j].size());
			for (int k = 0; k < min_index; k++) {
				if (solutions[i][k] == solutions[j][k])
					return Conflict(i, j, k, solutions[i][k], solutions[i][k]);
			}
		}
	}

	// Look for Edge Conflicts if there is no normal conflicts
	for (int i = 0; i < solutions.size(); i++) {
		for (int j = i + 1; j < solutions.size(); j++) {
			auto min_index = std::min(solutions[i].size(), solutions[j].size()) - 1;
			for (int k = 0; k < min_index; k++) {
				auto route1 = solutions[i];
				auto route2 = solutions[j];
				if (route1[k] == route2[k + 1] && route1[k + 1] == route2[k])
					return Conflict(i, j, k, route1[k + 1], route2[k + 1]);
			}
		}
	}
	// Trivial. It is assured that this method is called only when there is conflict
	return Conflict(0, 0, 0, Cell(0, 0), Cell(0, 0));
}

};