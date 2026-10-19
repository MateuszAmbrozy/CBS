#include <gtest/gtest.h>
#include "HighLevel.h"
#include "LowLevel.h"

#include <algorithm>
#include <array>
#include <functional>
#include <map>
#include <queue>
#include <stdexcept>
#include <tuple>

namespace {
Map grid(int width, int height) {
    Map map;
    for (int x = 0; x < width; ++x) {
        std::vector<Cell> column;
        for (int y = 0; y < height; ++y) column.emplace_back(Position(x, y));
        map.cells.push_back(std::move(column));
    }
    return map;
}

std::size_t cost(const std::vector<std::vector<Cell>>& paths) {
    std::size_t total = 0;
    for (const auto& path : paths) total += path.size() - 1;
    return total;
}

Position at(const std::vector<Cell>& path, std::size_t t) {
    return path[std::min(t, path.size() - 1)].position;
}

// Independent validation includes occupancy after arrival and all supplied constraints.
void validate(const Map& map, const std::vector<std::vector<Cell>>& paths,
              const std::vector<Constraint>& constraints = {}) {
    ASSERT_EQ(paths.size(), map.agents.size());
    std::size_t horizon = 0;
    for (std::size_t i = 0; i < paths.size(); ++i) {
        ASSERT_FALSE(paths[i].empty());
        EXPECT_EQ(paths[i].front().position, map.agents[i].start.position);
        EXPECT_EQ(paths[i].back().position, map.agents[i].goal.position);
        horizon = std::max(horizon, paths[i].size());
        for (std::size_t t = 0; t < paths[i].size(); ++t) {
            EXPECT_TRUE(isTraversable(map, paths[i][t].position));
            EXPECT_EQ(paths[i][t].time, static_cast<int>(t));
            if (t > 0) {
                EXPECT_LE(LowLevel::calculateHValue(at(paths[i], t - 1), at(paths[i], t)), 1);
            }
        }
        for (const auto& c : constraints) {
            if (c.agentID != map.agents[i].id) continue;
            const auto t = static_cast<std::size_t>(c.time);
            if (!c.isEdge) EXPECT_NE(at(paths[i], t), c.position);
            else EXPECT_FALSE(at(paths[i], t - 1) == c.from && at(paths[i], t) == c.position);
        }
    }
    for (std::size_t i = 0; i < paths.size(); ++i)
        for (std::size_t j = i + 1; j < paths.size(); ++j)
            for (std::size_t t = 0; t < horizon; ++t) {
                EXPECT_NE(at(paths[i], t), at(paths[j], t));
                if (t > 0) {
                    EXPECT_FALSE(at(paths[i], t - 1) == at(paths[j], t) &&
                                 at(paths[j], t - 1) == at(paths[i], t));
                }
            }
}

// Independent Dijkstra oracle over joint states. A finished agent
// stays forever; an unfinished agent can commit to its goal at any time at zero cost.
// Each move costs the number of unfinished agents, yielding sum of arrival times.
int oracle(const Map& map) {
    using State = std::array<int, 5>; // x1, y1, x2, y2, finished mask
    using Item = std::pair<int, State>;
    const auto a = map.agents[0].start.position;
    const auto b = map.agents[1].start.position;
    State start{{a.x, a.y, b.x, b.y, 0}};
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
    std::map<State, int> distance;
    distance[start] = 0;
    open.push({0, start});
    const int moves[5][2] = {{0,0},{1,0},{-1,0},{0,1},{0,-1}};
    while (!open.empty()) {
        const auto item = open.top(); open.pop();
        const int d = item.first;
        const auto s = item.second;
        if (distance.at(s) != d) continue;
        if (s[4] == 3) return d;
        auto relax = [&](State next, int weight) {
            const auto found = distance.find(next);
            if (found == distance.end() || d + weight < found->second) {
                distance[next] = d + weight;
                open.push({d + weight, next});
            }
        };
        for (int i = 0; i < 2; ++i) {
            if ((s[4] & (1 << i)) == 0 &&
                Position(s[2*i], s[2*i+1]) == map.agents[static_cast<std::size_t>(i)].goal.position) {
                auto next = s; next[4] |= 1 << i; relax(next, 0);
            }
        }
        for (const auto& m1 : moves) for (const auto& m2 : moves) {
            if (((s[4] & 1) != 0 && (m1[0] != 0 || m1[1] != 0)) ||
                ((s[4] & 2) != 0 && (m2[0] != 0 || m2[1] != 0))) continue;
            Position p(s[0]+m1[0],s[1]+m1[1]), q(s[2]+m2[0],s[3]+m2[1]);
            if (!isTraversable(map,p) || !isTraversable(map,q) || p == q ||
                (p == Position(s[2],s[3]) && q == Position(s[0],s[1]))) continue;
            relax(State{{p.x,p.y,q.x,q.y,s[4]}}, 2 - ((s[4]&1) != 0) - ((s[4]&2) != 0));
        }
    }
    return -1;
}
}

TEST(LowLevel, WaitsUntilTimedRestrictionsExpire) {
    auto map = grid(2, 2);
    map.agents.emplace_back(42, Position(0,0), Position(1,0));
    std::vector<Constraint> constraints{{42,Position(1,0),1},{42,Position(0,1),1},
                                        {42,Position(1,0),2},{42,Position(0,1),2}};
    LowLevel planner;
    auto paths = planner.getPaths(constraints,map);
    ASSERT_EQ(planner.status(), SearchStatus::Success);
    validate(map,paths,constraints);
    EXPECT_EQ(cost(paths),3u);
}

TEST(LowLevel, StartsAtGoalWithZeroBasedTime) {
    auto map=grid(1,1);
    map.agents.emplace_back(19,Position(0,0),Position(0,0));
    LowLevel planner;
    auto paths=planner.getPaths({},map);
    ASSERT_EQ(planner.status(),SearchStatus::Success);
    validate(map,paths);
    EXPECT_EQ(cost(paths),0u);
}

TEST(LowLevel, RejectsForbiddenStart) {
    auto map=grid(2,1);
    map.agents.emplace_back(5,Position(0,0),Position(1,0));
    LowLevel planner;
    EXPECT_TRUE(planner.aStarSearch(map,5,{{5,Position(0,0),0}}).empty());
    EXPECT_EQ(planner.status(),SearchStatus::NoSolution);
}

TEST(LowLevel, LeavesAndRevisitsGoalForFutureConstraint) {
    auto map=grid(2,1);
    map.agents.emplace_back(5,Position(0,0),Position(0,0));
    const std::vector<Constraint> constraints{{5,Position(0,0),2}};
    LowLevel planner;
    auto paths=planner.getPaths(constraints,map);
    ASSERT_EQ(planner.status(),SearchStatus::Success);
    validate(map,paths,constraints);
    EXPECT_EQ(cost(paths),3u);
}

TEST(LowLevel, RespectsDirectedEdgesAndFutureWaitConstraints) {
    auto map=grid(2,1);
    map.agents.emplace_back(7,Position(0,0),Position(1,0));
    const std::vector<Constraint> constraints{Constraint::edge(7,Position(0,0),Position(1,0),1),
        Constraint::edge(7,Position(1,0),Position(1,0),3)};
    LowLevel planner;
    auto paths=planner.getPaths(constraints,map);
    ASSERT_EQ(planner.status(),SearchStatus::Success);
    validate(map,paths,constraints);
    EXPECT_EQ(cost(paths),3u);
}

TEST(LowLevel, DetectsDisconnectedGoal) {
    auto map=grid(3,1);
    map.cells[1][0].isObstacle=true;
    map.agents.emplace_back(0,Position(0,0),Position(2,0));
    LowLevel planner;
    EXPECT_TRUE(planner.aStarSearch(map,0,{}).empty());
    EXPECT_EQ(planner.status(),SearchStatus::NoSolution);
}

TEST(Validation, RejectsMalformedInputs) {
    EXPECT_THROW(HighLevel(Map{}),std::invalid_argument);
    auto map=grid(2,2);
    map.cells[1].pop_back();
    EXPECT_THROW(HighLevel{map},std::invalid_argument);
    map=grid(2,2);
    map.agents.emplace_back(8,Position(-1,0),Position(1,1));
    EXPECT_THROW(HighLevel{map},std::invalid_argument);
    map.agents[0].start=Cell(Position(0,0));
    map.cells[1][1].isObstacle=true;
    EXPECT_THROW(HighLevel{map},std::invalid_argument);
    map.cells[1][1].isObstacle=false;
    map.agents.emplace_back(8,Position(1,0),Position(0,1));
    EXPECT_THROW(HighLevel{map},std::invalid_argument);
    map.agents.pop_back();
    EXPECT_THROW(HighLevel(map,{{9,Position(0,0),1}}),std::invalid_argument);
    EXPECT_THROW(HighLevel(map,{{8,Position(0,0),-1}}),std::invalid_argument);
    EXPECT_THROW(HighLevel(map,{Constraint::edge(8,Position(0,0),Position(1,1),1)}),std::invalid_argument);
    EXPECT_THROW(HighLevel(map,{Constraint::edge(8,Position(0,0),Position(1,0),0)}),std::invalid_argument);
    LowLevel planner;
    EXPECT_THROW(planner.aStarSearch(map,99,{}),std::invalid_argument);
}

TEST(Conflicts, ExtendsGoalOccupancyAndHandlesShortPaths) {
    const std::vector<Cell> a{Cell(Position(0,0))};
    const std::vector<Cell> b{Cell(Position(1,0)),Cell(Position(0,0))};
    EXPECT_TRUE(ConstraintTree::detectConflicts(a,b));
    EXPECT_FALSE(ConstraintTree::hasEdgeConflict({},a));
    EXPECT_FALSE(ConstraintTree::hasEdgeConflict(a,a));
    EXPECT_FALSE(ConstraintTree::detectConflicts({},a));
    TreeNode node;
    node.solution={a,b};
    Conflict conflict;
    ASSERT_TRUE(ConstraintTree::getFirstConflict(node,conflict));
    EXPECT_FALSE(conflict.isEdge);
    EXPECT_EQ(conflict.time,1);
}

TEST(Conflicts, ReportsSwapAtArrivalTime) {
    TreeNode node;
    node.solution={{Cell(Position(0,0)),Cell(Position(1,0))},
                   {Cell(Position(1,0)),Cell(Position(0,0))}};
    EXPECT_TRUE(ConstraintTree::hasEdgeConflict(node));
    Conflict conflict;
    ASSERT_TRUE(ConstraintTree::getFirstConflict(node,conflict));
    EXPECT_TRUE(conflict.isEdge);
    EXPECT_EQ(conflict.time,1);
    EXPECT_EQ(conflict.pos1,Position(0,0));
    EXPECT_EQ(conflict.pos2,Position(1,0));
}

TEST(HighLevel, SecondAgentMustYieldWhenFirstBranchIsInfeasible) {
    auto map=grid(3,2);
    map.agents.emplace_back(10,Position(0,0),Position(2,0));
    map.agents.emplace_back(20,Position(1,1),Position(1,0));
    // At t=1 agent 10 cannot wait or detour; agent 20 must yield.
    const std::vector<Constraint> constraints{{10,Position(0,0),1},{10,Position(0,1),1}};
    HighLevel solver(map,constraints);
    const auto paths=solver.solve();
    ASSERT_EQ(solver.status(),SearchStatus::Success);
    validate(map,paths,constraints);
    EXPECT_EQ(cost(paths),4u);
}

TEST(HighLevel, ResolvesSwapUsingBypass) {
    auto map=grid(2,2);
    map.agents.emplace_back(100,Position(0,0),Position(1,0));
    map.agents.emplace_back(-7,Position(1,0),Position(0,0));
    HighLevel solver(map);
    auto paths=solver.solve();
    ASSERT_EQ(solver.status(),SearchStatus::Success);
    validate(map,paths);
    EXPECT_EQ(cost(paths),4u);
}

TEST(HighLevel, RepeatedAndIndependentSolvers) {
    auto a=grid(3,2);
    a.agents.emplace_back(1,Position(0,0),Position(2,0));
    a.agents.emplace_back(2,Position(2,0),Position(0,0));
    HighLevel first(a);
    auto b=grid(1,1);
    b.agents.emplace_back(0,Position(0,0),Position(0,0));
    HighLevel second(b);
    auto paths=first.solve();
    ASSERT_EQ(first.status(),SearchStatus::Success);
    validate(a,paths);
    const auto firstCost=cost(paths);
    validate(b,second.solve());
    paths=first.solve();
    validate(a,paths);
    EXPECT_EQ(cost(paths),firstCost);
}

TEST(HighLevel, ReportsUnavoidableStartAndGoalCollisions) {
    auto map=grid(2,2);
    map.agents.emplace_back(0,Position(0,0),Position(1,0));
    map.agents.emplace_back(1,Position(0,0),Position(1,1));
    HighLevel first(map);
    EXPECT_TRUE(first.solve().empty());
    EXPECT_EQ(first.status(),SearchStatus::NoSolution);
    map.agents[1].start=Cell(Position(0,1));
    map.agents[1].goal=map.agents[0].goal;
    HighLevel second(map);
    EXPECT_TRUE(second.solve().empty());
    EXPECT_EQ(second.status(),SearchStatus::NoSolution);
}

TEST(HighLevel, HandlesZeroAgents) {
    HighLevel solver(grid(1,1));
    EXPECT_TRUE(solver.solve().empty());
    EXPECT_EQ(solver.status(),SearchStatus::Success);
}

TEST(Limits, DistinguishesResourceLimitsFromInfeasibility) {
    auto map=grid(2,2);
    map.agents.emplace_back(0,Position(0,0),Position(1,0));
    SearchLimits limits;
    limits.maxLowLevelStates=1;
    HighLevel lowBound(map,{},limits);
    EXPECT_TRUE(lowBound.solve().empty());
    EXPECT_EQ(lowBound.status(),SearchStatus::ResourceLimit);
    limits=SearchLimits{};
    limits.maxHighLevelNodes=0;
    HighLevel highBound(map,{},limits);
    EXPECT_TRUE(highBound.solve().empty());
    EXPECT_EQ(highBound.status(),SearchStatus::ResourceLimit);
    // Unsatisfiable swaps need a search bound rather than an infinite CBS tree.
    map=grid(2,1);
    map.agents.emplace_back(0,Position(0,0),Position(1,0));
    map.agents.emplace_back(1,Position(1,0),Position(0,0));
    limits.maxHighLevelNodes=20;
    HighLevel impossible(map,{},limits);
    EXPECT_TRUE(impossible.solve().empty());
    EXPECT_EQ(impossible.status(),SearchStatus::ResourceLimit);
}

TEST(Optimality, MatchesJointDijkstraOnEveryTwoAgentTwoByTwoScenario) {
    for (int s1=0;s1<4;++s1) for (int s2=0;s2<4;++s2) {
        if(s1==s2) continue;
        for(int g1=0;g1<4;++g1) for(int g2=0;g2<4;++g2) {
            if(g1==g2) continue;
            SCOPED_TRACE(::testing::Message()<<s1<<','<<s2<<" -> "<<g1<<','<<g2);
            auto map=grid(2,2);
            map.agents.emplace_back(10,Position(s1/2,s1%2),Position(g1/2,g1%2));
            map.agents.emplace_back(20,Position(s2/2,s2%2),Position(g2/2,g2%2));
            const int expected=oracle(map);
            ASSERT_GE(expected,0);
            HighLevel solver(map);
            const auto paths=solver.solve();
            ASSERT_EQ(solver.status(),SearchStatus::Success);
            validate(map,paths);
            EXPECT_EQ(cost(paths),static_cast<std::size_t>(expected));
        }
    }
}

TEST(Optimality, MatchesJointDijkstraOnRectangularGridsWithObstacles) {
    int checked = 0;
    for (int obstacle : {-1, 0, 2}) {
        for (int s1=0;s1<6;++s1) for (int s2=0;s2<6;++s2) {
            if (s1==s2 || s1==obstacle || s2==obstacle) continue;
            for (int g1=0;g1<6;++g1) for (int g2=0;g2<6;++g2) {
                if (g1==g2 || g1==obstacle || g2==obstacle) continue;
                auto map=grid(3,2);
                if (obstacle>=0) map.cells[obstacle/2][obstacle%2].isObstacle=true;
                map.agents.emplace_back(10,Position(s1/2,s1%2),Position(g1/2,g1%2));
                map.agents.emplace_back(20,Position(s2/2,s2%2),Position(g2/2,g2%2));
                const int expected=oracle(map);
                // Only feasible scenarios have an optimal cost to compare.
                if (expected<0) continue;
                SCOPED_TRACE(::testing::Message()<<"obstacle="<<obstacle<<' '<<s1<<','<<s2<<" -> "<<g1<<','<<g2);
                HighLevel solver(map);
                auto paths=solver.solve();
                ASSERT_EQ(solver.status(),SearchStatus::Success);
                validate(map,paths);
                EXPECT_EQ(cost(paths),static_cast<std::size_t>(expected));
                ++checked;
            }
        }
    }
    EXPECT_GT(checked,900);
}

TEST(HighLevel, CoordinatesThreeAgentsAroundObstacles) {
    auto map=grid(3,3);
    map.cells[1][1].isObstacle=true;
    map.agents.emplace_back(11,Position(0,0),Position(2,2));
    map.agents.emplace_back(22,Position(2,0),Position(0,2));
    map.agents.emplace_back(33,Position(0,2),Position(2,0));
    HighLevel solver(map);
    auto paths=solver.solve();
    ASSERT_EQ(solver.status(),SearchStatus::Success);
    validate(map,paths);
}

TEST(HighLevel, InfeasibleRootIsNotReportedAsSuccess) {
    auto map=grid(3,1);
    map.cells[1][0].isObstacle=true;
    map.agents.emplace_back(0,Position(0,0),Position(2,0));
    HighLevel solver(map);
    EXPECT_TRUE(solver.solve().empty());
    EXPECT_EQ(solver.status(),SearchStatus::NoSolution);
}
