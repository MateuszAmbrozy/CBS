#include "HighLevel.h"
#include <exception>
#include <iostream>

Map createMap() {
    Map map;
    for (int x = 0; x < 4; ++x) {
        std::vector<Cell> column;
        for (int y = 0; y < 4; ++y) column.emplace_back(Position(x, y));
        map.cells.push_back(std::move(column));
    }
    for (const auto& p : {Position(0, 0), Position(0, 3), Position(3, 0), Position(3, 3)})
        map.cells[p.x][p.y].isObstacle = true;
    map.agents.emplace_back(1, Position(1, 0), Position(2, 3));
    map.agents.emplace_back(2, Position(0, 1), Position(3, 2));
    return map;
}

int main() {
    try {
        HighLevel solver(createMap());
        const auto paths = solver.solve();
        if (solver.status() != SearchStatus::Success) {
            std::cerr << (solver.status() == SearchStatus::ResourceLimit
                ? "Search resource limit reached.\n" : "No solution exists.\n");
            return 1;
        }
        std::size_t cost = 0;
        for (std::size_t i = 0; i < paths.size(); ++i) {
            std::cout << "Agent " << i + 1 << '\n';
            for (const auto& cell : paths[i])
                std::cout << '(' << cell.position.x << ", " << cell.position.y << "), t: " << cell.time << '\n';
            cost += paths[i].size() - 1;
        }
        std::cout << "Sum of costs: " << cost << '\n';
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
