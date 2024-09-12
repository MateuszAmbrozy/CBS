#include "LowLevel.h"
#include <cmath>
#include <queue>
#include <iostream>
// Konstruktor
LowLevel::LowLevel() {}

// Destruktor
LowLevel::~LowLevel() {}

bool LowLevel::isBlocked(const Map& map, Position pos, const std::vector<Constraint>& constraints, int currentTime) {
    // for (const Constraint& c : constraints) {
    //     if (c.position == pos && c.time == currentTime + waitTime) {
    //         return true; // Jeśli pozycja jest zablokowana w danym momencie
    //     }
    // }
    return map.cells[pos.x][pos.y].isObstacle;
}

bool LowLevel::isValid(const Map& map, Position pos, int currentTime, const std::vector<Constraint>& constraints) {
    return (pos.x >= 0) && (pos.x < map.cells.size()) && 
           (pos.y >= 0) && (pos.y < map.cells[0].size()) && 
           !isBlocked(map, pos, constraints, currentTime);
}

bool LowLevel::isConstraint(Constraint constraint, const std::vector<Constraint> &constraints)
{
    for(Constraint c : constraints)
    {
        if(constraint == c)
            return true;
    }
    return false;
}

// Sprawdza, czy osiągnięto cel
bool LowLevel::isDestination(Position pos, Position dest) {
    return pos == dest;
}

// Funkcja heurystyczna - odległość Manhattan
double LowLevel::calculateHValue(Position pos, Position dest) {
    return abs(pos.x - dest.x) + abs(pos.y - dest.y);
}

std::vector<Cell> LowLevel::tracePath(Map& map, Cell* destCell) {
    std::vector<Cell> path;
    Cell* cell = destCell;
    
    while (cell != nullptr) {
        path.push_back(*cell);
        cell = cell->parent;
    }

    std::reverse(path.begin(), path.end());
    return path;
}
std::vector<Cell> LowLevel::aStarSearch(Map& map, int agentID, const std::vector<Constraint>& constraints) {
    std::vector<Cell> emptyPath;
    Position src = map.agents[agentID].start.position;
    Position dest = map.agents[agentID].goal.position;

    if (!isValid(map, src, 0, constraints) || !isValid(map, dest, 0, constraints)) {
        std::cout << "Źródło lub cel są poza granicami mapy.\n";
        return emptyPath;
    }

    if (isBlocked(map, src, constraints, 0) || isBlocked(map, dest, constraints, 0)) {
        std::cout << "Źródło lub cel są zablokowane.\n";
        return emptyPath;
    }

    if (isDestination(src, dest)) {
        std::cout << "Już jesteś na miejscu.\n";
        return emptyPath;
    }

    std::vector<std::vector<bool>> closedList(map.cells.size(), std::vector<bool>(map.cells[0].size(), false));
    std::priority_queue<Cell, std::vector<Cell>, CompareCells> openList;

    Cell startCell = map.agents[agentID].start;
    startCell.g = 0;
    startCell.h = calculateHValue(src, dest);
    startCell.f = startCell.g + startCell.h;
    startCell.parent = nullptr;
    startCell.time = 1;

    openList.push(startCell);

    const int maxIterations = 10000;  // Limit iteracji
    int iterations = 0;

    while (!openList.empty()) {
        iterations++;
        if (iterations > maxIterations) {
            std::cout << "Przekroczono limit iteracji, brak ścieżki.\n";
            return emptyPath;  // Przerwij, jeśli przekroczono limit iteracji
        }

        Cell currentCell = openList.top();
        openList.pop();

        if (isDestination(currentCell.position, dest)) {
            //std::cout << "Znaleziono ścieżkę\n" << "Total Time: " << currentCell.time << std::endl;
            return tracePath(map, &currentCell);
        }

        closedList[currentCell.position.x][currentCell.position.y] = true;

        for (int addX = -1; addX <= 1; addX++) {
            for (int addY = -1; addY <= 1; addY++) {
                if (std::abs(addX) == std::abs(addY)) 
                    continue;

                Position neighborPos(currentCell.position.x + addX, currentCell.position.y + addY);

                if (isValid(map, neighborPos, currentCell.time, constraints) && 
                    !isConstraint(Constraint(agentID, neighborPos, currentCell.time), constraints)) 
                {
                    if (!closedList[neighborPos.x][neighborPos.y] && !isBlocked(map, neighborPos, constraints, currentCell.time)) {
                        double gNew = currentCell.g + 1;
                        double hNew = calculateHValue(neighborPos, dest);
                        double fNew = gNew + hNew;

                        Cell neighborCell(neighborPos, gNew, hNew, fNew, currentCell.time + 1, new Cell(currentCell));
                        openList.push(neighborCell);
                    }
                }
            }
        }

        // Dodaj opcję czekania w obecnej pozycji
        Position waitPos = currentCell.position;
        if (!isConstraint(Constraint(agentID, waitPos, currentCell.time), constraints)) {
            Cell waitCell(waitPos, currentCell.g + 1, calculateHValue(waitPos, dest), currentCell.g + 1 + calculateHValue(waitPos, dest), currentCell.time + 1, new Cell(currentCell));
            openList.push(waitCell);
        }
    }

    std::cout << "Nie znaleziono ścieżki do celu.\n";
    return emptyPath;
}



std::vector<std::vector<Cell>> LowLevel::getPaths(const std::vector<Constraint> &constraints, Map &map) {
    std::vector<std::vector<Cell>> optimalPaths;

    // Solve pathfinding for each agent
    for (int k = 0; k < map.agents.size(); k++) {
        std::vector<Cell> path = aStarSearch(map, k, constraints);
        if (!path.empty()) {
            optimalPaths.push_back(path);
        } else {
            std::cout << "No path found for agent " << k << std::endl;
            return {};
        }
    }

    return optimalPaths;
}
