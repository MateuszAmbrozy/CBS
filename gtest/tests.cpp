#include <gtest/gtest.h>
#include "../lib/HighLevel.h"
#include "../lib/LowLevel.h"
#include "../lib/util.h"
#include "../lib/ConstraintTree.h"


class LowLevelTest : public ::testing::Test {
protected:
    LowLevel lowLevel;
    Map map;
    std::vector<Constraint> constraints;
    const int mapWidth = 4;
    const int mapHeight = 4;
    

    void SetUp() override {

        std::cout<<"setting up the tests\n";
        map.cells.resize(mapWidth, std::vector<Cell>(mapHeight, Cell(Position(0, 0), 0, 0)));
        
        // Ustawianie pozycji dla każdej komórki mapy
        for (int i = 0; i < mapWidth; ++i) {
            for (int j = 0; j < mapHeight; ++j) {
                map.cells[i][j] = Cell(Position(i, j));
            }
        }
    }

};


TEST_F(LowLevelTest, AStartSearch)
{
        Position startAgent1(0, 0); // D1
        Position goalAgent1(2, 0);  // B3
        Agent agent1(0, startAgent1, goalAgent1);
        map.agents.push_back(agent1);

        constraints.push_back(Constraint(0, Position(1, 0), 1));
        constraints.push_back(Constraint(0, Position(0, 1), 1));
        constraints.push_back(Constraint(0, Position(1, 0), 2));
        constraints.push_back(Constraint(0, Position(0, 1), 2));

        std::vector<Cell> result = lowLevel.aStarSearch(map, 0, constraints);

        std::vector<std::vector<int>> expectedPath = {
        {0, 0},
        {0, 0},
        {0, 0},
        {1, 0}, 
        {2, 0}
    };
        // Check if the result path is the same as the expected path
    ASSERT_EQ(result.size(), expectedPath.size()) << "Path size mismatch";

    for (size_t i = 0; i < result.size(); ++i) {
        std::cout<<"result: ("<<result[i].position.x << ", " << result[i].position.y << ") | expected: (" << expectedPath[i][0]<< "," << expectedPath[i][1] << ")\n";
        EXPECT_EQ(result[i].position.x, expectedPath[i][0]) << "Mismatch at index " << i;
        EXPECT_EQ(result[i].position.y, expectedPath[i][1]) << "Mismatch at index " << i;
    }

};

TEST_F(LowLevelTest, GetPaths) {
    // Tworzenie agentów
    Position startAgent1(1, 0); // Początkowa pozycja agenta 1
    Position goalAgent1(2, 3);  // Docelowa pozycja agenta 1
    Agent agent1(0, startAgent1, goalAgent1);
    map.agents.push_back(agent1);

    Position startAgent2(0, 1); // Początkowa pozycja agenta 2
    Position goalAgent2(3, 2);  // Docelowa pozycja agenta 2
    Agent agent2(1, startAgent2, goalAgent2);
    map.agents.push_back(agent2);

    constraints.push_back(Constraint(0, Position(1, 1), 1)); // Przykładowe ograniczenia
    constraints.push_back(Constraint(1, Position(2, 2), 2));

    std::vector<std::vector<Cell>> paths = lowLevel.getPaths(constraints, map);

    ASSERT_EQ(paths.size(), 2) << "Expected paths for 2 agents";

    // Oczekiwane ścieżki (przykłady, mogą wymagać dostosowania do rzeczywistej logiki)
    std::vector<std::vector<std::vector<int>>> expectedPaths = {
        {{1, 0}, {2, 0}, {2, 1}, {2, 2}, {2, 3}}, // Ścieżka agenta 1
        {{0, 1}, {1, 1}, {1, 2}, {2, 2}, {3, 2}}  // Ścieżka agenta 2
    };

    for (int i = 0; i < 2; ++i) {
        ASSERT_EQ(paths[i].size(), expectedPaths[i].size()) << "Path size mismatch for agent " << i;

        for (size_t j = 0; j < paths[i].size(); ++j) {
            std::cout << "result: (" << paths[i][j].position.x << ", " << paths[i][j].position.y << ") | expected: ("
                      << expectedPaths[i][j][0] << "," << expectedPaths[i][j][1] << ")\n";
            EXPECT_EQ(paths[i][j].position.x, expectedPaths[i][j][0]) << "Mismatch at index " << j << " for agent " << i;
            EXPECT_EQ(paths[i][j].position.y, expectedPaths[i][j][1]) << "Mismatch at index " << j << " for agent " << i;
        }
    }
};



class HighLevelTest : public ::testing::Test {
protected:
    Map map;
    std::vector<Constraint> constraints;

    void SetUp() override {
        map.cells.resize(4, std::vector<Cell>(4, Cell(Position(0, 0), 0, 0)));
        
        // Inicjalizacja pozycji komórek mapy
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                map.cells[i][j] = Cell(Position(i, j));
            }
        }
    }

    void createAgentAndConstraints() 
    {
        Position start1(0, 0);
        Position goal1(3, 3);
        Agent agent1(0, start1, goal1);
        map.agents.push_back(agent1);

        Position start2(0, 3);
        Position goal2(3, 0);
        Agent agent2(1, start2, goal2);
        map.agents.push_back(agent2);

        // constraints.push_back(Constraint(0, Position(1, 1), 1));
        // constraints.push_back(Constraint(1, Position(1, 2), 2));
    }
};

TEST_F(HighLevelTest, SolveWithoutConflicts) {
    // Ustawienie agentów i ograniczeń
    Position start1(0, 0);
    Position goal1(3, 0);
    Agent agent1(0, start1, goal1);
    map.agents.push_back(agent1);

    Position start2(0, 3);
    Position goal2(3, 3);
    Agent agent2(1, start2, goal2);
    map.agents.push_back(agent2);

    constraints.clear();  // Brak dodatkowych ograniczeń
    HighLevel highLevel(map, constraints);

    std::vector<std::vector<Cell>> solution = highLevel.solve();

    // Oczekiwane ścieżki dla agentów
    std::vector<Cell> expectedPath1 = {
        Cell(Position(0, 0)), Cell(Position(1, 0)), Cell(Position(2, 0)), Cell(Position(3, 0))
    };

    std::vector<Cell> expectedPath2 = {
        Cell(Position(0, 3)), Cell(Position(1, 3)), Cell(Position(2, 3)), Cell(Position(3, 3))
    };

    ASSERT_EQ(solution.size(), 2);  // Oczekujemy dwóch ścieżek, po jednej dla każdego agenta

    // Sprawdzanie ścieżki agenta 0
    ASSERT_EQ(solution[0].size(), expectedPath1.size()) << "Agent 0 path size mismatch";
    for (size_t i = 0; i < expectedPath1.size(); ++i) {
        EXPECT_EQ(solution[0][i].position.x, expectedPath1[i].position.x) << "Mismatch at index " << i << " for Agent 0";
        EXPECT_EQ(solution[0][i].position.y, expectedPath1[i].position.y) << "Mismatch at index " << i << " for Agent 0";
    }

    // Sprawdzanie ścieżki agenta 1
    ASSERT_EQ(solution[1].size(), expectedPath2.size()) << "Agent 1 path size mismatch";
    for (size_t i = 0; i < expectedPath2.size(); ++i) {
        EXPECT_EQ(solution[1][i].position.x, expectedPath2[i].position.x) << "Mismatch at index " << i << " for Agent 1";
        EXPECT_EQ(solution[1][i].position.y, expectedPath2[i].position.y) << "Mismatch at index " << i << " for Agent 1";
    }
}
TEST_F(HighLevelTest, NoSolution) {
    //To jest ogólnie do poprawki bo A* sprawdza że jak nie znajdzie drogi dla agenta po 10000 iteracjach to zwraca {} także słabo
    Position start1(0, 0);
    Position goal1(3, 3);
    Agent agent1(0, start1, goal1);
    map.agents.push_back(agent1);

    map.cells[0][1].isObstacle = true;
    map.cells[1][0].isObstacle = true;
    constraints.clear();  // Brak dodatkowych ograniczeń

    HighLevel highLevel(map, constraints);
    std::vector<std::vector<Cell>> solution = highLevel.solve();


    ASSERT_TRUE(solution.empty());
}


TEST_F(HighLevelTest, SolveWithConflicts) {

    std::cout<<"\n\n HighLevelTest::SolveWithConflicts \n\n";
    // Dodanie agentów i ich celów
    Position start1(0, 0);
    Position goal1(3, 3);
    Agent agent1(0, start1, goal1);
    map.agents.push_back(agent1);

    Position start2(0, 3);
    Position goal2(3, 0);
    Agent agent2(1, start2, goal2);
    map.agents.push_back(agent2);

    // Uruchomienie algorytmu
    HighLevel highLevel(map, constraints);
    std::vector<std::vector<Cell>> solution = highLevel.solve();
    
    // Oczekujemy dwóch ścieżek, po jednej dla każdego agenta
    ASSERT_EQ(solution.size(), 2);

    // Oczekiwane ścieżki, zakładając, że agenci muszą ominąć ograniczenia
    std::vector<Cell> expectedPath1 = {
        Cell(Position(0, 0)), Cell(Position(0, 1)), Cell(Position(0, 2)), Cell(Position(0, 3)), Cell(Position(1, 3)), Cell(Position(2, 3)), Cell(Position(3, 3))
    };

    std::vector<Cell> expectedPath2 = {
        Cell(Position(0, 3)), Cell(Position(1, 3)), Cell(Position(1, 2)), Cell(Position(1, 1)), Cell(Position(2,1)), Cell(Position(2, 0)), Cell(Position(3, 0))
    };


    std::cout<<"Solution Paths: \n";
    for(int i=0; i<solution.size(); i++)
    {
        std::cout<<"Agent " << i+1 << std::endl;
        for(int j=0; j<solution[i].size(); j++)
        {
            std::cout<<"(" << solution[i][j].position.x << ", " <<  solution[i][j].position.y << ")"  << ", t: " << j << std::endl;
        }
    }

    
    // Sprawdzanie ścieżki agenta 0
    if (!solution[0].empty()) {
        ASSERT_EQ(solution[0].size(), expectedPath1.size()) << "Agent 0 path size mismatch";
        for (size_t i = 0; i < expectedPath1.size(); ++i) {
            EXPECT_EQ(solution[0][i].position.x, expectedPath1[i].position.x) << "Mismatch at index " << i << " for Agent 0";
            EXPECT_EQ(solution[0][i].position.y, expectedPath1[i].position.y) << "Mismatch at index " << i << " for Agent 0";
        }
    } else {
        ADD_FAILURE() << "Agent 0 could not find a valid path";
    }

    // Sprawdzanie ścieżki agenta 1
    if (!solution[1].empty()) {
        ASSERT_EQ(solution[1].size(), expectedPath2.size()) << "Agent 1 path size mismatch";
        for (size_t i = 0; i < expectedPath2.size(); ++i) {
            EXPECT_EQ(solution[1][i].position.x, expectedPath2[i].position.x) << "Mismatch at index " << i << " for Agent 1";
            EXPECT_EQ(solution[1][i].position.y, expectedPath2[i].position.y) << "Mismatch at index " << i << " for Agent 1";
        }
    } else {
        ADD_FAILURE() << "Agent 1 could not find a valid path";
    }
}