#include "HighLevel.h"

HighLevel::HighLevel(Map &map, const std::vector<Constraint>& constraints)
{
    this->map = map;
    CT = std::make_unique<ConstraintTree>(map);

    root->constraints = constraints; //ale one powinny byc chyba zerowe na początku
    root->solution = ConstraintTree::findInitialSolution(constraints);  // Find initial paths
    root->updateCost();  // Calculate cost of the initial solution
    openList.push(root);
}

std::vector<std::vector<Cell>> HighLevel::solve()
{

    while(!openList.empty())
    {
        auto P = openList.top();  // Get the node with the lowest cost
        openList.pop();

        if(!ConstraintTree::detectConflicts(*P) && !ConstraintTree::hasEdgeConflict(*P))
        {
            return P->solution;
        }

        Conflict C = ConstraintTree::getFirstConflict(*P);
        if(ConstraintTree::detectConflicts(*P))
        {
            std::cout<<"Normal conflict (" << C.pos1.position.x << ", " << C.pos1.position.y << ")\n";
            for (int i = 0; i < 2; ++i) 
            {  
                auto child = P->addConstraint(Constraint(C.agent1, C.pos1.position, C.time));
                if(i==1)
                {
                    auto child = P->addConstraint(Constraint(C.agent1, C.pos1.position, C.time));
                }
                child->updateSolution(map, P->solution, P->cost);
                child->updateCost();

                // 25. Insert the node into OPEN if a valid solution was found.
                if (child->cost < DBL_MAX) {
                    openList.push(child);
                }
            }
        }
        else if(ConstraintTree::hasEdgeConflict(*P))
        {
            std::cout<<"Edge conflict (" << C.pos1.position.x << ", " << C.pos1.position.y << ")\n";
            for (int i = 0; i < 2; ++i) 
            {
                
                auto child = P->addConstraint(Constraint(C.agent1, C.pos1.position, C.time-1));
                auto child1 = P->addConstraint(Constraint(C.agent1, C.pos2.position, C.time));

                if(i==1)
                {
                    child = P->addConstraint(Constraint(C.agent2, C.pos2.position, C.time-1));
                    child1 = P->addConstraint(Constraint(C.agent2, C.pos1.position, C.time));
                }
//child
                child->updateSolution(map, P->solution, P->cost);  // Update solution
                child->updateCost();  // Update cost after generating the solution

                if (child->cost < DBL_MAX) {
                    openList.push(child);
                }

//child 1
                child1->updateSolution(map, P->solution, P->cost);  // Update solution
                child1->updateCost();  // Update cost after generating the solution
                if (child1->cost < DBL_MAX) {
                    openList.push(child1);
                }
            }
        }
    }
    return std::vector<std::vector<Cell>>();
}
