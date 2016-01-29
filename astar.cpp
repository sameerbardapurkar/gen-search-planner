#include "astar.h"
void astar::initializePlanner()
{
    //Initialize planner specific variables here
    //such as the open list, closed list
    //and the hash tables generated while expanding the state-space nodes
}
void astar::setStart()
{
    //Function to set the start configuration of robot
}
void astar::setGoal()
{
    //Function to set goal configuration of robot
}
bool astar::isClosed(int index)
{
    //Function to check if a node is in the closed list

}

void astar::generateChildren(int index)
{
    //Function to generate successors of a node
    //Ideally, I would like to create a class for a node
    //And store the predecessor, g-value, h-value, state-space co-ordinates 
    //etc. as node objects. The function generateChildren would then generate a
    //set of successors of a node and populate a queue/stack data structure 
    //like for example a std::vector
}
void astar::insertOpen(int node_index, double g_val, double h_val, int pred)
{
    //Function to insert a (fringe/frontier) node into the open list
}
bool astar::goalReached(Eigen::VectorXd point)
{
    //Function to check if we have reached the goal
}
void astar::plan()
{
    //The main function in the planner, this function repeatedly calls the 
    //generateChildren function and keeps searching for the least path-cost node
    //It then checks if the least path-cost node co-incides with the goal node 
    //if the goal is reached, the planner terminates sucessfully.
    //This function also keeps track of the open list. If the open list becomes empty before
    //the goal is reached, this function terminates with failure.
}
void astar::insertClosed(int index)
{
   //Function to insert nodes in the closed list 
}

int astar::insertNode(Eigen::VectorXd point)
{
    //This is the main function to generate the search graph on-the-fly
    //It can either be a simple table (for small graphs) or it can incorporate 
    //a hashing scheme (for larger graphs). This function works by taking in the co-ordinates of
    //a point and then adding them in a table which represents the search graph.
}
bool astar::isValid(Eigen::VectorXd point)
{
    //This function checks if a given state is valid (by comparing things like collisions/out-of bounds etc.)
}
