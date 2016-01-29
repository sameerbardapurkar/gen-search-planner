#include "planner.h"
class astar: public planner{
    public:
    void initializePlanner(); 
    void setStart();
    void setGoal();
    void plan();
    void insertClosed(int index);
    bool goalReached(Eigen::VectorXd point);
    bool isClosed(int index);
    int insertNode(Eigen::VectorXd point);
    void insertOpen(int,double,double,int);
    void generateChildren(int index);
    bool isValid (Eigen::VectorXd point);
    private:
    bool goal_achieved;
};
