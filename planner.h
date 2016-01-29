#include "includes.h"
#include "environment.h"
class planner{
    public:
        environment env;
        virtual void initializePlanner()
        {
        }
        virtual void setStart()
        {
        }
        virtual void setGoal()
        {
        }
        virtual void expandNode()
        {
        }
        virtual int isMember(Eigen::VectorXd point, Eigen::MatrixXd set)
        {
        }
        virtual bool isValid(Eigen::VectorXd point)
        {
        }
    protected:
        Eigen::MatrixXd open_list;
        Eigen::MatrixXd closed_list;
        Eigen::MatrixXd nodes_list;
};

