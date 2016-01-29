#include "astar.h"
void astar::initializePlanner()
{
    open_list = Eigen::MatrixXd::Zero(0,5);
    closed_list = Eigen::MatrixXd::Zero(0,2);
    nodes_list = Eigen::MatrixXd::Zero(0,env.dimensions);
    goal_achieved = false;
    cout<<"Planner initialized"<<endl;
    setStart();
    setGoal();
    plan();
}
void astar::setStart()
{
   insertNode(env.start_state);
   double g_val = 0;
   double h_val;
   h_val = sqrt(((env.start_state - env.goal_state).transpose())*(env.start_state - env.goal_state));
   insertOpen(0,g_val,h_val,0);
   cout<<"Start state set"<<endl;
}
void astar::setGoal()
{
   nodes_list.conservativeResize(nodes_list.rows()+1,nodes_list.cols());
   nodes_list.row(nodes_list.rows()-1) = env.goal_state.transpose();
   cout<<"Goal state set"<<endl;
}
bool astar::isClosed(int index)
{
    bool closed = false;
    for(int i=0; i<closed_list.rows();i++)
    {
        if(index == closed_list(i,0))
        {
            closed = true;
            break;
        }
    }
    return(closed);
}
void astar::generateChildren(int index)
{
    Eigen::VectorXd parent_node = Eigen::VectorXd::Zero(env.dimensions);
    for(int i =0;i<parent_node.rows();i++)
    {
       parent_node(i) = nodes_list(open_list(index,0),i);
   }
    for(int i = 0; i<parent_node.rows(); i++)
    {
        for(int j = 0; j<2; j++)
        {
            Eigen::VectorXd child_node = parent_node;
            int child_index;
            child_node(i) = parent_node(i) + pow(-1,j)*env.primitives(i);
            if(isValid(child_node))
            {
            child_index = insertNode(child_node);
            if(!isClosed(child_index))
            {
                double g_val,h_val,score;
                g_val = open_list(index,1) + env.primitives(i);
                h_val = sqrt(((child_node - env.goal_state).transpose())*(child_node - env.goal_state));
                cout<<g_val+h_val<<endl;
                score = g_val + h_val;
                insertOpen(child_index,g_val,h_val,open_list(index,0));
            }
            }   
        }
    }

}
void astar::insertOpen(int node_index, double g_val, double h_val, int pred)
{
    open_list.conservativeResize(open_list.rows()+1,open_list.cols());
    Eigen::VectorXd new_entry = Eigen::VectorXd::Zero(5);
    new_entry(0) = node_index;
    new_entry(1) = g_val;
    new_entry(2) = h_val;
    new_entry(3) = pred;
    new_entry(4) = 0;
    open_list.row(open_list.rows()-1) = new_entry.transpose();
}
bool astar::goalReached(Eigen::VectorXd point)
{
    bool goalReached = false;
    if(((point-env.goal_state).transpose())*(point-env.goal_state)<1e-11)
    {
        goalReached = true;
    }
    return(goalReached);
}
void astar::plan()
{
  cout<<"Commencing plan"<<endl;
  while(goal_achieved == false)
  {
    double min_score = -1;
    int min_index = 0;
    for(int i = 0; i<open_list.rows();i++)
    {
        if(open_list(i,4)>=0)
        {
            double score = open_list(i,1) + open_list(i,2);
            if(score < min_score || min_score < 0)
            {
                min_score = score;
               // cout<<score<<endl;
                min_index = i;
            }
        }
    }
    Eigen::VectorXd test = Eigen::VectorXd::Zero(env.dimensions);
    for(int i = 0; i<env.dimensions; i++)
    {
        test(i) = nodes_list(open_list(min_index,0),i);
    }
    goal_achieved = goalReached(test);
    if(goal_achieved)
    {
    break;
    }
    insertClosed(min_index);
    generateChildren(min_index);
    open_list(min_index,4) = -1;
    //cout<<closed_list<<endl<<"****"<<endl;

  }


}
void astar::insertClosed(int index)
{
    closed_list.conservativeResize(closed_list.rows()+1,closed_list.cols());
    Eigen::Vector2d state;
    state(0) = open_list(index,0);
    state(1) = open_list(index,3);
    closed_list.row(closed_list.rows()-1) = state.transpose();
    
}

int astar::insertNode(Eigen::VectorXd point)
{
    int result = -1;
    for(int i = 0; i<nodes_list.rows();i++)
    {
        double residual = 0;
        for(int j = 0; j<nodes_list.cols(); j++)
        {
            residual = residual + pow((point(j)-nodes_list(i,j)),2);
        }
        residual = sqrt(residual);
        if(residual < 1e-10)
        {
            result = i;
            break;
        }
    }
    if(result == -1)
    {
        nodes_list.conservativeResize(nodes_list.rows()+1,nodes_list.cols());
        nodes_list.row(nodes_list.rows()-1) = point.transpose();
        result = nodes_list.rows()-1;
    }
    return(result);
}
bool astar::isValid(Eigen::VectorXd point)
{
bool valid = true;
if(point(0)>env.height||point(0)<0)
{valid = false;}
    
if(point(1)>env.width||point(1)<0)
{valid = false;}
   
if(env.gridMap(int(point(0)/env.resolution),int(point(1)/env.resolution)) == 1)
{valid = false;}
return(valid);  
}
