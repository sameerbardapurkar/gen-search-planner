#include "includes.h"
//This is the main file for constructing an environment for the planner. This takes as input various variables like motion primitives, map, map resolution etc. and creates an environment object.
using namespace std;
class environment
{
    public:
        int dimensions;
        int resolution;
        int height;
        int width;
        Eigen::MatrixXd gridMap;
        Eigen::VectorXd start_state;
        Eigen::VectorXd goal_state;
        Eigen::VectorXd primitives;
        environment(string config_file, string primitives_file,string start_goal){
            cout<<"Initializing environment from file, please wait."<<endl;
            ifstream initializer(config_file.c_str());
            if(initializer.is_open())
            {   
                string data;
                int line_count = 0;
                while(getline(initializer,data))
                {
                    if(line_count<8)
                    {
                        if(line_count%2 == 0)
                        {
                            cout<<"Loading "<<data<<": ";
                        }
                        else
                        {
                            cout<<data<<endl;
                            data_holder((line_count-1)/2) = atof(data.c_str());
                        }
                    }
                    else if(line_count == 8)
                    {
                        cout<<"Now reading "<<data<<endl;
                    }
                    else
                    {
                        obstaclemapstr = obstaclemapstr + data;
                    }
                    line_count++;
                }
            initializer.close();
            }
            else cout<<"Failed to open file \n";
            dimensions = data_holder(0);
            resolution = data_holder(1);
            height = data_holder(2);
            width = data_holder(3);
            gridMap = Eigen::MatrixXd::Zero(int(height/resolution),int(width/resolution));
            stringstream obsmap(obstaclemapstr);
            temp_counter1 = 0;
            temp_counter2 = 0;
            primitives = Eigen::VectorXd::Zero(dimensions);
            while(obsmap.good())
            {
                string occ;
                getline(obsmap,occ,',');
                gridMap(temp_counter1,temp_counter2) = atof(occ.c_str());
                temp_counter2++;
                if(temp_counter2 == width)
                {
                    temp_counter1++;
                    temp_counter2 = 0;
                }
            }
            cout<<gridMap<<endl<<"Now loading motion primitives"<<endl;
            initializer.open(primitives_file.c_str());
            temp_counter1 = 0;
            if(initializer.is_open())
            {
                string data;
                while(getline(initializer,data))
                {
                   primitives(temp_counter1) = atof(data.c_str());
                   temp_counter1++;                                
                }

            }
            else cout<<"Failed to open file \n";
            cout<<"Primitives are \n"<<primitives<<endl;
            initializer.close();
            start_state = Eigen::VectorXd::Zero(dimensions);
            goal_state = Eigen::VectorXd::Zero(dimensions);
            cout<<"Now reading the start and goal locations"<<endl;
            initializer.open(start_goal.c_str());
            temp_counter1 = 0;
            string start;
            if(initializer.is_open())
            {
                string data;
                while(getline(initializer,data))
                {
                        temp_counter2 = 0;
                        stringstream pointcoord(data);
                        while(pointcoord.good())
                        {
                            string point;
                            getline(pointcoord,point,',');
                            if(temp_counter1 == 0)
                                 start_state(temp_counter2) = atof(point.c_str());
                            else if(temp_counter1 == 1)
                                 goal_state(temp_counter2) = atof(point.c_str());

                            temp_counter2++;
                        }
                    temp_counter1++;
                }
            cout<<"Start location is: "<<start_state<<endl;
            cout<<"Goal location is: "<<goal_state<<endl;
            }
            else cout<<"Failed to open file \n";
        }
        environment()
        {
        }
        ~environment(){
        }
    private:
        int temp_counter1, temp_counter2;
        Eigen::Vector4f data_holder;
        string obstaclemapstr;

};


