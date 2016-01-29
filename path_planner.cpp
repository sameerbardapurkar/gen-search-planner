#include "astar.h"
using namespace std;
int main(){
    cout<<"Started"<<endl;
    environment env("env_config.txt","prims.txt", "locations.txt");
    astar planner1;
    planner1.env = env;
    planner1.initializePlanner();
    return(0);
}

