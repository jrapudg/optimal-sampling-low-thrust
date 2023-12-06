
// ROS includes
#include "ros/console.h"
#include "orbital_planner/orbital_planner_node.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "orbital_planner_node");
    ros::NodeHandle nh;
    double planning_rate = 30;
    double visualization_rate = 5;

    OrbitalPlannerNode planner_node(nh, planning_rate, visualization_rate);
    planner_node.run();
}