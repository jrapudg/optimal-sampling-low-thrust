
// ROS includes
#include "ros/console.h"
#include "orbital_planner/orbital_planner_node.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "orbital_planner_node");
    ros::NodeHandle nh;
    double loop_rate = 10;

    OrbitalPlannerNode planner_node(nh, loop_rate);
    planner_node.run();
}