
// ROS includes
#include "ros/console.h"
#include "orbital_visualization/orbital_visualization_node.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "orbital_visualization_node");
    ros::NodeHandle nh;
    double loop_rate = 10;

    OrbitalVisualizationNode planner_node(nh, loop_rate);
    planner_node.run();
}