
// ROS includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/console.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "orbital_planner_node");
    ros::NodeHandle nh;

    ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
    
    //test node to begin with
    int count = 0;
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        chatter_pub.publish(msg);
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }
}