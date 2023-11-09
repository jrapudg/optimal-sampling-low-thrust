
#include "ros/ros.h"
#include "std_msgs/String.h"

class OrbitalPlannerNode
{
//everything public for now
public:

    // ROS specific things
    ros::Rate loop_rate;
    ros::NodeHandle &nh;
    std::string local_frame = "map"; //TODO: need to replace with right frame

    OrbitalPlannerNode(ros::NodeHandle &nh, double rate)
    : nh(nh), loop_rate(rate)
    {
        ROS_INFO_STREAM("Orbital Planner Initialization Complete!");
    }

    //main planning loop here
    void run()
    {
        ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

        int count = 0;
        while (ros::ok())
        {
            //test code for now
            std_msgs::String msg;

            std::stringstream ss;
            ss << "hello world " << count;
            msg.data = ss.str();

            ROS_INFO("%s", msg.data.c_str());

            chatter_pub.publish(msg);

            ros::spinOnce();
            this->loop_rate.sleep();
            ++count;
        }
    }

};
