
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"

// LQR-RRT* includes
#include "orbital_planner/utils.hpp"
#include "orbital_planner/rrt_star.hpp"

State convertArrayToState(const double* array) {
    State state;
    for (int i = 0; i < state.rows(); ++i) {
        state(i) = array[i];
    }
    return state;
}

visualization_msgs::Marker visualize_tree_msg(const Tree* tree, std::string local_frame="map")
{
    visualization_msgs::Marker m;
    m.header.frame_id = local_frame;
    m.header.stamp = ros::Time();
    m.ns = "tree";
    m.id = 0;
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.color.a = 0.25;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0;

    for(const auto& node_info : tree->list)
    {
        std::shared_ptr<Graph_Node> node = node_info.second;
        
        //skip root node
        if(node->parent == nullptr)
        {
            continue;
        }

        geometry_msgs::Point child_point;
        child_point.x = node->config[0];
        child_point.y = node->config[1];
        child_point.z = node->config[2];
        geometry_msgs::Point parent_point;
        parent_point.x = node->parent->config[0];
        parent_point.y = node->parent->config[1];
        parent_point.z = node->parent->config[2];
        m.points.push_back(child_point);
        m.points.push_back(parent_point);
    }

    return m;
}

visualization_msgs::Marker visualize_path_msg(std::vector<State> path, std::string local_frame="map")
{
    visualization_msgs::Marker m;
    m.header.frame_id = local_frame;
    m.header.stamp = ros::Time();
    m.ns = "path";
    m.id = 0;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.b = 0.0;

    //iterate through path and add each node to path message
    for(auto node : path)
    {
        geometry_msgs::Point child_point;
        child_point.x = node[0];
        child_point.y = node[1];
        child_point.z = node[2];
        m.points.push_back(child_point);
    }

    return m;

}

class OrbitalPlannerNode
{
//everything public for now
public:
    // ROS specific things
    ros::Rate loop_rate;
    ros::NodeHandle &nh;
    std::string local_frame = "map"; //TODO: need to replace with right frame

    ros::Publisher tree_viz_pub;
    ros::Publisher path_viz_pub;

    OrbitalPlannerNode(ros::NodeHandle &nh, double rate)
    : nh(nh), loop_rate(rate)
    {
        tree_viz_pub = nh.advertise<visualization_msgs::Marker>("tree_visualization", 10);
        path_viz_pub = nh.advertise<visualization_msgs::Marker>("path_visualization", 10);
        ROS_INFO_STREAM("Orbital Planner Initialization Complete!");
    }

    //main planning loop here
    void run()
    {
        // load start and goal state
        double* start_d = doubleArrayFromString("8,2.4,7.3,-0.2,0.3,-0.2");
        double* goal_d = doubleArrayFromString("0,0,0,0,0,0");

        State start_state = convertArrayToState(start_d);
        State goal_state = convertArrayToState(goal_d);

        Print(start_state, "Start");
	    Print(goal_state, "Goal");

        const int n_dofs = start_state.rows();
        double** plan = NULL;
        int plan_length = 0;

        std::cout << std::endl << "******WELCOME TO LQR-RRT* MY FRIEND*****" << std::endl;
        
        RRTStar planner = RRTStar(start_state, goal_state, &plan, &plan_length);
        std::vector<State> state_path;

        std::cout << "---- Starting LQR-RRT* ----" << std::endl;

        auto start = std::chrono::high_resolution_clock::now();
        bool print_path_info = false;
        int i = 0;

        while(ros::ok())
        {
            //visualize tree
            const Tree* tree = planner.GetTree();
            visualization_msgs::Marker tree_marker = visualize_tree_msg(tree);
            tree_viz_pub.publish(tree_marker);

            //main planning loop
            if(!planner.PathFound() && i < RRT_STAR_NUM_ITER)
            {
                std::cout << "Iteration " << i << std::endl;
                planner.Iterate();
                i++;
            }
            else if(planner.PathFound())
            {
                double cost = planner.ComputePath(state_path);

                //final path visualization
                visualization_msgs::Marker path_marker = visualize_path_msg(state_path);
                path_viz_pub.publish(path_marker);

                if(!print_path_info)
                {
                    for (auto state : state_path)
                    {
                        Print(state, "here we go");
                    }

                    std::cout << "Finished planning with " << i + 1 << " samples!" << std::endl;
                    std::cout << "Quadratic State Cost " << cost << std::endl;
                    std::cout << "PATH SIZE: " << plan_length << std::endl;
                    std::cout << "RESULT -> SOLUTION FOUND!" << std::endl;
                    auto end = std::chrono::high_resolution_clock::now();
                    auto time_elapsed_milli = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                    std::cout << "Graph Nodes: " << tree->list.size() << std::endl;
                    std::cout << "TIME: " << (double) time_elapsed_milli/1000 << std::endl;
                    print_path_info = true;
                }
            }
            else
            {
                ROS_ERROR_STREAM("RESULT -> PATH NOT FOUND WITH RRT-STAR");
            }

            //loop rate here
            ros::spinOnce();
            loop_rate.sleep();

        }
    }
};
