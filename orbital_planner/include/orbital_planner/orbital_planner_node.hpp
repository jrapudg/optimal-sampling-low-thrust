
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

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

tf2::Quaternion convert_vel_to_quat(double vel_x, double vel_y, double vel_z)
{
    double vec_mag = std::sqrt(std::pow(vel_x,2) + std::pow(vel_y,2) + std::pow(vel_z,2));
    tf2::Quaternion orientation;

    if(vec_mag == 0.0)
    {
        
        orientation.setRPY(0.0, 0.0, 0.0);
    }
    else
    {
        geometry_msgs::Vector3 velocity_vector;
        velocity_vector.x = vel_x / vec_mag;
        velocity_vector.y = vel_y / vec_mag;
        velocity_vector.z = vel_z / vec_mag;

        geometry_msgs::Vector3 reference_vector;
        reference_vector.x = 0.0;
        reference_vector.y = 0.0;
        reference_vector.z = 1.0;

        tf2::Vector3 rotation_axis(velocity_vector.y, -velocity_vector.x, 0);
        double rotation_angle = std::acos(velocity_vector.z);
        
        orientation.setRotation(rotation_axis, rotation_angle);
    }
    return orientation;
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

visualization_msgs::MarkerArray visualize_path_points_msg(std::vector<State> path, std::string local_frame="map")
{
    visualization_msgs::MarkerArray ma;
    int marker_idx = 0;
    for(auto node : path)
    {
        visualization_msgs::Marker m;
        m.header.frame_id = local_frame;
        m.header.stamp = ros::Time();
        m.ns = "path_points";
        m.id = marker_idx;
        m.type = visualization_msgs::Marker::ARROW;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = node[0];
        m.pose.position.y = node[1];
        m.pose.position.z = node[2];
        m.scale.x = 0.5;
        m.scale.y = 0.1;
        m.scale.z = 0.1;
        m.color.a = 0.75;
        m.color.g = 1.0;

        tf2::Quaternion orientation = convert_vel_to_quat(node[3], node[4], node[5]);
        m.pose.orientation.x = orientation.x();
        m.pose.orientation.y = orientation.y();
        m.pose.orientation.z = orientation.z();
        m.pose.orientation.w = orientation.w();

        ma.markers.push_back(m);
        marker_idx += 1;
    }
    return ma;
}

visualization_msgs::Marker visualize_agent_msg(State state, std::string local_frame="map")
{
    visualization_msgs::Marker m;
    m.header.frame_id = local_frame;
    m.header.stamp = ros::Time();
    m.ns = "agent";
    m.id = 0;
    m.type = visualization_msgs::Marker::MESH_RESOURCE;
    m.action = visualization_msgs::Marker::ADD;
    m.mesh_resource = "package://orbital_planner/config/dragon_centered.stl";
    
    tf2::Quaternion orientation = convert_vel_to_quat(state[3], state[4], state[5]);
    m.pose.position.x = state[0];
    m.pose.position.y = state[1];
    m.pose.position.z = state[2];
    m.pose.orientation.x = orientation.x();
    m.pose.orientation.y = orientation.y();
    m.pose.orientation.z = orientation.z();
    m.pose.orientation.w = orientation.w();
    m.scale.x = 0.001;
    m.scale.y = 0.001;
    m.scale.z = 0.001;
    // Set the color (RGBA values)
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.b = 1.0;
    m.color.a = 1.0;
    return m;
}

visualization_msgs::Marker visualize_sampled_states_msg(std::vector<State> sampled_states, std::string local_frame="map")
{
    visualization_msgs::Marker m;
    m.header.frame_id = local_frame;
    m.header.stamp = ros::Time();
    m.ns = "sampled_points";
    m.id = 0;
    m.type = visualization_msgs::Marker::SPHERE_LIST;
    m.action = visualization_msgs::Marker::ADD;

    for(State state : sampled_states)
    {
        geometry_msgs::Point point;
        point.x = state[0];
        point.y = state[1];
        point.z = state[2];
        m.points.push_back(point);
    }

    m.scale.x = 0.15;
    m.scale.y = 0.15;
    m.scale.z = 0.15;
    m.color.a = 1.0;
    m.color.r = 0.0;
    m.color.g = 0.0;
    m.color.b = 1.0;

    return m;
}

visualization_msgs::Marker visualize_earth_msg(std::vector<double> pose, std::string local_frame="map")
{
    visualization_msgs::Marker m;
    m.header.frame_id = local_frame;
    m.header.stamp = ros::Time();
    m.ns = "earth";
    m.id = 0;
    m.type = visualization_msgs::Marker::MESH_RESOURCE;
    m.action = visualization_msgs::Marker::ADD;
    m.mesh_resource = "package://orbital_planner/config/earth.dae";
    m.mesh_use_embedded_materials = true;
    m.pose.position.x = pose[0];
    m.pose.position.y = pose[1];
    m.pose.position.z = pose[2];

    tf::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, -M_PI/2.0);

    m.pose.orientation.x = orientation.x();
    m.pose.orientation.y = orientation.y();
    m.pose.orientation.z = orientation.z();
    m.pose.orientation.w = orientation.w();
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.color.a = 1.0;
    return m;
}

visualization_msgs::MarkerArray visualize_asteriods(std::vector<std::vector<double>> poses, std::string local_frame="map")
{
    visualization_msgs::MarkerArray ma;
    int marker_idx = 0;
    for(auto pose : poses)
    {
        visualization_msgs::Marker m;
        m.header.frame_id = local_frame;
        m.header.stamp = ros::Time();
        m.ns = "asteriod";
        m.id = marker_idx;
        m.type = visualization_msgs::Marker::MESH_RESOURCE;
        m.action = visualization_msgs::Marker::ADD;
        m.mesh_resource = "package://orbital_planner/config/asteroid.stl";
        m.pose.position.x = pose[0];
        m.pose.position.y = pose[1];
        m.pose.position.z = pose[2];
        m.pose.orientation.x = 0.0;
        m.pose.orientation.y = 0.0;
        m.pose.orientation.z = 0.0;
        m.pose.orientation.w = 1.0;
        // m.mesh_use_embedded_materials = true;
        m.scale.x = 7.0;
        m.scale.y = 7.0;
        m.scale.z = 7.0;
        m.color.a = 1.0;
        m.color.r = 0.6980;
        m.color.g = 0.7450;
        m.color.b = 0.7098;

        ma.markers.push_back(m);
        marker_idx += 1;
    }

    return ma;
}

visualization_msgs::Marker visualize_goal_pose(State state, std::string local_frame="map")
{
    visualization_msgs::Marker m;
    m.header.frame_id = local_frame;
    m.header.stamp = ros::Time();
    m.ns = "agent";
    m.id = 0;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.mesh_resource = "package://orbital_planner/config/dragon_centered.stl";
    
    tf2::Quaternion orientation = convert_vel_to_quat(state[3], state[4], state[5]);
    m.pose.position.x = state[0];
    m.pose.position.y = state[1];
    m.pose.position.z = state[2];
    m.pose.orientation.x = orientation.x();
    m.pose.orientation.y = orientation.y();
    m.pose.orientation.z = orientation.z();
    m.pose.orientation.w = orientation.w();
    m.scale.x = 1.0;
    m.scale.y = 0.2;
    m.scale.z = 0.2;
    // Set the color (RGBA values)
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0;
    m.color.a = 1.0;
    return m;
}

class OrbitalPlannerNode
{
//everything public for now
public:
    bool begin_planning = false;

    // ROS specific things
    ros::Rate planning_rate;
    ros::Rate visualization_rate;
    ros::NodeHandle &nh;
    std::string local_frame = "map"; //TODO: need to replace with right frame

    ros::Publisher tree_viz_pub;
    ros::Publisher path_viz_pub;
    ros::Publisher agent_viz_pub;
    ros::Publisher sampled_state_pub;
    ros::Publisher path_points_viz_pub;
    ros::Publisher earth_viz_pub;
    ros::Publisher asteriod_viz_pub;
    ros::Publisher goal_viz_pub;
    ros::Subscriber start_planner_sub;

    OrbitalPlannerNode(ros::NodeHandle &nh, double rate1, double rate2)
    : nh(nh), planning_rate(rate1), visualization_rate(rate2)
    {
        start_planner_sub = nh.subscribe<std_msgs::String>("start_planning", 10, &OrbitalPlannerNode::start_planner_callback, this);
        tree_viz_pub = nh.advertise<visualization_msgs::Marker>("tree_visualization", 10);
        path_viz_pub = nh.advertise<visualization_msgs::Marker>("path_visualization", 10);
        agent_viz_pub = nh.advertise<visualization_msgs::Marker>("agent_visualization", 10);
        earth_viz_pub = nh.advertise<visualization_msgs::Marker>("earth_visualization", 10);
        goal_viz_pub = nh.advertise<visualization_msgs::Marker>("goal_visualization", 10);
        sampled_state_pub = nh.advertise<visualization_msgs::Marker>("sampled_state_visualization", 10);
        path_points_viz_pub = nh.advertise<visualization_msgs::MarkerArray>("path_points_visualization", 10);
        asteriod_viz_pub = nh.advertise<visualization_msgs::MarkerArray>("asteriod_visualization", 10);

        ROS_INFO_STREAM("Orbital Planner Initialization Complete!");
    }

    void start_planner_callback(const std_msgs::String::ConstPtr& msg) 
    {
        // Your callback logic here
        ROS_INFO("Starting planning");

        // Set the boolean variable to true when a message is received
        begin_planning = true;
    }

    //main planning loop here
    void run()
    {
        // load start and goal state
        double* start_d = doubleArrayFromString("-12.306,24.39,34.307,0.01,0.02,-0.015");
        double* goal_d = doubleArrayFromString("0,0,0,0,0,0");

        State start_state = convertArrayToState(start_d);
        State goal_state = convertArrayToState(goal_d);

        int waypoint_num = 0;
        State curr_state = start_state;

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
            //visualize goal pose
            visualization_msgs::Marker goal_pose_marker = visualize_goal_pose(goal_state);
            goal_viz_pub.publish(goal_pose_marker);

            //visualize tree
            const Tree* tree = planner.GetTree();
            visualization_msgs::Marker tree_marker = visualize_tree_msg(tree);
            tree_viz_pub.publish(tree_marker);

            //visualize agent pose
            visualization_msgs::Marker agent_marker = visualize_agent_msg(curr_state);
            agent_viz_pub.publish(agent_marker);

            //visualize earth model
            earth_viz_pub.publish(visualize_earth_msg({50.0, -10.0, 5.0}));

            //visualize asteriod model
            std::vector<std::vector<double>> asteriod_poses;
            asteriod_poses.push_back({-5, 10, 20});
            asteriod_viz_pub.publish(visualize_asteriods(asteriod_poses));

            //visualize sampled states
            std::vector<State> sampled_states = planner.GetSampledStates();
            sampled_state_pub.publish(visualize_sampled_states_msg(sampled_states));

            if(begin_planning)
            {
                //main planning loop
                if(!planner.PathFound() && i < RRT_STAR_NUM_ITER)
                {
                    std::cout << "Iteration " << i << std::endl;
                    planner.Iterate();
                    i++;
                    planning_rate.sleep();

                }
                else if(planner.PathFound())
                {
                    double cost = planner.ComputePath(state_path);

                    //final path visualization
                    visualization_msgs::Marker path_marker = visualize_path_msg(state_path);
                    path_viz_pub.publish(path_marker);

                    visualization_msgs::MarkerArray path_points_marker = visualize_path_points_msg(state_path);
                    path_points_viz_pub.publish(path_points_marker);

                    //update current waypoint
                    if(waypoint_num < state_path.size())
                    {
                        curr_state = state_path[waypoint_num];
                        waypoint_num += 1;
                    }

                    if(!print_path_info)
                    {
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
                    visualization_rate.sleep();
                }
                else
                {
                    ROS_ERROR_STREAM("RESULT -> PATH NOT FOUND WITH RRT-STAR");
                }
            }

            //loop rate here
            ros::spinOnce();
            // visualization_loop_rate.sleep();

        }
    }
};
