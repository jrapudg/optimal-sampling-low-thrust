
#include <random>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>


#define EPSILON 50
#define MAX_ITERS 10000
#define GOAL_PROB_THRESH 0.10
#define NUM_INTERP_POINTS 50
#define GOAL_DIST_THRESH 10

//status message values for extending
#define REACHED 0
#define ADVANCED 1
#define TRAPPED 2

//goal prob distribution variables
std::random_device goal_rd;
std::mt19937 goal_rng(2);  
std::uniform_real_distribution<double> goal_prob_dist(0.0, 1.0); //mean of 0 & std_dev of 1.0

//xy distribution variables
std::random_device xy_rd;
std::mt19937 xy_rng(2);
std::uniform_real_distribution<double> rand_xy_dist(0.0, 1000.0); //sample from 0 --> 1000

//z distribution variables
std::random_device z_rd;
std::mt19937 z_rng(2);
std::uniform_real_distribution<double> rand_z_dist(0.0, 20.0); //sample from 0 --> 1000

struct Node
{
	//base member variables for all planners
    double cost;
    int depth;
    double x;
    double y;
    double z;
    std::shared_ptr<Node> parent_ptr;

    Node() {}

    Node(double _x, double _y, double _z, double _cost, double _depth, std::shared_ptr<Node> _parent_ptr)
    {
        x = _x;
        y = _y;
        z = _z;
        cost = _cost;
        depth = _depth;
        parent_ptr = _parent_ptr;
    }
};

class OrbitalVisualizationNode
{
//everything public for now
public:
    //global variables
    bool begin_planning = false;

    // ROS specific things
    ros::Rate loop_rate;
    ros::NodeHandle &nh;
    std::string local_frame = "map"; //TODO: need to replace with right frame

    OrbitalVisualizationNode(ros::NodeHandle &nh, double rate)
    : nh(nh), loop_rate(rate)
    {
        ROS_INFO_STREAM("Orbital Visualization Initialization Complete!");
    }

    std::shared_ptr<Node> get_random_config(std::shared_ptr<Node> goal_node)
    {   
        //apply a random weight for sampling the goal config
        double goal_prob = goal_prob_dist(goal_rng);
        if(goal_prob < GOAL_PROB_THRESH) //around 60% probability
        {
            return goal_node;
        }
        //generate random x y and z values
        double rand_x = rand_xy_dist(xy_rng);
        double rand_y = rand_xy_dist(xy_rng);
        double rand_z = rand_z_dist(z_rng);

        //create a node pointer for the randomized spacecraft pose
        return std::make_shared<Node>(rand_x, rand_y, rand_z, 0.0, 0.0, nullptr);
    }

    //simple euclid distance
    double get_distance(double x1, double y1, double z1, double x2, double y2, double z2)
    {
        return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2) + std::pow(z1 - z2, 2));
    }

    std::shared_ptr<Node> nearest_neighbor(std::vector<std::shared_ptr<Node>> &nodes_list, std::shared_ptr<Node> q)
    {
        //iterate through node list and determine which node is closest
        std::shared_ptr<Node> nearest_node;
        double min_dist = std::numeric_limits<double>::max();
        for(std::shared_ptr<Node> node : nodes_list)
        {
            //get the distance between the current node and sampled node
            double node_dist = get_distance(node->x, node->y, node->z, q->x, q->y, q->z);
            //check to see if it's less than current dist
            if(node_dist < min_dist)
            {
                min_dist = node_dist;
                nearest_node = node;
            }
        }
        return nearest_node;
    }

    //move at most by epsilon towards the sampled point
    bool get_new_config(std::shared_ptr<Node> q, std::shared_ptr<Node> q_near, std::shared_ptr<Node> &q_new)
    {
        //perform linear interpolation towards the sampled point
        double distance = get_distance(q_near->x, q_near->y, q_near->z, q->x, q->y, q->z);
        int numofsamples = (int)(distance/(1.0/NUM_INTERP_POINTS));
        bool success = false;
        for (int i = 1; i < numofsamples; ++i) 
        {
            double x_interp = q_near->x + (static_cast<double>(i) / (numofsamples - 1)) * (q->x - q_near->x);
            double y_interp = q_near->y + (static_cast<double>(i) / (numofsamples - 1)) * (q->y - q_near->y);
            double z_interp = q_near->z + (static_cast<double>(i) / (numofsamples - 1)) * (q->z - q_near->z);

            // Check if it's a valid configuration and within epsilon distance to the neighbor node
            double interp_dist = get_distance(x_interp, y_interp, z_interp, q_near->x, q_near->y, q_near->z);
            if(interp_dist < EPSILON)
            {
                q_new->x = x_interp;
                q_new->y = y_interp;
                q_new->z = z_interp;
                q_new->depth = q_near->depth + 1;
                q_new->cost = q_near->cost + get_distance(x_interp, y_interp, z_interp, q_near->x, q_near->y, q_near->z);
                q_new->parent_ptr = q_near;
                success = true;
            }
        }
        return success;
    }

    bool equalNodes(std::shared_ptr<Node> a, std::shared_ptr<Node> b)
    {
        //check abs of x
        if(abs(a->x - b->x) > 1e-3 || abs(a->y - b->y) > 1e-3 || abs(a->z - b->z) > 1e-3)
        {
            return false;
        }
        return true;
    }

    int extend(std::vector<std::shared_ptr<Node>> &nodes_list, 
    std::shared_ptr<Node> q, std::shared_ptr<Node> q_extend)
    {
        //get nearest neighbor
        std::shared_ptr<Node> q_near = nearest_neighbor(nodes_list, q);

        //get new configuration that moves at most by epsilon towards sampled point
        std::shared_ptr<Node> q_new = std::make_shared<Node>();
        if(get_new_config(q, q_near, q_new))
        {
            //push node into nodes list
            nodes_list.push_back(q_new);

            //set the extended node as q_new
            q_extend->x = q_new->x;
            q_extend->y = q_new->y;
            q_extend->z = q_new->z;
            q_extend->cost = q_new->cost;
            q_extend->depth = q_near->depth + 1;
            q_extend->parent_ptr = q_near;

            //check to see if we're at the sampled node
            if(equalNodes(q_new, q))
            {
                return REACHED;
            }
            return ADVANCED;
        }
        return TRAPPED;
    }

    visualization_msgs::Marker visualize_tree_msg(const std::vector<std::shared_ptr<Node>> nodes_list,
                                            std::string local_frame)
    {
        visualization_msgs::Marker m;
        m.header.frame_id = local_frame;
        m.header.stamp = ros::Time();
        m.ns = "tree_viz";
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
        m.scale.x = 1.0;
        m.scale.y = 1.0;
        m.scale.z = 1.0;
        m.color.a = 0.5;
        m.color.r = 0.0;
        m.color.g = 1.0;
        m.color.b = 0.0;

        //iterate through tree and add nodes to marker
        for (auto node : nodes_list)
        {
            if (node->parent_ptr == nullptr)
            {
                continue; //skip root node
            }

            geometry_msgs::Point child_point;
            child_point.x = node->x;
            child_point.y = node->y;
            child_point.z = node->z;
            geometry_msgs::Point parent_point;
            parent_point.x = node->parent_ptr->x;
            parent_point.y = node->parent_ptr->y;
            parent_point.z = node->parent_ptr->z;

            m.points.push_back(child_point);
            m.points.push_back(parent_point);
        }
        return m;
    }

    visualization_msgs::Marker visualize_start_goal(const std::shared_ptr<Node> q_start,
                                            const std::shared_ptr<Node> q, std::string local_frame)
    {
        visualization_msgs::Marker m;
        m.header.frame_id = local_frame;
        m.header.stamp = ros::Time();
        m.ns = "start_goal_viz";
        m.id = 0;
        m.type = visualization_msgs::Marker::SPHERE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = 0.0;
        m.pose.position.y = 0.0;
        m.pose.position.z = 0.0;
        m.pose.orientation.x = 0.0;
        m.pose.orientation.y = 0.0;
        m.pose.orientation.z = 0.0;
        m.pose.orientation.w = 1.0;
        m.scale.x = 1.0;
        m.scale.y = 1.0;
        m.scale.z = 1.0;
        m.color.a = 1.0;
        m.color.r = 0.0;
        m.color.g = 0.0;
        m.color.b = 1.0;

        geometry_msgs::Point start_point;
        start_point.x = q_start->x;
        start_point.y = q_start->y;
        start_point.z = q_start->z;
        geometry_msgs::Point goal_point;
        goal_point.x = q->x;
        goal_point.y = q->y;
        goal_point.z = q->z;

        m.points.push_back(start_point);
        m.points.push_back(goal_point);

        return m;
    }

    visualization_msgs::Marker visualize_path(std::vector<std::shared_ptr<Node>> path,
                                        std::string local_frame)
    {
        visualization_msgs::Marker m;
        m.header.frame_id = local_frame;
        m.header.stamp = ros::Time();
        m.ns = "path_viz";
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
        m.scale.x = 1.0;
        m.scale.y = 1.0;
        m.scale.z = 1.0;
        m.color.a = 1.0;
        m.color.r = 1.0;
        m.color.g = 1.0;
        m.color.b = 0.0;

        //backtrack the path and append each segment to nodes list
        for(auto node_ptr : path)
        {
            geometry_msgs::Point child_point;
            child_point.x = node_ptr->x;
            child_point.y = node_ptr->y;
            child_point.z = node_ptr->z;
            geometry_msgs::Point parent_point;
            parent_point.x = node_ptr->parent_ptr->x;
            parent_point.y = node_ptr->parent_ptr->y;
            parent_point.z = node_ptr->parent_ptr->z;

            m.points.push_back(child_point);
            m.points.push_back(parent_point);
        }

        return m;
    }

    void start_planning_callback(const std_msgs::String::ConstPtr& msg) {
        // Your callback logic here
        ROS_INFO("Starting planning");

        // Set the boolean variable to true when a message is received
        begin_planning = true;
    }

    visualization_msgs::Marker visualize_agent_mesh(double agent_x, double agent_y, double agent_z,
                                                std::string local_frame)
    {
        visualization_msgs::Marker m;
        m.header.frame_id = local_frame;
        m.header.stamp = ros::Time();
        m.ns = "agent_mesh";
        m.id = 0;
        m.type = visualization_msgs::Marker::MESH_RESOURCE;
        m.action = visualization_msgs::Marker::ADD;
        m.mesh_resource = "package://orbital_visualization/config/Simple_Spacecraft.dae";
        m.pose.position.x = agent_x;
        m.pose.position.y = agent_y;
        m.pose.position.z = agent_z;
        m.pose.orientation.x = 0.0;
        m.pose.orientation.y = 0.0;
        m.pose.orientation.z = 0.0;
        m.pose.orientation.w = 1.0;
        m.scale.x = 5.0;
        m.scale.y = 5.0;
        m.scale.z = 5.0;
        m.color.a = 1.0;

        return m;
    }

    geometry_msgs::TransformStamped move_spacecraft(std::vector<double> desired_pose)
    {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "spacecraft";
        transformStamped.transform.translation.x = desired_pose[0];
        transformStamped.transform.translation.y = desired_pose[1];
        transformStamped.transform.translation.z = desired_pose[2];
        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = 0.0;
        transformStamped.transform.rotation.w = 1.0;
        return transformStamped;
    }
    
    std::vector<std::shared_ptr<Node>> backtrack_path(std::shared_ptr<Node> q_reach_goal)
    {
        //backtrack the path and append each segment to nodes list
        std::vector<std::shared_ptr<Node>> path;
        std::shared_ptr<Node> node_ptr = q_reach_goal;
        while(node_ptr->parent_ptr)
        {   
            path.push_back(node_ptr);
            node_ptr = node_ptr->parent_ptr;
        }
        return path;
    }

    std::vector<std::vector<double>> discretize_path(const std::vector<std::shared_ptr<Node>> &path)
    {
        std::vector<std::vector<double>> discretized_path;
        //iterate over path and discretize between every pair of nodes
        for(size_t i = 1; i < path.size(); ++i)
        {
            const std::shared_ptr<Node> start = path[i-1];
            const std::shared_ptr<Node> end = path[i];

            //perform linear interpolation between start and end point
            double distance = get_distance(start->x, start->y, start->z, end->x, end->y, end->z);
            int numofwaypoints = (int)(distance/(1.0/10.0));
            for(int i = 1; i < numofwaypoints; ++i)
            {
                double x_interp = start->x + (static_cast<double>(i) / (numofwaypoints - 1)) * (end->x - start->x);
                double y_interp = start->y + (static_cast<double>(i) / (numofwaypoints- 1)) * (end->y - start->y);
                double z_interp = start->z + (static_cast<double>(i) / (numofwaypoints - 1)) * (end->z - start->z);
                discretized_path.emplace_back(std::vector<double>{x_interp, y_interp, z_interp});
            }
        }
        return discretized_path;
    }

    //main planning loop here
    void run()
    {
        //define start and goal positions
        std::vector<double> start_pose = {1000.0, 1000.0, 10.0};
        std::vector<double> goal_pose = {0.0, 0.0, 0.0};
        std::vector<double> current_pose = start_pose;

        ROS_INFO_STREAM("Start pose: " << start_pose[0] << " " << start_pose[1] << " " << start_pose[2]);
        ROS_INFO_STREAM("Goal pose: " << goal_pose[0] << " " << goal_pose[1] << " " << goal_pose[2]);

        //global variables
        int waypoint_num = 0; //keep track of waypoint num for path following
        bool reached_goal = false;
        ros::Rate loop_rate(10);
        std::vector<std::shared_ptr<Node>> nodes_list; //keep track of nodes in tree
        ros::Publisher tree_viz_pub = nh.advertise<visualization_msgs::Marker>("tree_visualization", 10);
        ros::Publisher path_viz_pub = nh.advertise<visualization_msgs::Marker>("path_visualization", 10);
        ros::Publisher start_goal_viz_pub = nh.advertise<visualization_msgs::Marker>("start_goal_visualization", 10);
        ros::Subscriber path_begin_sub = nh.subscribe<std_msgs::String>("start_planning", 10, &OrbitalVisualizationNode::start_planning_callback, this);

        //spacecraft visualization
        tf2_ros::TransformBroadcaster broadcaster;

        //create a node for the start pose of the aircraft
        std::shared_ptr<Node> q_init = std::make_shared<Node>(start_pose[0], start_pose[1], start_pose[2], 0.0, 0.0, nullptr);
        nodes_list.push_back(q_init);

        //create a goal node
        std::shared_ptr<Node> q = std::make_shared<Node>(goal_pose[0], goal_pose[1], goal_pose[2], 0, 0, nullptr);

        //create a node for the extended node
        std::shared_ptr<Node> q_extend = std::make_shared<Node>();

        //create node for the node we use to reach the goal
        std::shared_ptr<Node> q_reach_goal = std::make_shared<Node>();

        //store backtracked path and discretized path
        std::vector<std::shared_ptr<Node>> path;
        std::vector<std::vector<double>> discretized_path;

        int num_iters = 0;
        while(ros::ok())
        {
            //visualize tree
            visualization_msgs::Marker tree_viz_msg = visualize_tree_msg(nodes_list, "map");
            //visualize start and goal points
            visualization_msgs::Marker start_goal_viz_msg = visualize_start_goal(q_init, q, "map");

            //publish visualization msgs
            tree_viz_pub.publish(tree_viz_msg);
            start_goal_viz_pub.publish(start_goal_viz_msg);

            //visualize spacecraft here (for now)
            geometry_msgs::TransformStamped spacecraft_transform = move_spacecraft(current_pose);
            broadcaster.sendTransform(spacecraft_transform);

            if(num_iters < MAX_ITERS && !reached_goal && begin_planning)
            {
                //generate random spacecraft configuration
                std::shared_ptr<Node> q_rand = get_random_config(q);

                //extend towards the sampled node
                int status = extend(nodes_list, q_rand, q_extend);

                if(status != TRAPPED)
                {
                    //check to see if we can connect to the goal from our extend node
                    double goal_dist = get_distance(q_extend->x, q_extend->y, q_extend->z, q->x, q->y, q->z);
                    ROS_INFO_STREAM("New node: " << q_extend->x << " " << q_extend->y << " " << q_extend->z << " added. Goal dist: " << goal_dist);

                    std::shared_ptr<Node> q_new = std::make_shared<Node>();
                    if(goal_dist < GOAL_DIST_THRESH) //if we're within epsilon of the goal then check to see if we can extend to it
                    {
                        ROS_INFO_STREAM("REACHED GOAL!");
                        q_reach_goal->x = q->x;
                        q_reach_goal->y = q->y;
                        q_reach_goal->z = q->z;
                        q_reach_goal->cost = q_extend->cost + get_distance(q->x, q->y, q->z, q_extend->x, q_extend->y, q_extend->z);
                        q_reach_goal->depth = q_extend->depth + 1;
                        q_reach_goal->parent_ptr = q_extend;
                        reached_goal = true;
                        //push into nodes list so we can visualize
                        nodes_list.push_back(q_reach_goal);
                        //backtrack path and discretize it further so we can follow it
                        path = backtrack_path(q_reach_goal);
                        discretized_path = discretize_path(path);
                        std::reverse(discretized_path.begin(), discretized_path.end());
                    }

                }
                num_iters += 1;

                ros::spinOnce();
                loop_rate.sleep();
            }

            if(reached_goal)
            {
                //move waypoint along path according to waypoint number
                current_pose = discretized_path[waypoint_num];
                ROS_INFO_STREAM("current pose: " << current_pose[0] << " " << current_pose[1] << " " << current_pose[2]);
                waypoint_num += 1;

                visualization_msgs::Marker path_viz_msg = visualize_path(path, "map");
                path_viz_pub.publish(path_viz_msg);
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

};