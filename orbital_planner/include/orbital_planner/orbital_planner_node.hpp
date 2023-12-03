
#include "ros/ros.h"
#include "std_msgs/String.h"

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
            //tree visualization here -- TODO:
            const Tree* tree = planner.GetTree();

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
                //final path visualization here -- TODO

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
