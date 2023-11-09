#ifndef RRT_STAR_H
#define RRT_STAR_H

#include "utils.hpp"
#include "tree.hpp"
#include "basic_structs.hpp"

/* General params*/
#define DEBUG_DEV false
#define DEBUG_USR false

#define MAX_ANGLE 2*M_PI
#define RRT_STAR_NUM_ITER 1000000 //500000
#define RRT_STAR_INTER_POINTS 15 //15
#define RRT_STAR_STEP_EXTEND 1.0 //4.0
#define RRT_STAR_STEP_EXTEND_GOAL 1.0
#define RRT_STAR_GOAL_TOL 5.0 //2.0
#define RRT_STAR_GOAL_CONNECT 100 // 50

#define RRT_STAR_DEBUG_REWIRING false
#define RRT_STAR_DEBUG_LOCAL false
#define RRT_STAR_DEBUG_INTER false

#define RRT_STAR_GOAL_BIASED 0.5 //0.5
#define RRT_STAR_LOCAL_BIASED 0.2 //0.5
#define RRT_STAR_TIME_MAX 29900
#define RRT_STAR_REF_MAX 4900
#define RRT_STAR_STD_DEV 2.0
#define RRT_STAR_R_MIN 0.1*RRT_STAR_STEP_EXTEND
#define RRT_STAR_R_MAX 0.5*RRT_STAR_STEP_EXTEND

#define RRT_STAR_REW_RADIUS 10.0
#define RRT_STAR_K_NEIGHBORS 8

#define RRT_STAR_KEEP_UNTIL_TIME_MAX false
#define RRT_STAR_NODE_REJECTION_ACTIVE false
#define RRT_STAR_LOCAL_BIAS_ACTIVE false

enum sample_state {
    Reached,
    Advanced,
    Trapped
};

class RRT_Star_Planner{
	public:
		// Attributes
		int DOF, x_size, y_size;
		double* map;
		double* start_config;
		double* goal_config;
		double ***plan;
		int *plan_length;
		std::vector<std::shared_ptr<Graph_Node>> path;
		int count = 0;
		bool path_found = false;
		int update_count = 0;

        int seed = 0; // Change this value as needed
        // Create the Mersenne Twister engine with the fixed seed
        std::mt19937 gen;
        std::normal_distribution<double> distribution_rrt_star;
        std::uniform_real_distribution<double> distribution_local_bias;

		Tree tree;

		std::shared_ptr<Graph_Node> start_node;
		std::shared_ptr<Graph_Node> goal_node;
		std::shared_ptr<Graph_Node> q_new_node;

		// Constructor
		RRT_Star_Planner(double* armstart_anglesV_rad, double* armgoal_anglesV_rad, 
					int _DOF, double* _map, int _x_size, int _y_size, 
					double ***_plan, int *_plan_length){
			DOF = _DOF;
			map = _map;
			x_size = _x_size;
			y_size = _y_size;
			tree = Tree(armstart_anglesV_rad, _DOF);
			start_config = armstart_anglesV_rad;
			goal_config = armgoal_anglesV_rad;
			start_node = tree.list[0];
			start_node->g = 0;
			plan = _plan;
			plan_length = _plan_length;
            gen.seed(seed);
            distribution_rrt_star = std::normal_distribution<double>(0.0, RRT_STAR_STD_DEV);
            distribution_local_bias = std::uniform_real_distribution<double>(RRT_STAR_R_MIN, RRT_STAR_R_MAX);
		}

		// Methods
		void FindPath();
		bool _can_connect(const double* alpha_config, const double* neighbor_config, int n_points);
		sample_state connect(double* config);
		sample_state extend_to_goal(double* target_config);
		void compute_path(std::shared_ptr<Graph_Node> node);
		sample_state extend_with_rewiring(Tree& current_tree, double* q_rand);
		void _get_q_new(double* q_near, double* q_rand, double* q_new, double step_size);

		// Utils Methods
		void SampleConfiguration(double* config);
		void _sample_goal_biased_config(double* config);
		void _sample_local_biased_config(double* config);
		void _sample_random_config(double* config);
};

#endif // RRT_STAR_H

