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
#define RRT_STAR_NODE_REJECTION_ACTIVE true
#define RRT_STAR_LOCAL_BIAS_ACTIVE true

enum Extent_State {
    Reached,
    Advanced,
    Trapped
};

class RRT_Star_Planner{
	public:
		// Attributes
		int x_size, y_size;
		double* map;
		std::vector<double> start_config;
		std::vector<double> goal_config;
		double ***plan;
		int *plan_length;
		std::vector<std::shared_ptr<Graph_Node>> path;
		int count = 0;
		bool path_found = false;
		int update_count = 0;
		int DOF;

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
		RRT_Star_Planner(std::vector<double>& armstart_anglesV_rad, std::vector<double>& armgoal_anglesV_rad, 
						 int _DOF, double* _map, int _x_size, int _y_size, 
						 double ***_plan, int *_plan_length){
			DOF = _DOF;
			map = _map;
			x_size = _x_size;
			y_size = _y_size;
			tree = Tree(armstart_anglesV_rad);
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
		// Sample State 
		void SampleConfiguration(std::vector<double>& sample_state, bool& path_found);
		// Sample State Goal Biased
		void SampleConfiguration(std::vector<double>& sample_state, std::vector<double>& goal_state, bool& path_found);
		
		// Find nearest states 
		// Need to change distance function in kd_tree.cpp to LQR distance
		std::shared_ptr<Graph_Node> FindNearestStateTo(Tree& tree, std::vector<double>& state);
		std::vector<std::shared_ptr<Graph_Node>> FindNearestStates(Tree& tree, std::vector<double>& new_state);

		// Check if the state is valid
		bool ValidState(std::vector<double>& state);

		// Tree methods
		std::shared_ptr<Graph_Node> CreateTreeNode(Tree& tree, std::vector<double>& state);
		void AddToTree(Tree& tree, std::shared_ptr<Graph_Node> state_node, std::shared_ptr<Graph_Node> parent_state_node);
		
		// Choose Parent
		void ChooseParent(Tree& tree, std::shared_ptr<Graph_Node>& new_state_node, std::shared_ptr<Graph_Node>& parent_node, std::vector<std::shared_ptr<Graph_Node>>& near_nodes);

		// Rewire function
		void Rewire(Tree& tree, std::shared_ptr<Graph_Node>& new_state_node, std::shared_ptr<Graph_Node>& parent_node, std::vector<std::shared_ptr<Graph_Node>>& near_nodes);

		bool _can_connect(const std::vector<double>& alpha_config, const std::vector<double>& neighbor_config, int n_points);
		Extent_State connect(std::vector<double>& config);
		Extent_State extend_to_goal(std::vector<double>& target_config);
		void compute_path(std::shared_ptr<Graph_Node> node);
		std::vector<double> SteerTowards(Tree& tree, std::vector<double>& sample_state, std::shared_ptr<Graph_Node>& nearest_node);
		void _get_q_new(std::vector<double>& q_near, std::vector<double>& q_rand, std::vector<double>& q_new, double step_size);


		void FindPath(std::vector<double>& start_state, std::vector<double>& goal_state);


		// Utils Methods
		void _sample_goal_biased_config(std::vector<double>& sample_state, std::vector<double>& goal_state);
		void _sample_local_biased_config(std::vector<double>& sample_state);
		void _sample_random_config(std::vector<double>& sample_state);
};

#endif // RRT_STAR_H

