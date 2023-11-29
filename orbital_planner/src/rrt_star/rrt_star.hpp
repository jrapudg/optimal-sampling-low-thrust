#ifndef RRT_STAR_H
#define RRT_STAR_H

#include "utils.hpp"
#include "tree.hpp"
#include "basic_structs.hpp"
#include "../dynamics/lqr.hpp"
#include "../dynamics/simulation.hpp"

/* General params*/
#define DEBUG_DEV false
#define DEBUG_USR false

#define MAX_ANGLE 2*M_PI
#define RRT_STAR_NUM_ITER 1000000 //500000
#define RRT_STAR_STEP_EXTEND 1.0 //4.0
#define RRT_STAR_GOAL_TOL 0.1 //2.0

#define RRT_STAR_DEBUG_REWIRING false
#define RRT_STAR_DEBUG_LOCAL false
#define RRT_STAR_DEBUG_INTER false

#define RRT_STAR_GOAL_BIASED 0.5 //0.5
#define RRT_STAR_LOCAL_BIASED 0.2 //0.5
#define RRT_STAR_TIME_MAX 2990000000000000
#define RRT_STAR_REF_MAX 49000000000000000
#define RRT_STAR_STD_DEV 2.0
#define RRT_STAR_R_MIN 0.1*RRT_STAR_STEP_EXTEND
#define RRT_STAR_R_MAX 0.5*RRT_STAR_STEP_EXTEND

#define RRT_STAR_REW_RADIUS 10.0
#define RRT_STAR_K_NEIGHBORS 10

#define RRT_STAR_KEEP_UNTIL_TIME_MAX false
#define RRT_STAR_NODE_REJECTION_ACTIVE true //not implemented
#define RRT_STAR_LOCAL_BIAS_ACTIVE true 

#define SIM_DT 0.1
#define SIM_COST_TOL 1.0
#define MAX_EXTENT_TRIALS 80

using namespace Astrodynamics;
using namespace Optimal;
using namespace Simulation;

class RRT_Star_Planner{
	public:
		// Attributes
		State start_config;
		State goal_config;
		double ***plan;
		int *plan_length;
		std::vector<std::shared_ptr<Graph_Node>> path;
		int count = 0;
		bool path_found = false;
		int update_count = 0;

		// Sampling
        int seed = 42; // Change this value as needed
        // Create the Mersenne Twister engine with the fixed seed
        std::mt19937 gen;
        std::normal_distribution<double> distribution_rrt_star;
        std::uniform_real_distribution<double> distribution_local_bias;
		OrbitalSampler sampler = OrbitalSampler(seed);

		// Dynamics
		MatrixA A;
		MatrixB B;
		MatrixA Ad;
		MatrixB Bd;

		Simulator sim = Simulator(ClohessyWiltshire, SIM_DT);

		// LQR
		MatrixQ Q;
		MatrixR R;
		MatrixS S;
		MatrixK K;

		LQR lqr = LQR(Q, R);

		// Tree
		Tree tree;
		std::shared_ptr<Graph_Node> start_node;
		std::shared_ptr<Graph_Node> goal_node;
		std::shared_ptr<Graph_Node> new_state_node;

		// Constructor
		RRT_Star_Planner(State& armstart_anglesV_rad, State& armgoal_anglesV_rad, 
						 double ***_plan, int *_plan_length){
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

			// Dynamics initialization
			GetClohessyWiltshireMatrices(A, B);
			Simulator::Discretize(A, B, SIM_DT, Ad, Bd);
			// LQR Initialization
			Q = Eigen::MatrixXd::Identity(6, 6) * 5;
			R = Eigen::MatrixXd::Identity(3, 3);

			lqr = LQR(Q, R);
			lqr.ComputeCostMatrix(Ad,Bd, S);
		}

		// Methods
		// Sample State 
		void SampleConfiguration(State& sample_state, bool& path_found);
		// Sample State Goal Biased
		void SampleConfiguration(State& sample_state, State& goal_state, bool& path_found);
		
		// Find nearest states 
		// Need to change distance function in kd_tree.cpp to LQR distance
		std::shared_ptr<Graph_Node> FindNearestStateTo(Tree& tree, State& state, MatrixS& S);
		std::vector<std::shared_ptr<Graph_Node>> FindNearestStates(Tree& tree, State& new_state, MatrixS& S);

		// Check if the state is valid
		bool ValidState(State& state);

		// Tree methods
		std::shared_ptr<Graph_Node> CreateTreeNode(Tree& tree, State& state);
		void AddToTree(Tree& tree, std::shared_ptr<Graph_Node> state_node, std::shared_ptr<Graph_Node> parent_state_node);
		
		// Choose Parent
		void ChooseParent(Tree& tree, std::shared_ptr<Graph_Node>& new_state_node, std::shared_ptr<Graph_Node>& parent_node, std::vector<std::shared_ptr<Graph_Node>>& near_nodes);

		// Rewire function
		void Rewire(Tree& tree, std::shared_ptr<Graph_Node>& new_state_node, std::shared_ptr<Graph_Node>& parent_node, std::vector<std::shared_ptr<Graph_Node>>& near_nodes);

		void ComputePath(std::shared_ptr<Graph_Node> node);
		void SteerTowards(Tree& tree, State& sample_state, std::shared_ptr<Graph_Node>& nearest_node);
		void Step(MatrixA& A, MatrixB& B, const State& state, State& next_state, double eps=2);

		void FindPath(State& start_state, State& goal_state);


		// Utils Methods
		void _sample_goal_biased_config(State& sample_state, State& goal_state);
		void _sample_local_biased_config(State& sample_state);
		void _sample_random_config(State& sample_state);
};

#endif // RRT_STAR_H

