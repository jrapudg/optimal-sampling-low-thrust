#include "rrt_star.hpp"
#include "../dynamics/jacobians.hpp"

using namespace Astrodynamics;


// Constructor
RRTStar::RRTStar(State& starting_configuration, State& goal_configuration, double ***_plan, int *_plan_length) 
: 
count(0), 
path_found(false), 
update_count(0), 
seed(42), 
plan(_plan),
plan_length(_plan_length),
sim(Simulator(NonlinearRelativeKeplerianDynamics, SIM_DT)),
Q(Eigen::MatrixXd::Identity(6, 6)),
R(Eigen::MatrixXd::Identity(3, 3)),
lqr(LQR(Q,R))
{
    std::cout << "Status query" << std::endl;
    tree = Tree(starting_configuration);
    start_config = starting_configuration;
    goal_config = goal_configuration;
    start_node = tree.list[0];
    start_node->g = 0;

    // Sampling initialization
    if (seed == 0)
    {
        // From hardware
        std::random_device rd;
        gen = std::mt19937(rd());
    } else 
    {
        gen = std::mt19937(seed);
    }

    distribution_rrt_star = std::normal_distribution<double>(0.0, RRT_STAR_STD_DEV);
    distribution_local_bias = std::uniform_real_distribution<double>(RRT_STAR_R_MIN, RRT_STAR_R_MAX);
    sampler = OrbitalSampler(seed);

    lqr = LQR(Q, R);
    //std::cout << "S: " << S << std::endl;

    std::cout << "Finished initialization." << std::endl;
}


void RRTStar::BacktrackToStart(std::vector<std::shared_ptr<Graph_Node>>& path, std::shared_ptr<Graph_Node> last_node_ptr)
{

    // Push_back amortized constant while insert is log
    std::shared_ptr<Graph_Node> node = last_node_ptr;

    path.clear();
    path.push_back(node);
    while(node->parent != start_node){
        Print(node->config, "State");
        node = node->parent;
        //Print(node->config, "Parent");
        path.push_back(node);
    }
    path.push_back(start_node);

    // Reversing the reversed plan
    std::reverse(path.begin(), path.end());

}


double RRTStar::ComputeQuadraticCost(const std::vector<std::shared_ptr<Graph_Node>>& path, const State& goal_state)
{
    double cost = 0.0;

    // For now just state cost
    // In reality, we should use LQR policies locally for each nodes at a fixed dt (so finer resolution) and compute the quadratic cost
    // We don't necessarily need the policy, just neeed to ensure the path is dynamically feasible
    for (auto node : path)
    {
        // lqr.QuadraticCost(state, control)
        cost += ((node->config - goal_state).transpose() * Q * (node->config - goal_state))(0,0);
    }
    return cost;
    
}



double RRTStar::ComputePath(std::vector<State>& state_path)
{
    
    std::cout << "Computing path ..." << std::endl;

    // Must check if path if found first 
    if (!PathFound())
    {
        throw std::runtime_error("Path isn't found yet.");
    }


    int DOF = start_node->config.rows();

    // Get the path - could combine the steps 
    BacktrackToStart(path, goal_node);
    state_path.clear();
    for (auto node : path)
    {
        state_path.push_back(node->config);
    }

    // Compute quadratic cost 
    double cost = ComputeQuadraticCost(path, goal_config);


    // TODO remove this 
    *plan = NULL;
    *plan_length = 0;
        
    *plan = (double**) malloc(path.size()*sizeof(double*));
    for (int i = 0; i < path.size(); i++){
        (*plan)[i] = (double*) malloc(DOF*sizeof(double)); 
        for(int j = 0; j < DOF; j++){
            (*plan)[i][j] = (path[i])->config[j];
        }

    }    
    *plan_length = path.size();

    return cost;
}



void RRTStar::Iterate()
{
    
    
    State sampled_state;
    std::shared_ptr<Graph_Node> parent_node;


    // Sample 
    SampleConfiguration(sampled_state, goal_config, path_found);
    Print(sampled_state, "Sampled configuration");


    // LQRNearest
    // Locally linearize around sampled_state and 0 control 
    UpdateLinearizedDiscreteDynamicsAround(sampled_state, zero_control);
    // Compute the S matrix for the search of the nearest node
    lqr.ComputeCostMatrix(Ad, Bd, S);

    // Add node rejection latter
    if (path_found & RRT_STAR_NODE_REJECTION_ACTIVE){
        double cost_to_start = config_distance(start_node->config, sampled_state, S); 
        double cost_to_goal = config_distance(sampled_state, goal_node->config, S);

        std::cout << "Current Path Cost: " << cost_goal << std::endl;
        std::cout << "Current Sample Cost: " << cost_to_goal + cost_to_start << std::endl;
        if (cost_to_goal + cost_to_start > 3*cost_goal){
            std::cout << "Sample Rejected!" << std::endl;
            return;
        }
        std::cout << "Sample Accepted!" << std::endl;
    }

    std::shared_ptr<Graph_Node> nearest_node = FindNearestStateTo(tree, sampled_state, S);
    //Print(nearest_node->config, "FindNearestStateTo node");

    // LQRSteer
    // This new state represents the end of a trajectory executed with the LQR-Policy generated from the LQRNearest above with a pre-specified step size based on the cost
    // Compute the optimal gain K based on the S from sampled state
    K = lqr.ComputeOptimalGain(Ad, Bd, S);
    //std::cout << "Optimal Gain: " << K << std::endl;
    State new_state;
    Control policy_used;
    SteerTowards(tree, sampled_state, nearest_node, new_state, policy_used);
    Print(new_state, "After steering");

    // Initialize new state as a node
    //std::cout << "CreateTreeNode" << std::endl;
    new_state_node = CreateTreeNode(tree, new_state);

    // LQRNear
    // Setting up local LQR cost and policy on new_state
    // Linearize around new_state (A and B)
    UpdateLinearizedDiscreteDynamicsAround(new_state, zero_control);
    // Compute subsequent S and K (around new_state as mentioned above)
    lqr.ComputeCostMatrix(Ad, Bd, S);
    //std::cout << "FindNearestStates -" << std::endl;
    std::vector<std::shared_ptr<Graph_Node>> near_nodes = FindNearestStates(tree, new_state, S);

    // ChooseParent
    // Give the newly computed S
    //std::cout << "-- ChooseParent" << std::endl;
    ChooseParent(tree, new_state_node, parent_node, near_nodes, S);
    // Supposed to be returning the trajectory for collision checking
    // For now, we assume collision-free. For collision-checking, just check along a spline interpolation of the state instead? 

    // AddToTree
    //std::cout << "AddToTree" << std::endl;
    AddToTree(tree, new_state_node, parent_node, S);
    Print(parent_node->config, "Parent");
    //std::cout << "Node cost with that parent " << new_state_node->g << std::endl;
    
    // Rewiring
    //std::cout << "Rewiring" << std::endl;
    Rewire(tree, new_state_node, parent_node, near_nodes);


    // S used here must be around the goal state
    UpdateLinearizedDiscreteDynamicsAround(goal_config, zero_control);
    lqr.ComputeCostMatrix(Ad, Bd, S);

    std::cout << "Cost-to-go w.r.t to the GOAL: " << config_distance(new_state_node->config, goal_config, S) << std::endl;
    if (!path_found && are_configs_close(new_state_node->config, goal_config, S, RRT_STAR_GOAL_TOL))
    { 
        goal_node = CreateTreeNode(tree, goal_config);
        goal_node->g = 0;
        AddToTree(tree, goal_node, new_state_node, S);
        path_found = true;
    }

    std::cout << "Tree size: " << tree.list.size() << std::endl;
    //std::cout << "--------" << std::endl;

}

bool RRTStar::PathFound()
{
    return path_found;
}

const Tree* RRTStar::GetTree() const 
{
    return &tree; 
}

void RRTStar::Reset()
{
    path_found = false;
    // TODO clear all vectors and stuff
}



void RRTStar::UpdateLinearizedDiscreteDynamicsAround(const State& x, const Control& u)
{
    // Get continuous dnamics matrices
    GetJacobiansNRKD(A, B, x, u);
    // Get corresponding discrete dynamics
    sim.Discretize(A, B, SIM_DT, Ad, Bd);

}

void RRTStar::SteerTowards(Tree& tree, State& sample_state, std::shared_ptr<Graph_Node>& nearest_node, State& next_state, Control& policy){
    //Print(nearest_node->config, "Steering from");
    Step(Ad, Bd, K, nearest_node->config, sample_state, next_state, policy);
}



void RRTStar::Step(MatrixA& A, MatrixB& B, MatrixK& K, const State& current_state, State& target_state, State& next_state, Control& control)
{
    // current_state: current state
    // target_state: state to drive towards
    // next_state: container that will contain the new state
    // control: container to get back the policy used
    
    // The "epsilon" here should be the dt here, that is already selected 
    control = - K * (current_state - target_state);
    sim.Step(current_state, control, next_state);
}




void RRTStar::ChooseParent(Tree& tree, 
                                    std::shared_ptr<Graph_Node>& new_state_node, 
                                    std::shared_ptr<Graph_Node>& parent_node, 
                                    std::vector<std::shared_ptr<Graph_Node>>& near_nodes,
                                    MatrixS& S_new)
{
    // New state node is exactly the new node added to the tree
    // Parent node is a container for the chosen parent 
    // near_nodes are the nearest nodes found around new_state_node
    // S_new is the cost matrix around new_state_node
    
    double current_cost;

    for (auto near_node : near_nodes)
    {   
        if (!path_found){
            if (near_node->index != new_state_node->index)
            {
                // The cost here should the cost-to-go
                current_cost = near_node->g + config_distance(near_node->config, new_state_node->config, S_new);

                if (current_cost < new_state_node->g)
                {   
                    // Changing the cost and parent here
                    parent_node = near_node;
                    new_state_node->g = current_cost;
                }
            }
        }
        else{
            if ((near_node->index != new_state_node->index) && (near_node->index != goal_node->index))
            {
                // The cost here should the cost-to-go
                current_cost = near_node->g + config_distance(near_node->config, new_state_node->config, S_new);

                if (current_cost < new_state_node->g)
                {   
                    // Changing the cost and parent here
                    parent_node = near_node;
                    new_state_node->g = current_cost;
                }
            }
        }
    }
}


void RRTStar::Rewire(Tree& tree, 
                            std::shared_ptr<Graph_Node>& new_state_node, 
                            std::shared_ptr<Graph_Node>& parent_node, 
                            std::vector<std::shared_ptr<Graph_Node>>& near_nodes)
{
    /// Container for each computation
    MatrixA At;
    MatrixB Bt;
    //MatrixS St;

    MatrixA Adt;
    MatrixB Bdt;

    //std::cout << "Rewiring ..." << std::endl;
    for (auto& near_node : near_nodes)
    {
        
        // Use of pre-computed S
        if (!path_found){
            if ((near_node != new_state_node) && (near_node != parent_node) && 
            (near_node->g > new_state_node->g + config_distance(new_state_node->config, near_node->config, near_node->S)))
            {
                
                if (RRT_STAR_DEBUG_REWIRING)
                {
                    
                    //std::cout << "Cost near: " << near_node->g << " Cost new: " << new_state_node->g + config_distance(new_state_node->config, near_node->config, S) << std::endl;
                    Print(new_state_node->config, "Parent");
                    Print(near_node->config, "Child near");
                }


                near_node->parent = new_state_node;
                near_node->g = new_state_node->g + config_distance(new_state_node->config, near_node->config, near_node->S);


            }
        }
        else{
            if ((near_node != new_state_node) && (near_node != parent_node) && (near_node != parent_node) && 
            (near_node->g > new_state_node->g + config_distance(new_state_node->config, near_node->config, near_node->S)))
            {
                
                if (RRT_STAR_DEBUG_REWIRING)
                {
                    
                    //std::cout << "Cost near: " << near_node->g << " Cost new: " << new_state_node->g + config_distance(new_state_node->config, near_node->config, S) << std::endl;
                    Print(new_state_node->config, "Parent");
                    Print(near_node->config, "Child near");
                }


                near_node->parent = new_state_node;
                near_node->g = new_state_node->g + config_distance(new_state_node->config, near_node->config, near_node->S);


            }
        } 
    }
}



std::shared_ptr<Graph_Node> RRTStar::CreateTreeNode(Tree& tree, State& state){
    return tree.add_vertex(state);
};

void RRTStar::AddToTree(Tree& tree, std::shared_ptr<Graph_Node> state_node, std::shared_ptr<Graph_Node> parent_state_node, MatrixS& S){
    tree.add_edge(state_node, parent_state_node, S);
};

bool RRTStar::ValidState(State& state){
    return true;
}

std::shared_ptr<Graph_Node> RRTStar::FindNearestStateTo(Tree& tree, State& state, MatrixS& S){
    return tree.find_nearest_neighbor(state, S);
};

std::vector<std::shared_ptr<Graph_Node>> RRTStar::FindNearestStates(Tree& tree, State& new_state, const MatrixS& S){
    return tree.find_neighbors_within_radius(new_state, S, RRT_STAR_NEW_RADIUS, RRT_STAR_K_NEIGHBORS);
};

// INTEGRATION OF ORBITAL SAMPLER
void RRTStar::SampleConfiguration(State& sample_state, State& goal_state, bool& path_found){

    double p_value = static_cast<double>(rand()) / RAND_MAX;
    if ((p_value < RRT_STAR_GOAL_BIASED) && (!path_found))
    {
        sample_state = goal_state;
    }
    else
    {
        sampler.SampleCW(sample_state);
    }
    
}


    /*

        if(path_found){
            update_count++;
            if (update_count == 5){
                ComputePath(goal_node);
                update_count = 0;
            }
        }

        if (!path_found & are_configs_close(new_state_node->config, goal_config, S, RRT_STAR_GOAL_TOL)){
            std::cout << "Goal Node Reached!" << std::endl;
            std::cout << "Num Samples: "<< i + 1 << std::endl;
        
            goal_node = CreateTreeNode(tree, goal_config);
            goal_node->g = 0;
            AddToTree(tree, goal_node, new_state_node);
            
            std::cout << "GOAL COST: " << goal_node->g << std::endl;
            ComputePath(goal_node);
            std::cout << "GOAL COST: " << goal_node->g << std::endl;
            std::cout << "PATH SIZE: " << path.size() << std::endl;
            path_found = true;
            std::cout << "RESULT -> SOLUTION FOUND!" << std::endl;
            auto end = std::chrono::high_resolution_clock::now();
            auto time_elapsed_milli = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            std::cout << "TIME: " << (double) time_elapsed_milli/1000 << std::endl;
            std::cout << "Graph Nodes Before Refinement: " << tree.list.size() << std::endl;
            if (!RRT_STAR_LOCAL_BIAS_ACTIVE & !RRT_STAR_NODE_REJECTION_ACTIVE & !RRT_STAR_KEEP_UNTIL_TIME_MAX){
                std::cout << "NO REFINEMENT ACTIVE" << std::endl;
                return;
            }
            std::cout << "Refining path ..." << std::endl;
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto time_elapsed_milli = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        if (path_found){
  
            if (time_elapsed_milli >= RRT_STAR_REF_MAX){
                std::cout << "TIME IS OVER! " << std::endl;
                std::cout << "Refinement DONE! " << std::endl;
                ComputePath(goal_node);
                std::cout << "NEW GOAL COST: " << goal_node->g << std::endl;
                return;
            }
        }
        
        if (time_elapsed_milli >= RRT_STAR_TIME_MAX){
            std::cout << "TIME IS OVER! " << std::endl;
            return;
        }
        std::cout << "Nodes count: " << tree.list.size() << std::endl;
    }*/


// Generate a random number between 0 and 1
/*double p_value = static_cast<double>(rand()) / RAND_MAX;
if (RRT_STAR_LOCAL_BIAS_ACTIVE & (p_value < RRT_STAR_LOCAL_BIASED) & path_found){
    _sample_local_biased_config(sample_state);
}
else{
    //_sample_random_config(sample_state);
    sampler.SampleCW(sample_state);
}*/

// Utils Methods
/*void RRTStar::_sample_goal_biased_config(State& sample_state, State& goal_state){
    if(RRT_STAR_DEBUG_LOCAL){
        std::cout << "GOAL sampling ..." << std::endl;
    }

    for (int i = 0; i < goal_state.rows(); i++){ 
        sample_state[i] = goal_state[i] + distribution_rrt_star(gen);
    }
}

void RRTStar::_sample_local_biased_config(State& sample_state){
    if(RRT_STAR_DEBUG_LOCAL){
        std::cout << "LOCAL sampling ..." << std::endl;
    }
    // Generate a random number between 0 and Tree size
    std::uniform_int_distribution<int> distribution_node(2, path.size()-1);
    int random_node_index = distribution_node(gen);
    State q_tmp;
    double q_tmp_norm;

    if(RRT_STAR_DEBUG_LOCAL){
        std::cout << "Node definition" << std::endl;
        std::cout << "Random index: " << random_node_index << std::endl;
    }

    auto q_2_node = path[random_node_index];
    auto q_node = q_2_node->parent;
    auto q_1_node = q_node->parent;

    if(RRT_STAR_DEBUG_LOCAL){
        print_config(q_1_node->config);
        print_config(q_node->config);
        print_config(q_2_node->config);
        std::cout << "Temporal node" << std::endl;
    }
    
    for (int i=0; i<sample_state.rows(); i++){
        q_tmp[i] = (q_1_node->config[i] + q_2_node->config[i])/2 - q_node->config[i];			
    }

    if(RRT_STAR_DEBUG_LOCAL){
        print_config(q_tmp);
        std::cout << "Norm of temporal" << std::endl;
    }

    q_tmp_norm = calculate_norm(q_tmp);
    if (q_tmp_norm < 0.001){
        q_tmp_norm  = 1;
    }

    if(RRT_STAR_DEBUG_LOCAL){
        std::cout << "Sample calculation" << std::endl;
    }

    for (int i=0; i < sample_state.rows(); i++){
        sample_state[i] = q_node->config[i] + q_tmp[i]*distribution_local_bias(gen)/q_tmp_norm;
    }

    if(RRT_STAR_DEBUG_LOCAL){
        print_config(sample_state);
        std::cout << "Local sampling DONE!" << std::endl;
    }
};*/



// Add node rejection latter
/*
if (path_found & RRT_STAR_NODE_REJECTION_ACTIVE){
    double cost_to_goal = config_distance(sample_state, goal_node->config);
    double cost_to_start = config_distance(sample_state, start_node->config); 
    if (cost_to_goal + cost_to_start > goal_node->g){
        //std::cout << "Sample rejected " << i << std::endl;
        continue;
    }
}
*/
