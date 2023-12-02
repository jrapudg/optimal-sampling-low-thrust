#include "rrt_star.hpp"

using namespace Astrodynamics;


// Constructor
RRT_Star_Planner::RRT_Star_Planner(State& starting_configuration, State& goal_configuration, double ***_plan, int *_plan_length) 
: 
count(0), 
path_found(false), 
update_count(0), 
seed(42), 
plan(_plan),
plan_length(_plan_length),
sim(Simulator(ClohessyWiltshire, SIM_DT)),
Q(Eigen::MatrixXd::Identity(6, 6) * 5),
R(Eigen::MatrixXd::Identity(3, 3)),
lqr(LQR(Q, R))
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

    // Dynamics initialization
    GetClohessyWiltshireMatrices(A, B);
    Simulator::Discretize(A, B, SIM_DT, Ad, Bd);


    // LQR Initialization
    lqr.ComputeCostMatrix(Ad, Bd, S);
    K = lqr.ComputeOptimalGain(Ad, Bd, S);

    std::cout << "Finished initialization." << std::endl;
}





// Methods
void RRT_Star_Planner::ComputePath(std::shared_ptr<Graph_Node> node){
    std::cout << "Computing path ..." << std::endl;
    int DOF = start_node->config.rows();
    path.clear();

    path.insert(path.begin(), node);

    while(node->parent != start_node){
        Print(node->config, "State");
        node = node->parent;
        //Print(node->config, "Parent");
        path.insert(path.begin(), node);
    }
    path.insert(path.begin(), start_node);

    //no plan by default
    *plan = NULL;
    *plan_length = 0;
        
    *plan = (double**) malloc(path.size()*sizeof(double*));
    for (int i = 0; i < path.size(); i++){
        (*plan)[i] = (double*) malloc(DOF*sizeof(double)); 
        for(int j = 0; j < DOF; j++){
            (*plan)[i][j] = (path[i])->config[j];
        }
        /*if (i != 0){
            (path[i])->g = (path[i])->parent->g + lqr.GetTrajectoryCost((path[i])->parent->config, path[i]->config, Ad, Bd, sim, SIM_COST_TOL);
        }*/
    }    
    *plan_length = path.size();
}



void RRT_Star_Planner::FindPath(State& start_state, State& goal_state){
    
    auto start = std::chrono::high_resolution_clock::now();
    State state_rand;
    std::shared_ptr<Graph_Node> parent_node;


    for (int i = 0; i < RRT_STAR_NUM_ITER; i++) { // RRT_STAR_NUM_ITER
        
        std::cout << "Iteration " << i << std::endl;



        // Sample 
        SampleConfiguration(state_rand, goal_state, path_found);
        //Print(state_rand, "Sampled configuration");

        // TO DELETE AFTER 
        /*State test;
        SampleConfiguration(test, goal_state, path_found);
        CreateTreeNode(tree, test);*/


        // LQRNearest
        // TODO For other dynamics, locally linearize around sampled_state and 0 control 
        // TODO Then, compute the S matrix for the search of the nearest node
        std::shared_ptr<Graph_Node> nearest_node = FindNearestStateTo(tree, state_rand, S);
        //Print(nearest_node->config, "FindNearestStateTo node");
        //std::cout << "Cost of nearest node: " << nearest_node->g << std::endl;



        // LQRSteer
        // This new state represents the end of a trajectory executed with the LQR-Policy generated from the LQRNearest above with a pre-specified step size based on the cost
        // TODO Compute the optimal gain K based on the S from sampled state
        State new_state;
        SteerTowards(tree, state_rand, nearest_node, new_state);
        //Print(new_state, "New state after steering");



        // Initialize new state as a node
        //std::cout << "CreateTreeNode" << std::endl;
        new_state_node = CreateTreeNode(tree, new_state);



        // LQRNear
        // Setting up local LQR cost and policy on new_state
        // TODO Linearize around new_state (A and B)
        // TODO Compute subsequent S and K (around new_state as mentioned above)
        //std::cout << "FindNearestStates -" << std::endl;
        std::vector<std::shared_ptr<Graph_Node>> near_nodes = FindNearestStates(tree, new_state, S);



        // ChooseParent
        // Should give the newly computed S
        //std::cout << "ChooseParent" << std::endl;
        ChooseParent(tree, new_state_node, parent_node, near_nodes, S);
        //Print(parent_node->config, "Parent");
        //std::cout << "Node cost with that parent " << new_state_node->g << std::endl;


        // AddToTree
        //std::cout << "AddToTree" << std::endl;
        AddToTree(tree, new_state_node, parent_node);
        // For now, we assume collision-free. For collision-checking, just check along a spline interpolation of the state instead? 
        // Or more discrete simulation? 

        // Rewiring
        //std::cout << "Rewiring" << std::endl;
        //Rewire(tree, new_state_node, parent_node, near_nodes);

        std::cout << "Close to the goal mf: " << config_distance(new_state_node->config, goal_config, S) << std::endl;
        if (!path_found && are_configs_close(new_state_node->config, goal_config, S, RRT_STAR_GOAL_TOL))
        {


            std::cout << "Goal Node Reached with " << i + 1 << " samples!" << std::endl;
        
            goal_node = CreateTreeNode(tree, goal_config);
            goal_node->g = 0;
            AddToTree(tree, goal_node, new_state_node);
            
            std::cout << "GOAL COST: " << goal_node->g << std::endl;


            ComputePath(goal_node);
            std::cout << "PATH SIZE: " << path.size() << std::endl;
            path_found = true;
            std::cout << "RESULT -> SOLUTION FOUND!" << std::endl;
            auto end = std::chrono::high_resolution_clock::now();
            auto time_elapsed_milli = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            std::cout << "TIME: " << (double) time_elapsed_milli/1000 << std::endl;
            break;

        }



        //std::cout << "--------" << std::endl;

    }

    /*for (int i = 0; i < RRT_STAR_NUM_ITER; i++) { 
        // Sample 
        std::cout << "Sample Configuration" << std::endl;
        SampleConfiguration(state_rand, goal_state, path_found);
        std::cout << "Sample ";
        print_config(state_rand);



        // LQRNearest
        std::cout << "FindNearestStateTo" << std::endl;
        auto nearest_node = FindNearestStateTo(tree, state_rand, S);
        std::cout << "Start ";
        print_config(nearest_node->config);
        std::cout << "Cost " << nearest_node->g << std::endl;

        // LQRSteer
        std::cout << "SteerTowards" << std::endl;
        State new_state;
        SteerTowards(tree, state_rand, nearest_node, new_state);
        std::cout << "New State ";
        print_config(new_state);

        std::cout << "Start ";
        print_config(nearest_node->config);
        std::cout << "Cost " << nearest_node->g << std::endl;

        // Initialize new state as a node
        std::cout << "CreateTreeNode" << std::endl;
        new_state_node = CreateTreeNode(tree, new_state);

        // LQRNear
        std::cout << "FindNearestStates" << std::endl;
        auto near_nodes = FindNearestStates(tree, new_state, S);
        
        std::cout << "ChooseParent" << std::endl;
        ChooseParent(tree, new_state_node, parent_node, near_nodes);
        std::cout << "Parent: ";
        print_config(parent_node->config);

        std::cout << "AddToTree" << std::endl;
        AddToTree(tree, new_state_node, parent_node);
        std::cout << "Rewire" << std::endl;
        Rewire(tree, new_state_node, parent_node, near_nodes);

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
}
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

void RRT_Star_Planner::Step(MatrixA& A, MatrixB& B, MatrixK& K, const State& state, State& target_state, State& next_state)
{
    // state: current state
    // target_state: state to drive towards
    // next_state: container that will contain the new state
    
    // You should provide the A and B
    // you can change eps obviously, I put a random default value

    //MatrixS S;
    //lqr.ComputeCostMatrix(A, B, S);
    //K = lqr.ComputeOptimalGain(A, B, S);

    Control u;
    State current_state = state;

    // The "epsilon" here should be the dt here, that is already selected 
    u = - K * (current_state - target_state);
    sim.Step(current_state, u, next_state);
}


void RRT_Star_Planner::SteerTowards(Tree& tree, State& sample_state, std::shared_ptr<Graph_Node>& nearest_node, State& next_state){
    //Print(nearest_node->config, "Steering from");
    Step(Ad, Bd, K, nearest_node->config, sample_state, next_state);
}



void RRT_Star_Planner::ChooseParent(Tree& tree, 
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
        if (near_node->index != new_state_node->index)
        {

            // The cost here should the cost-to-go
            current_cost = near_node->g + config_distance(near_node->config, new_state_node->config, S);

            if (current_cost < new_state_node->g)
            {   
                // Changing the cost and parent here
                parent_node = near_node;
                new_state_node->g = current_cost;
            }
        }

    }
 
}


void RRT_Star_Planner::Rewire(Tree& tree, 
                            std::shared_ptr<Graph_Node>& new_state_node, 
                            std::shared_ptr<Graph_Node>& parent_node, 
                            std::vector<std::shared_ptr<Graph_Node>>& near_nodes,
                            MatrixS& S_new)
{
    for (auto& near_node : near_nodes)
    {
        
        // Not sure if we have to linearize around near_node then compute S_near here (and not use the S_new)
        
        if ((near_node != new_state_node) && (near_node != parent_node) && 
            (near_node->g > new_state_node->g + config_distance(new_state_node->config, near_node->config, S)))
        {
            
            if (RRT_STAR_DEBUG_REWIRING)
            {
                std::cout << "Rewiring ..." << std::endl;
                //std::cout << "Cost near: " << near_node->g << " Cost new: " << new_state_node->g + config_distance(new_state_node->config, near_node->config, S) << std::endl;
                Print(new_state_node->config, "Parent");
                Print(near_node->config, "Child near");
            }


            near_node->parent = new_state_node;
            near_node->g = new_state_node->g + config_distance(new_state_node->config, near_node->config, S);


        }
    }
}



std::shared_ptr<Graph_Node> RRT_Star_Planner::CreateTreeNode(Tree& tree, State& state){
    return tree.add_vertex(state);
};

void RRT_Star_Planner::AddToTree(Tree& tree, std::shared_ptr<Graph_Node> state_node, std::shared_ptr<Graph_Node> parent_state_node){
    tree.add_edge(state_node, parent_state_node);
};

bool RRT_Star_Planner::ValidState(State& state){
    return true;
}

std::shared_ptr<Graph_Node> RRT_Star_Planner::FindNearestStateTo(Tree& tree, State& state, MatrixS& S){
    return tree.find_nearest_neighbor(state, S);
};

std::vector<std::shared_ptr<Graph_Node>> RRT_Star_Planner::FindNearestStates(Tree& tree, State& new_state, MatrixS& S){
    return tree.find_neighbors_within_radius(new_state, S, RRT_STAR_NEW_RADIUS, RRT_STAR_K_NEIGHBORS);
};

// INTEGRATION OF ORBITAL SAMPLER
void RRT_Star_Planner::SampleConfiguration(State& sample_state, State& goal_state, bool& path_found){

    double p_value = static_cast<double>(rand()) / RAND_MAX;
    if (p_value < RRT_STAR_GOAL_BIASED)
    {
        sample_state = goal_state;
    }
    else
    {
        sampler.SampleCW(sample_state);
    }
    
}




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
void RRT_Star_Planner::_sample_goal_biased_config(State& sample_state, State& goal_state){
    if(RRT_STAR_DEBUG_LOCAL){
        std::cout << "GOAL sampling ..." << std::endl;
    }

    for (int i = 0; i < goal_state.rows(); i++){ 
        sample_state[i] = goal_state[i] + distribution_rrt_star(gen);
    }
}

void RRT_Star_Planner::_sample_local_biased_config(State& sample_state){
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
};
