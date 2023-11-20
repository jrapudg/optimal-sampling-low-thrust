#include "rrt_star.hpp"

// Methods
void RRT_Star_Planner::ComputePath(std::shared_ptr<Graph_Node> node){
    //std::cout << "Computing path ..." << std::endl;
    path.clear();
    path.insert(path.begin(), node);
    while(node->parent != start_node){
        node = node->parent;
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
        if (i != 0){
            (path[i])->g = (path[i])->parent->g + GetTrajectoryCost((path[i])->parent->config, path[i]->config);
        }
    }    
    *plan_length = path.size();
}

void RRT_Star_Planner::FindPath(std::vector<double>& start_state, std::vector<double>& goal_state){
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<double> state_rand(start_state.size());
    std::shared_ptr<Graph_Node> parent_node;

    for (int i = 0; i < RRT_STAR_NUM_ITER; i++) { 
        // Sample 
        SampleConfiguration(state_rand, goal_state, path_found);

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

        // LQRNearest
        auto nearest_node = FindNearestStateTo(tree, state_rand);

        // LQRSteer
        auto new_state = SteerTowards(tree, state_rand, nearest_node);

        // Initialize new state as a node
        q_new_node = CreateTreeNode(tree, new_state);

        // LQRNear
        auto near_nodes = FindNearestStates(tree, new_state);
        
        ChooseParent(tree, q_new_node, parent_node, near_nodes);
        AddToTree(tree, q_new_node, parent_node);
        Rewire(tree, q_new_node, parent_node, near_nodes);

        if(path_found){
            update_count++;
            if (update_count == 5){
                ComputePath(goal_node);
                update_count = 0;
            }
        }

        if (!path_found & are_configs_close(q_new_node->config, goal_config, RRT_STAR_GOAL_TOL)){
            std::cout << "Goal Node Reached!" << std::endl;
            std::cout << "Num Samples: "<< i + 1 << std::endl;
        
            goal_node = CreateTreeNode(tree, goal_config);
            AddToTree(tree, goal_node, q_new_node);
            
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
    }
}
 
std::vector<double> RRT_Star_Planner::SteerTowards(Tree& tree, std::vector<double>& sample_state, std::shared_ptr<Graph_Node>& nearest_node){
    std::vector<double> q_new(sample_state.size());
    
    return q_new;
}

void RRT_Star_Planner::Rewire(Tree& tree, std::shared_ptr<Graph_Node>& new_state_node, std::shared_ptr<Graph_Node>& parent_node, std::vector<std::shared_ptr<Graph_Node>>& near_nodes){
    for (auto& near_node : near_nodes){
        if ((near_node != new_state_node) & (near_node != parent_node) & 
            (near_node->g > new_state_node->g + GetTrajectoryCost(new_state_node->config, near_node->config))){
            
            if (RRT_STAR_DEBUG_REWIRING){
                std::cout << "Rewiring ..." << std::endl;
                std::cout << "Cost near: " << near_node->g << " Cost new: " << new_state_node->g + GetTrajectoryCost(new_state_node->config, near_node->config) << std::endl;
                std::cout << "Parent ";
                print_config(new_state_node->config);
                std::cout << "Child near: ";
                print_config(near_node->config);
                std::cout << std::endl;
            }
                near_node->parent = new_state_node;
                near_node->g = new_state_node->g + GetTrajectoryCost(new_state_node->config, near_node->config);
        }
    }
}

void RRT_Star_Planner::ChooseParent(Tree& tree, 
                                    std::shared_ptr<Graph_Node>& new_state_node, 
                                    std::shared_ptr<Graph_Node>& parent_node, 
                                    std::vector<std::shared_ptr<Graph_Node>>& near_nodes){
    double current_cost;
    new_state_node->g = std::numeric_limits<double>::max();
    for (auto& near_node : near_nodes){
        current_cost = near_node->g + GetTrajectoryCost(near_node->config, new_state_node->config);//config_distance(near_node->config, new_state_node->config);
        if (current_cost < q_new_node->g){
            parent_node = near_node;
            q_new_node->g = current_cost;
        }
    }
}

std::shared_ptr<Graph_Node> RRT_Star_Planner::CreateTreeNode(Tree& tree, std::vector<double>& state){
    return tree.add_vertex(state);
}

void RRT_Star_Planner::AddToTree(Tree& tree, std::shared_ptr<Graph_Node> state_node, std::shared_ptr<Graph_Node> parent_state_node){
    tree.add_edge(state_node, parent_state_node);
}

// Change function for our project. Ask space guys, what is a valid state?
bool RRT_Star_Planner::ValidState(std::vector<double>& state){
    return true;
}

// Change distance function in kd_tree.cpp for lqr cost-to-go -> KD_Tree::_distance(const std::vector<double>& a, const std::vector<double>& b)
std::shared_ptr<Graph_Node> RRT_Star_Planner::FindNearestStateTo(Tree& tree, std::vector<double>& state){
    return tree.find_nearest_neighbor(state);
}

std::vector<std::shared_ptr<Graph_Node>> RRT_Star_Planner::FindNearestStates(Tree& tree, std::vector<double>& new_state){
    return tree.find_neighbors_within_radius(new_state, RRT_STAR_REW_RADIUS, RRT_STAR_K_NEIGHBORS);
}

// Utils Methods
void RRT_Star_Planner::SampleConfiguration(std::vector<double>& sample_state, bool& path_found){
    // Generate a random number between 0 and 1
    double p_value = static_cast<double>(rand()) / RAND_MAX;
    if (RRT_STAR_LOCAL_BIAS_ACTIVE & (p_value < RRT_STAR_LOCAL_BIASED) & path_found){
        _sample_local_biased_config(sample_state);
    }
    else{
        _sample_random_config(sample_state);
    }
}

void RRT_Star_Planner::SampleConfiguration(std::vector<double>& sample_state, std::vector<double>& goal_state, bool& path_found){
    // Generate a random number between 0 and 1
    double p_value = static_cast<double>(rand()) / RAND_MAX;
    if ((p_value < RRT_STAR_GOAL_BIASED) & !path_found){
        _sample_goal_biased_config(sample_state, goal_state);
    }
    else if (RRT_STAR_LOCAL_BIAS_ACTIVE & (p_value < RRT_STAR_LOCAL_BIASED) & path_found){
        _sample_local_biased_config(sample_state);
    }
    else{
        _sample_random_config(sample_state);
    }
}

// Utils Methods
void RRT_Star_Planner::_sample_goal_biased_config(std::vector<double>& sample_state, std::vector<double>& goal_state){
    if(RRT_STAR_DEBUG_LOCAL){
        std::cout << "GOAL sampling ..." << std::endl;
    }

    for (int i = 0; i < goal_state.size(); i++){ 
        sample_state[i] = goal_state[i] + distribution_rrt_star(gen);
    }
}

// Utils Methods
void RRT_Star_Planner::_sample_local_biased_config(std::vector<double>& sample_state){
    if(RRT_STAR_DEBUG_LOCAL){
        std::cout << "LOCAL sampling ..." << std::endl;
    }
    // Generate a random number between 0 and Tree size
    std::uniform_int_distribution<int> distribution_node(2, path.size()-1);
    int random_node_index = distribution_node(gen);
    std::vector<double> q_tmp(sample_state.size());
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
    
    for (int i=0; i<DOF; i++){
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

    for (int i=0; i < DOF; i++){
        sample_state[i] = q_node->config[i] + q_tmp[i]*distribution_local_bias(gen)/q_tmp_norm;
    }

    if(RRT_STAR_DEBUG_LOCAL){
        print_config(sample_state);
        std::cout << "Local sampling DONE!" << std::endl;
    }
}

void RRT_Star_Planner::_sample_random_config(std::vector<double>& sample_state){
    if(RRT_STAR_DEBUG_LOCAL){
        std::cout << "RANDOM sampling ..." << std::endl;
    }
    
    for (int i = 0; i < DOF; i++){ 
        sample_state[i] = (double)rand() / RAND_MAX * MAX_ANGLE;
    }
}
