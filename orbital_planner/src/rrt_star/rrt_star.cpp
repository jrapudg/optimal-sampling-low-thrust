#include "rrt_star.hpp"

// Methods
void RRT_Star_Planner::FindPath(){
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<double> q_rand(DOF);
    if (DEBUG_USR){
        std::cout << "Start sampling .. " << std::endl;
    }

    for (int i = 0; i < RRT_STAR_NUM_ITER; i++) { 
        //_sample_random_config(q_rand);
        SampleConfiguration(q_rand);

        if (path_found & RRT_STAR_NODE_REJECTION_ACTIVE){
            double cost_to_goal = config_distance(q_rand, goal_node->config);
            double cost_to_start = config_distance(q_rand, start_node->config); 
            if (cost_to_goal + cost_to_start > goal_node->g){
                //std::cout << "Sample rejected " << i << std::endl;
                continue;
            }
        }
        sample_state state = extend_with_rewiring(tree, q_rand);

        if (state == sample_state::Trapped){
            continue;
        }

        if(path_found){
            if (state != sample_state::Trapped){
                update_count++;
                if (update_count == 5){
                    compute_path(goal_node);
                    update_count = 0;
                }
            }
        }

        if (!path_found & are_configs_close(q_new_node->config, goal_config, RRT_STAR_GOAL_TOL)){
            if (_can_connect(q_new_node->config, goal_config, RRT_STAR_GOAL_CONNECT)){
                std::cout << "Goal Node Reached!" << std::endl;
                std::cout << "Num Samples: "<< i + 1 << std::endl;
                auto state = connect(goal_config);
                if (state == sample_state::Reached){
                    goal_node = q_new_node;
                    std::cout << "GOAL COST: " << goal_node->g << std::endl;
                    compute_path(goal_node);
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
            }
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto time_elapsed_milli = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        if (path_found){
            //if (time_elapsed_milli % 100 == 0){
            //	std::cout << goal_node->g << std::endl;
            //}
            if (time_elapsed_milli >= RRT_STAR_REF_MAX){
                std::cout << "TIME IS OVER! " << std::endl;
                std::cout << "Refinement DONE! " << std::endl;
                compute_path(goal_node);
                std::cout << "NEW GOAL COST: " << goal_node->g << std::endl;
                return;
            }
        }
        
        if (time_elapsed_milli >= RRT_STAR_TIME_MAX){
            std::cout << "TIME IS OVER! " << std::endl;
            return;
        }
    }
};

bool RRT_Star_Planner::_can_connect(const std::vector<double>& alpha_config, const std::vector<double>& neighbor_config, int n_points){
    std::vector<std::vector<double>> interpolated_points = 
            generate_uniform_interpolation(alpha_config, neighbor_config, n_points);

    for (auto i_pts : interpolated_points){
        if (!IsValidArmConfiguration(i_pts, map, x_size, y_size)){
            return false;
        }
    }
    return true;
};

sample_state RRT_Star_Planner::connect(std::vector<double>& config){
    sample_state state = sample_state::Advanced;
    //connect_tree_b.last_node_connected = connect_tree_b.find_nearest_neighbor(q_new);
    while(state == sample_state::Advanced){
        state = extend_to_goal(config);
        if (DEBUG_DEV){
            std::cout << "State: " << state << std::endl;
        }
    }
    return state;
};

sample_state RRT_Star_Planner::extend_to_goal(std::vector<double>& target_config){
    std::vector<double> q_new(target_config.size());
    if (DEBUG_DEV){
        std::cout << "Get q_new" << std::endl;
    }
    _get_q_new(q_new_node->config, target_config, q_new, RRT_STAR_STEP_EXTEND_GOAL);

    if (!are_configs_equal(q_new_node->config, q_new)){
        auto new_node = tree.add_vertex(q_new);
        tree.add_edge(new_node, q_new_node);
        q_new_node = new_node;
        if (are_configs_equal(target_config, q_new)){
            return sample_state::Reached;
        }
        else{
            return sample_state::Advanced;
        }
    }
    else{
        return sample_state::Trapped;
    }
};

void RRT_Star_Planner::compute_path(std::shared_ptr<Graph_Node> node){
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
            (path[i])->g = (path[i])->parent->g + config_distance((path[i])->parent->config, path[i]->config);
        }
    }    
    *plan_length = path.size();
};

sample_state RRT_Star_Planner::extend_with_rewiring(Tree& current_tree, std::vector<double>& q_rand){
    std::vector<double> q_new(q_rand.size());
    std::shared_ptr<Graph_Node> q_min_node;
    if (RRT_STAR_DEBUG_INTER){
        std::cout << "New cost" << std::endl;
    }

    auto nearest_node = current_tree.find_nearest_neighbor(q_rand);

    if (RRT_STAR_DEBUG_INTER){
        std::cout << "Got nearest neighbor" << std::endl;
        std::cout << "Nearest config ";
        print_config(nearest_node->config);

        std::cout << "Q rand config ";
        print_config(q_rand);
    }
    _get_q_new(nearest_node->config, q_rand, q_new, RRT_STAR_STEP_EXTEND);

    if (RRT_STAR_DEBUG_INTER){
        std::cout << "Got new sample q new" << std::endl;
    }

    if (!are_configs_equal(nearest_node->config, q_new)){
        double new_cost = config_distance(nearest_node->config, q_new);
        q_new_node = current_tree.add_vertex(q_new);
        q_new_node->g = new_cost;

        if (RRT_STAR_DEBUG_REWIRING){
            std::cout << "New node ";
            print_config(q_new_node->config);
        }

        q_min_node = nearest_node;
        if (RRT_STAR_DEBUG_REWIRING){
            std::cout << "Nearest node ";
            print_config(nearest_node->config);
        }

        auto near_alpha_nodes = tree.find_neighbors_within_radius(q_new, RRT_STAR_REW_RADIUS, RRT_STAR_K_NEIGHBORS);
        for (auto& near_node : near_alpha_nodes){
            if ((near_node != q_new_node) & _can_connect(near_node->config, q_new, RRT_STAR_INTER_POINTS)){
                new_cost = near_node->g + config_distance(near_node->config, q_new);
                if (new_cost < q_new_node->g){
                    q_min_node = near_node;
                    q_new_node->g = new_cost;
                }
            }
        }
        current_tree.add_edge(q_new_node, q_min_node);

        if (RRT_STAR_DEBUG_REWIRING){
            std::cout << "Min node ";
            print_config(q_min_node->config);
            std::cout << std::endl;
        }

        for (auto& near_node : near_alpha_nodes){
            if ((near_node != q_new_node) & (near_node != q_min_node) & (_can_connect(near_node->config, q_new, RRT_STAR_INTER_POINTS))
                & (near_node->g > q_new_node->g + config_distance(q_new, near_node->config))){
                
                if (RRT_STAR_DEBUG_REWIRING){
                    std::cout << "Rewiring ..." << std::endl;
                    std::cout << "Cost near: " << near_node->g << " Cost new: " << q_new_node->g + config_distance(q_new, near_node->config) << std::endl;
                    std::cout << "Parent ";
                    print_config(q_new_node->config);
                    std::cout << "Child near: ";
                    print_config(near_node->config);
                    std::cout << std::endl;
                }
                    near_node->parent = q_new_node;
                    near_node->g = q_new_node->g + config_distance(q_new, near_node->config);
            }
        }

        if (are_configs_equal(q_rand, q_new)){
            //std::cout << "Reached!" << std::endl;
            return sample_state::Reached;
        }
        else{
            //std::cout << "Advanced!" << std::endl;
            return sample_state::Advanced;
        }
    }
    else{
        //std::cout << "Trapped!" << std::endl;
        return sample_state::Trapped;
    }
};

void RRT_Star_Planner::_get_q_new(std::vector<double>& q_near, std::vector<double>& q_rand, std::vector<double>& q_new, double step_size){
    std::vector<std::vector<double>> interpolated_points = 
            generate_epsilon_interpolation(q_near, q_rand, RRT_STAR_INTER_POINTS, step_size);

    int new_index = 0;

    for (int i=0; i<RRT_STAR_INTER_POINTS; i++){
        new_index = i;
        if (!IsValidArmConfiguration(interpolated_points[i], map, x_size, y_size)){
            if (DEBUG_DEV){
                std::cout << "Invalid at " << i << std::endl;
            }
            new_index = new_index-1;
            break;
        }
    }
    for (int i=0; i<DOF; i++){
        q_new[i] = interpolated_points[new_index][i];
    }
};

// Utils Methods
void RRT_Star_Planner::SampleConfiguration(std::vector<double>& config){
    // Generate a random number between 0 and 1
    double p_value = static_cast<double>(rand()) / RAND_MAX;
    if ((p_value < RRT_STAR_GOAL_BIASED) & !path_found){
        _sample_goal_biased_config(config);
    }
    else if (RRT_STAR_LOCAL_BIAS_ACTIVE & (p_value < RRT_STAR_LOCAL_BIASED) & path_found){
        _sample_local_biased_config(config);
    }
    else{
        _sample_random_config(config);
    }
};

// Utils Methods
void RRT_Star_Planner::_sample_goal_biased_config(std::vector<double>& config){
    if(RRT_STAR_DEBUG_LOCAL){
        std::cout << "GOAL sampling ..." << std::endl;
    }

    for (int i = 0; i < DOF; i++){ 
        config[i] = goal_config[i] + distribution_rrt_star(gen);
    }
}

// Utils Methods
void RRT_Star_Planner::_sample_local_biased_config(std::vector<double>& config){
    if(RRT_STAR_DEBUG_LOCAL){
        std::cout << "LOCAL sampling ..." << std::endl;
    }
    // Generate a random number between 0 and Tree size
    std::uniform_int_distribution<int> distribution_node(2, path.size()-1);
    int random_node_index = distribution_node(gen);
    std::vector<double> q_tmp(config.size());
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
        config[i] = q_node->config[i] + q_tmp[i]*distribution_local_bias(gen)/q_tmp_norm;
    }

    if(RRT_STAR_DEBUG_LOCAL){
        print_config(config);
        std::cout << "Local sampling DONE!" << std::endl;
    }
};

void RRT_Star_Planner::_sample_random_config(std::vector<double>& config){
    if(RRT_STAR_DEBUG_LOCAL){
        std::cout << "RANDOM sampling ..." << std::endl;
    }
    
    for (int i = 0; i < DOF; i++){ 
        config[i] = (double)rand() / RAND_MAX * MAX_ANGLE;
    }
};
