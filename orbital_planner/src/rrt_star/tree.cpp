#include "tree.hpp"

// Methods
std::shared_ptr<Graph_Node> Tree::add_vertex(double* config){
    auto node = create_graph_node(current_index, config, DOF);
    list[current_index] = node;
    kd_tree.insert(kd_tree.root, node, 0);
    current_index ++;
    return node;
}

void Tree::add_edge(std::shared_ptr<Graph_Node> node, std::shared_ptr<Graph_Node> parent_node){
    node->parent = parent_node;
    node->g = parent_node->g + config_distance(node->config, parent_node->config, DOF);
}

std::shared_ptr<Graph_Node> Tree::find_nearest_neighbor(const double* target){
    return kd_tree.find_nearest_neighbor(kd_tree.root, target, 0);
};

std::vector<std::shared_ptr<Graph_Node>> Tree::find_neighbors_within_radius(const double* target, double radius, int k_neighbors){
    return kd_tree.find_k_nearest_neighbors(kd_tree.root, target, k_neighbors, radius);
};

int Tree::get_current_index(){
    return current_index;
};
