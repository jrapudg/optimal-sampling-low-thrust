#include "tree.hpp"

using namespace Astrodynamics;
using namespace Optimal;

// Methods
std::shared_ptr<Graph_Node> Tree::add_vertex(const State& config){
    std::shared_ptr<Graph_Node> node = create_graph_node(current_index, config);
    list[current_index] = node;
    current_index ++;
    return node;
}

void Tree::add_edge(std::shared_ptr<Graph_Node> node, std::shared_ptr<Graph_Node> parent_node){
    node->parent = parent_node;
    //node->g = parent_node->g + config_distance(node->config, parent_node->config);
}

std::shared_ptr<Graph_Node> Tree::find_nearest_neighbor(const State& target, const MatrixS& S){
    int nearest_neighbor = 0;
    double best_distance = config_distance(list[0]->config, target, S);
    for(const auto& value : list)
    {
        double neighbor_dist = config_distance(target, value.second->config, S);
        if(neighbor_dist < best_distance)
        {
            best_distance = neighbor_dist;
            nearest_neighbor = value.first;
        }
    }
    return list[nearest_neighbor];
};

struct CompareNodeDist {
    bool operator()(const std::pair<std::shared_ptr<Graph_Node>, double>& a, const std::pair<std::shared_ptr<Graph_Node>, double>& b) {
        // sort based off of min distance
        return a.second < b.second;
    }
};

std::vector<std::shared_ptr<Graph_Node>> Tree::find_neighbors_within_radius(const State& target, const MatrixS& S, double radius, int k_neighbors){
    std::priority_queue<std::pair<std::shared_ptr<Graph_Node>, double>, std::vector<std::pair<std::shared_ptr<Graph_Node>, double>>, CompareNodeDist> node_dist_queue;
    for(auto& value : list)
    {
        double neighbor_dist = config_distance(target, value.second->config, S);
        if(neighbor_dist < radius)
        {
            node_dist_queue.push(std::make_pair(value.second, neighbor_dist));
        }
    }

    std::vector<std::shared_ptr<Graph_Node>> nearest_neighbors;
    for(int idx = 0; idx < k_neighbors; ++idx)
    {
        //check to see if node dist queue is empty or not
        if(!node_dist_queue.empty())
        {
            std::pair<std::shared_ptr<Graph_Node>, double> radius_neighbor = node_dist_queue.top();
            node_dist_queue.pop();

            nearest_neighbors.push_back(radius_neighbor.first);
        }
    }
    return nearest_neighbors;
};

int Tree::get_current_index(){
    return current_index;
};