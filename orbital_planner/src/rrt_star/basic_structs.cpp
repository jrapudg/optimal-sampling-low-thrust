#include "basic_structs.hpp"

// Initialization function for the Graph_Node struct
std::shared_ptr<Graph_Node> create_graph_node(int index, double* config, int DOF) {
    std::shared_ptr<Graph_Node> node = std::make_shared<Graph_Node>(index, config, DOF);
    return node;
};