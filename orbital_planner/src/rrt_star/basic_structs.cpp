#include "basic_structs.hpp"

using namespace Astrodynamics;

// Initialization function for the Graph_Node struct
std::shared_ptr<Graph_Node> create_graph_node(int index, const Astrodynamics::State& config) {
    std::shared_ptr<Graph_Node> node = std::make_shared<Graph_Node>(index, config);
    return node;
};