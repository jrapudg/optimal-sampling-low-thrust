#ifndef BASIC_STRUCTS_H
#define BASIC_STRUCTS_H

#include <vector>
#include <unordered_map>
#include <memory>
#include <limits>

//#include "../dynamics/astrodynamics.hpp"

#include "../dynamics/pendelum.hpp"

//using namespace Astrodynamics;

using namespace Pendelum; 

struct Graph_Node {
    // Input Attributes
    int index;
    State config;

    // Methods Attributes
    double g = std::numeric_limits<double>::max();
    double h = std::numeric_limits<double>::max();
    std::shared_ptr<Graph_Node> parent;

    // Constructor
    Graph_Node(int _i, const State& _config) : index(_i), config(_config) {
        // config is directly initialized using the initializer list
    }
};

// Initialization function for the Graph_Node struct
std::shared_ptr<Graph_Node> create_graph_node(int index, const State& config);

#endif // BASIC_STRUCTS_H