#ifndef BASIC_STRUCTS_H
#define BASIC_STRUCTS_H

#include <vector>
#include <unordered_map>
#include <memory>
#include <limits>

struct Graph_Node {
    // Input Attributes
    int index;
    std::vector<double> config;

    // Methods Attributes
    double g = std::numeric_limits<double>::max();
    double h = std::numeric_limits<double>::max();
    std::shared_ptr<Graph_Node> parent;
    std::vector<int> neighbors;

    // Constructor
    Graph_Node(int _i, const std::vector<double>& _config) : index(_i), config(_config) {
        // config is directly initialized using the initializer list
    }

    // No need for a custom destructor as std::vector handles its own memory
};

// Initialization function for the Graph_Node struct
std::shared_ptr<Graph_Node> create_graph_node(int index, const std::vector<double>& config);

#endif // BASIC_STRUCTS_H