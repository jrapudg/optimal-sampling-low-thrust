#ifndef BASIC_STRUCTS_H
#define BASIC_STRUCTS_H

#include <vector>
#include <unordered_map>
#include <memory>
#include <limits>

struct Graph_Node{
	// Input Attributes
	int index;
	double* config;

	// Methods Attributes
	double g = std::numeric_limits<double>::max();
	double h = std::numeric_limits<double>::max();
	std::shared_ptr<Graph_Node> parent;
	std::vector<int> neighbors;

	// Constructor
    Graph_Node(int _i, const double* _config, int DOF) : index(_i) {
        config = new double[DOF];
        for (int i = 0; i < DOF; ++i) {
            config[i] = _config[i];
        }
    }
    // Destructor to release memory
    ~Graph_Node() {
        delete[] config;
    }
};

// Initialization function for the Graph_Node struct
std::shared_ptr<Graph_Node> create_graph_node(int index, double* config, int DOF);

#endif // BASIC_STRUCTS_H