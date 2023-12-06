#ifndef TREE_H
#define TREE_H

#include <vector>
#include <unordered_map>
#include <memory>
#include <limits>


#include "orbital_planner/utils.hpp"
#include "orbital_planner/astrodynamics.hpp"
#include "orbital_planner/lqr.hpp"

struct Graph_Node {
    // Input Attributes
    int index;
    Astrodynamics::State config;

    // Methods Attributes
    double g = std::numeric_limits<double>::max();
    double h = std::numeric_limits<double>::max();
    std::shared_ptr<Graph_Node> parent;

    Optimal::MatrixS S;

	//Save the K matrix to get a free TVLQR controller
	Optimal::MatrixK K;

    // Constructor
    Graph_Node(int _i, const Astrodynamics::State& _config) : index(_i), config(_config) {
        // config is directly initialized using the initializer list
    }
};

// Initialization function for the Graph_Node struct
std::shared_ptr<Graph_Node> create_graph_node(int index, const Astrodynamics::State& config);


class Tree {
	public:
		// Attributes
		std::unordered_map<int, std::shared_ptr<Graph_Node>> list;
		std::shared_ptr<Graph_Node> last_node_connected;

		// Constructors
		Tree(const Astrodynamics::State& init_config) {
			add_vertex(init_config);
		}

		Tree() {};

		// Methods
		std::shared_ptr<Graph_Node> add_vertex(const Astrodynamics::State& config);
		void add_edge(std::shared_ptr<Graph_Node> node, std::shared_ptr<Graph_Node> parent_node, const Optimal::MatrixS& S, const Optimal::MatrixK& K);
		std::shared_ptr<Graph_Node> find_nearest_neighbor(const Astrodynamics::State& target, const Optimal::MatrixS& S);
		std::vector<std::shared_ptr<Graph_Node>> find_neighbors_within_radius(const Astrodynamics::State& target, const Optimal::MatrixS& S, double radius, int k_neighbors);
		int get_current_index();

	private:
		int current_index = 0;
};

#endif // TREE_H