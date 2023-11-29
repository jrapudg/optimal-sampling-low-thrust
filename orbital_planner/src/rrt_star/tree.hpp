#ifndef TREE_H
#define TREE_H

#include "basic_structs.hpp"
#include "utils.hpp"
#include "../dynamics/astrodynamics.hpp"
#include "../dynamics/lqr.hpp"

using namespace Astrodynamics;
using namespace Optimal;

class Tree {
	public:
		// Attributes
		std::unordered_map<int, std::shared_ptr<Graph_Node>> list;
		std::shared_ptr<Graph_Node> last_node_connected;

		// Constructors
		Tree(const State& init_config) {
			add_vertex(init_config);
		}

		Tree() {};

		// Methods
		std::shared_ptr<Graph_Node> add_vertex(const State& config);
		void add_edge(std::shared_ptr<Graph_Node> node, std::shared_ptr<Graph_Node> parent_node);
		std::shared_ptr<Graph_Node> find_nearest_neighbor(const State& target, const MatrixS& S);
		std::vector<std::shared_ptr<Graph_Node>> find_neighbors_within_radius(const State& target, const MatrixS& S, double radius, int k_neighbors);
		int get_current_index();

	private:
		int current_index = 0;
};

#endif // TREE_H