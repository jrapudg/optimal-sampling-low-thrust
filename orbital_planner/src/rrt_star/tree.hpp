#ifndef TREE_H
#define TREE_H

#include "basic_structs.hpp"
#include "utils.hpp"

class Tree {
	public:
		// Attributes
		std::unordered_map<int, std::shared_ptr<Graph_Node>> list;
		std::shared_ptr<Graph_Node> last_node_connected;

		// Constructors
		Tree(const std::vector<double>& init_config) {
			add_vertex(init_config);
		}

		Tree() {};

		// Methods
		std::shared_ptr<Graph_Node> add_vertex(const std::vector<double>& config);
		void add_edge(std::shared_ptr<Graph_Node> node, std::shared_ptr<Graph_Node> parent_node);
		std::shared_ptr<Graph_Node> find_nearest_neighbor(const std::vector<double>& target);
		std::vector<std::shared_ptr<Graph_Node>> find_neighbors_within_radius(const std::vector<double>& target, double radius, int k_neighbors);
		int get_current_index();

	private:
		int current_index = 0;
};

#endif // TREE_H