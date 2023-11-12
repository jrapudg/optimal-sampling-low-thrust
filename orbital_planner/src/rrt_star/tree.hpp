#ifndef TREE_H
#define TREE_H

#include "kd_tree.hpp"
#include "basic_structs.hpp"

class Tree {
	public:
		// Attributes
		KD_Tree kd_tree;
		std::unordered_map<int, std::shared_ptr<Graph_Node>> list;
		std::shared_ptr<Graph_Node> last_node_connected;

		// Constructors
		Tree(const std::vector<double>& init_config) {
			kd_tree = KD_Tree();
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