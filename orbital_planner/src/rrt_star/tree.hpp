#ifndef TREE_H
#define TREE_H

#include "kd_tree.hpp"
#include "basic_structs.hpp"

class Tree{
	public:
		// Attributes
		KD_Tree kd_tree;
		int DOF;
		int current_index = 0;
		std::unordered_map<int, std::shared_ptr<Graph_Node>> list;
		std::shared_ptr<Graph_Node> last_node_connected;

		// Constructor
		Tree(double* init_config, int _DOF){
			DOF = _DOF;
			kd_tree = KD_Tree(_DOF);
			add_vertex(init_config);
		}
		//Constructor empty
		Tree(){}

		// Methods
		std::shared_ptr<Graph_Node> add_vertex(double* config);

		void add_edge(std::shared_ptr<Graph_Node> node, std::shared_ptr<Graph_Node> parent_node);

		std::shared_ptr<Graph_Node> find_nearest_neighbor(const double* target);
		
		std::vector<std::shared_ptr<Graph_Node>> find_neighbors_within_radius(const double* target, double radius, int k_neighbors);
		
		int get_current_index();
};

#endif // TREE_H