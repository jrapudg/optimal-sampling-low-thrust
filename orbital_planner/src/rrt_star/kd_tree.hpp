#ifndef KD_TREE_H
#define KD_TREE_H

#include "basic_structs.hpp"
#include "utils.hpp"

struct Compare_K_Neighbors {
	bool operator()(const std::shared_ptr<Graph_Node>& a, const std::shared_ptr<Graph_Node>& b) {
		// Compare distances in descending order (max-heap)
		return a->h < b->h;
	}
};

// Define a struct KD_Node
struct KD_Node {
	// Input Attributes
    std::shared_ptr<Graph_Node> g_node; // The configuration (or point) associated with this node    

	// Methods Attributes
	int axis;
	std::shared_ptr<KD_Node> left;  // Pointer to the left subtree
    std::shared_ptr<KD_Node> right; // Pointer to the right subtree

	// Constructor
	KD_Node(std::shared_ptr<Graph_Node> _g_node): g_node(_g_node){}
};

class KD_Tree{
	public:
		// Tree attributes
		std::shared_ptr<KD_Node> root = nullptr;

		// Constructor empty
   		KD_Tree() {};

		// Methods
		// Method to create a KD_Node with state
		std::shared_ptr<KD_Node> create_KD_Node(std::shared_ptr<Graph_Node> g_node);
		// Method to insert a KD_Node into the tree
		void insert(std::shared_ptr<KD_Node>& root, std::shared_ptr<Graph_Node> new_node, int depth);
		// Method to calculate distance between nodes
		double _distance(const std::vector<double>& a, const std::vector<double>& b);
		// Method to find the nearest neighbor
		std::shared_ptr<Graph_Node> find_nearest_neighbor(std::shared_ptr<KD_Node>& root, const std::vector<double>& target, int depth);
		// Method to find the k nearest neighbors
		std::vector<std::shared_ptr<Graph_Node>> find_k_nearest_neighbors(std::shared_ptr<KD_Node>& root, const std::vector<double>& target, int k, double max_distance);
        // Method to find nearest neighbors within a radius
		std::vector<std::shared_ptr<Graph_Node>> find_neighbors_within_radius(std::shared_ptr<KD_Node>& root, const std::vector<double>& target, double alpha);
};

#endif // KD_TREE_H