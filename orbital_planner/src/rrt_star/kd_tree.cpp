#include "kd_tree.hpp"


std::shared_ptr<KD_Node> KD_Tree::create_KD_Node(std::shared_ptr<Graph_Node> g_node) {
    std::shared_ptr<KD_Node> kd_node = std::make_shared<KD_Node>(g_node);
    return kd_node;
};

// Method to insert a KD_Node into the tree
void KD_Tree::insert(std::shared_ptr<KD_Node>& root, std::shared_ptr<Graph_Node> new_node, int depth) {
    int DOF = new_node->config.size();
    if (root == nullptr) {
        // Create a new node if the tree is empty
        root = create_KD_Node(new_node);
        root->axis = depth % DOF; // DoF is the number of dimensions (e.g., 2 for 2D)
        root->left = root->right = nullptr;
        return;
    }

    // Decide which axis to split based on depth
    int axis = depth % DOF;

    // Compare the current configuration's axis value to determine left or right
    if (new_node->config[axis] < (root->g_node)->config[axis]) {
        insert(root->left, new_node, depth + 1);
    } else {
        insert(root->right, new_node, depth + 1);
    }
    return;
};

// Method to calculate distance between nodes
double KD_Tree::_distance(const std::vector<double>& a, const std::vector<double>& b) {
    return config_distance(a, b);
};

// Method to find the nearest neighbor
std::shared_ptr<Graph_Node> KD_Tree::find_nearest_neighbor(std::shared_ptr<KD_Node>& root, const std::vector<double>& target, int depth) {
    if (root == nullptr) {
        //std::cout << "Nullptr found in root" << std::endl;
        return nullptr;
    }

    int axis = depth % target.size();
    std::shared_ptr<Graph_Node> best = root->g_node;
    double best_distance = _distance(target, best->config);

    if (target[axis] < root->g_node->config[axis]) {
        std::shared_ptr<Graph_Node> left_best = find_nearest_neighbor(root->left, target, depth + 1);
        if (left_best && _distance(target, left_best->config) < best_distance) {
            best = left_best;
        }
    } else {
        std::shared_ptr<Graph_Node> right_best = find_nearest_neighbor(root->right, target, depth + 1);
        if (right_best && _distance(target, right_best->config) < best_distance) {
            best = right_best;
        }
    }
    return best;
};

// Method to find the k nearest neighbors
std::vector<std::shared_ptr<Graph_Node>> KD_Tree::find_k_nearest_neighbors(std::shared_ptr<KD_Node>& root, const std::vector<double>& target, int k, double max_distance) {
    if (root == nullptr) {
        //std::cout << "Nullptr found in root" << std::endl;
        return {};
    }

    std::vector<std::shared_ptr<Graph_Node>> nearest_neighbors;
    std::priority_queue<std::shared_ptr<Graph_Node>, std::vector<std::shared_ptr<Graph_Node>>, Compare_K_Neighbors> max_heap;

    std::function<void(std::shared_ptr<KD_Node>, int)> search;
    search = [&](std::shared_ptr<KD_Node> node, int depth) {
        if (node == nullptr) {
            return;
        }

        int axis = depth % target.size();
        std::shared_ptr<Graph_Node> current = node->g_node;
        current->h = _distance(target, current->config);

        if (max_heap.size() < k) {
            max_heap.push(current);
        } else {
            if (current->h < max_heap.top()->h) {
                max_heap.pop();
                max_heap.push(current);
            }
        }

        // Check which subtree to explore first
        if (target[axis] < current->config[axis]) {
            search(node->left, depth + 1);
            if (max_heap.size() < k || max_heap.top()->h >= fabs(target[axis] - current->config[axis])) {
                search(node->right, depth + 1);
            }
        } else {
            search(node->right, depth + 1);
            if (max_heap.size() < k || max_heap.top()->h >= fabs(target[axis] - current->config[axis])) {
                search(node->left, depth + 1);
            }
        }
    };

    search(root, 0);

    while (!max_heap.empty()) {
        if (max_heap.top()->h < max_distance){
            nearest_neighbors.push_back(max_heap.top());
        }
        max_heap.pop();
    }
    return nearest_neighbors;
};

std::vector<std::shared_ptr<Graph_Node>> KD_Tree::find_neighbors_within_radius(std::shared_ptr<KD_Node>& root, const std::vector<double>& target, double alpha) {
    if (root == nullptr) {
        return {};
    }

    std::vector<std::shared_ptr<Graph_Node>> neighbors;
    std::stack<std::pair<std::shared_ptr<KD_Node>, int>> node_stack;
    node_stack.push({root, 0});

    while (!node_stack.empty()) {
        std::shared_ptr<KD_Node> current = node_stack.top().first;
        int depth = node_stack.top().second;
        node_stack.pop();

        int axis = depth % target.size();
        double distance = _distance(target, current->g_node->config);

        // If the current node is within the radius
        if (distance <= alpha) {
            neighbors.push_back(current->g_node);
        }

        // Check which subtree to explore first
        if (target[axis] < current->g_node->config[axis]) {
            if (current->left && (target[axis] - alpha) <= current->g_node->config[axis]) {
                node_stack.push({current->left, depth + 1});
            }
            if (current->right && (target[axis] + alpha) >= current->g_node->config[axis]) {
                node_stack.push({current->right, depth + 1});
            }
        } else {
            if (current->right && (target[axis] + alpha) >= current->g_node->config[axis]) {
                node_stack.push({current->right, depth + 1});
            }
            if (current->left && (target[axis] - alpha) <= current->g_node->config[axis]) {
                node_stack.push({current->left, depth + 1});
            }
        }
    }
    return neighbors;
};
