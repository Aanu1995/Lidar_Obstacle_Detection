// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node {
	std::array<float, 3> point;
	int id;
	Node* left;
	Node* right;

	Node(std::array<float, 3> arr, int setId):	point(arr), id(setId), left(NULL), right(NULL) {}

	~Node() {
		delete left;
		delete right;
	}
};

struct KdTree {
	Node* root;

	KdTree(): root(NULL) {}

	~KdTree(){
		delete root;
	}

	void insert(std::array<float, 3> point, int id) {
		// This function insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertHelper(&root, point, id, 0);
	}

	void insertHelper(Node** node, std::array<float, 3> point, int id, uint depth) {
		if (*node == NULL) {
			*node = new Node(point, id);
		} else {
			// calculate current dimension (cd)
			uint cd = depth % 2; // 0 for x-axis, 1 for y-axis
			uint newDepth = depth + 1;

			if (point[cd] < (*node)->point[cd]) {
				insertHelper(&((*node)->left), point, id, newDepth);
			} else {
				insertHelper(&((*node)->right), point, id, newDepth);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::array<float, 3> target, float distanceTol) {
		std::vector<int> ids {};

		searchHelper(root, target, distanceTol, ids, 0);

		return ids;
	}

	void searchHelper(Node* node, std::array<float, 3> target, float distanceTol, std::vector<int>& ids, uint depth) {
		if (node != NULL) {
			// check if current node is within distanceTol
			if (node->point[0] >= target[0] - distanceTol && node->point[0] <= target[0] + distanceTol &&
				node->point[1] >= target[1] - distanceTol && node->point[1] <= target[1] + distanceTol) {
				float d = sqrt(pow(node->point[0] - target[0], 2) + pow(node->point[1] - target[1], 2));
				if (d <= distanceTol) {
					ids.push_back(node->id);
				}
			}

			// check which side of the tree to search
			uint cd = depth % 2; // 0 for x-axis, 1 for y-axis
			if ((target[cd] - distanceTol) < node->point[cd]) {
				searchHelper(node->left, target, distanceTol, ids, depth + 1);
			}
			if ((target[cd] + distanceTol) > node->point[cd]) {
				searchHelper(node->right, target, distanceTol, ids, depth + 1);
			}
		}
	}

};




