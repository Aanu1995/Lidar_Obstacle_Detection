
// Structure to represent node of kd tree
struct Node {
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId):	point(arr), id(setId), left(nullptr), right(nullptr) {}

	~Node() {
		delete left;
		delete right;
	}
};

struct KdTree {
	Node* root;

	KdTree(): root(nullptr) {}

	~KdTree(){
		delete root;
	}

	void insert(const std::vector<float> point, int id) {
		// This function insert a new 3D point into the tree
		// the function should create a new node and place correctly with in the root
		insertHelper(&root, point, id, 0);
	}

	void insertHelper(Node** node, std::vector<float> point, int id, uint depth) {
		if (*node == nullptr) {
			*node = new Node(point, id);
		} else {
			// calculate current dimension (cd) for 3D points
			uint cd = depth % 3; // 0 for x-axis, 1 for y-axis, 2 for z-axis

			if (point[cd] < (*node)->point[cd]) {
				insertHelper(&((*node)->left), point, id, depth + 1);
			} else {
				insertHelper(&((*node)->right), point, id, depth + 1);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol) {
		std::vector<int> ids {};

		searchHelper(root, target, distanceTol, ids, 0);

		return ids;
	}

	void searchHelper(Node* node, std::vector<float> target, float distanceTol, std::vector<int>& ids, uint depth) {
		if (node != nullptr) {
			// check if current node is within distanceTol (3D bounding box)
			bool withinBoundingBox = (node->point[0] >= target[0] - distanceTol &&
									 node->point[0] <= target[0] + distanceTol) &&
									(node->point[1] >= target[1] - distanceTol &&
									 node->point[1] <= target[1] + distanceTol) &&
									(node->point[2] >= target[2] - distanceTol &&
									 node->point[2] <= target[2] + distanceTol);

			if (withinBoundingBox) {
				// calculate 3D Euclidean distance
				float dx = node->point[0] - target[0];
				float dy = node->point[1] - target[1];
				float dz = node->point[2] - target[2];
				float distance = sqrt(dx*dx + dy*dy + dz*dz);

				if (distance <= distanceTol) {
					ids.push_back(node->id);
				}
			}

			// check which side of the tree to search (3D)
			uint cd = depth % 3; // 0 for x-axis, 1 for y-axis, 2 for z-axis
			if ((target[cd] - distanceTol) < node->point[cd]) {
				searchHelper(node->left, target, distanceTol, ids, depth + 1);
			}
			if ((target[cd] + distanceTol) > node->point[cd]) {
				searchHelper(node->right, target, distanceTol, ids, depth + 1);
			}
		}
	}

};




