#ifndef TREE_H_
#define TREE_H_



struct Point
{
	std::vector<float> coordinates;
	bool processed;
	int id;

	void print();
};


// Structure to represent node of kd tree
struct Node
{
	struct Point point;
	int id;
	Node* left;
	Node* right;

	Node(struct Point p, int setId);

	~Node();
};




struct KdTree
{
	Node* root;

	KdTree();

	~KdTree();


	Node* getNewNode(struct Point point, int id);

	void insertNode(Node *&node, struct Point point, int id, int level);


	void insert(struct Point point, int id);


	std::vector<int> searchNode(Node *&node, struct Point target, float distanceTol);

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(struct Point target, float distanceTol);	

};



#endif