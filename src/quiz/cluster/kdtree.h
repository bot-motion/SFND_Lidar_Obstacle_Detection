/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}


	Node* getNewNode(std::vector<float> point, int id)
	{
		Node* n = new Node(point, id);
		return n;
	}

	void insertNode(Node *&node, std::vector<float> point, int id, int level)
	{
		if(node == NULL)
		{
			node = getNewNode(point, id);
		}
		else 
		{
			if ((level % 2) == 0)
			{
				if (node->point[0] > point[0])
				{
					insertNode(node->left, point, id, level + 1);
				}
				else
				{
					insertNode(node->right, point, id, level + 1);
				}
			}
			else
			{
				if (node->point[1] > point[1])
				{
					insertNode(node->left, point, id, level + 1);
				}
				else
				{
					insertNode(node->right, point, id, level + 1);
				}
			}

		}
	}



	void insert(std::vector<float> point, int id)
	{
		insertNode(root, point, id, 0);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




