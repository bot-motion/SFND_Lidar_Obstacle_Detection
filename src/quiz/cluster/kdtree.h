/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"



struct Point
{
	std::vector<float> coordinates;
	bool processed;
	int id;

	void print()
	{
		std::cout << "ID = " << id << " processed = " << processed << ", at (";
		for (int i = 0; i < coordinates.size(); ++i)
			std::cout << coordinates[i] << ", ";
		std::cout << ")" << std::endl; 
	}
};


// Structure to represent node of kd tree
struct Node
{
	struct Point point;
	int id;
	Node* left;
	Node* right;

	Node(struct Point p, int setId)
	:	point(p), id(setId), left(NULL), right(NULL)
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


	Node* getNewNode(struct Point point, int id)
	{
		Node* n = new Node(point, id);
		return n;
	}

	void insertNode(Node *&node, struct Point point, int id, int level)
	{
		if(node == NULL)
		{
			node = getNewNode(point, id);
		}
		else 
		{
			if ((level % 2) == 0)
			{
				if (node->point.coordinates[0] > point.coordinates[0])
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
				if (node->point.coordinates[1] > point.coordinates[1])
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



	void insert(struct Point point, int id)
	{
		insertNode(root, point, id, 0);
	}


	std::vector<int> searchNode(Node *&node, struct Point target, float distanceTol)
	{
		std::vector<int> ids, idsLeft, idsRight;

		std::cout << "Current node (" << node->point.coordinates[0] << ", " << node->point.coordinates[1] << ") - target (" << target.coordinates[0] << ", " << target.coordinates[1] << ") pm " << distanceTol << std::endl;

		if ( ((target.coordinates[0]-distanceTol) < node->point.coordinates[0]) && (node->point.coordinates[0] < (target.coordinates[0]+distanceTol)) &&
		     ((target.coordinates[1]-distanceTol) < node->point.coordinates[1]) && (node->point.coordinates[1] < (target.coordinates[1]+distanceTol)) )
		{
			if (sqrt(pow(node->point.coordinates[0] - target.coordinates[0], 2) + pow(node->point.coordinates[1] - target.coordinates[1], 2)) < distanceTol)
			{
				ids.push_back(node->id);
			}
		}

		if (node->left != NULL) 
		{
			idsLeft = searchNode(node->left, target, distanceTol);
			ids.insert(ids.begin(), idsLeft.begin(), idsLeft.end());
		}
		
		if (node->right != NULL)
		{
			idsRight = searchNode(node->right, target, distanceTol);
			ids.insert(ids.begin(), idsRight.begin(), idsRight.end());
		}

		return ids;
	}


	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(struct Point target, float distanceTol)
	{
		return searchNode(root, target, distanceTol);
	}
	

};




