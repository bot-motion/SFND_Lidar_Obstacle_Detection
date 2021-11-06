#ifndef TREE_H_
#define TREE_H_

#include <vector>
#include <iostream>
#include <math.h>

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
			int dim = level % point.coordinates.size();
			if (node->point.coordinates[dim] > point.coordinates[dim])
			{
				insertNode(node->left, point, id, level + 1);
			}
			else
			{
				insertNode(node->right, point, id, level + 1);
			}
		}
	}





	void insert(struct Point point, int id)
	{
		insertNode(root, point, id, 0);
	}


	std::vector<int> searchNode(Node *&node, struct Point target, float distanceTol, int level)
	{
		std::vector<int> ids, idsLeft, idsRight;

		if ( ((target.coordinates[0]-distanceTol) < node->point.coordinates[0]) && (node->point.coordinates[0] < (target.coordinates[0]+distanceTol)) &&
		     ((target.coordinates[1]-distanceTol) < node->point.coordinates[1]) && (node->point.coordinates[1] < (target.coordinates[1]+distanceTol)) )
		{
			if (sqrt(pow(node->point.coordinates[0] - target.coordinates[0], 2) + pow(node->point.coordinates[1] - target.coordinates[1], 2)) < distanceTol)
			{
				ids.push_back(node->id);
			}
		}

		int dim = level % target.coordinates.size();

		if (target.coordinates[dim] - distanceTol < node->point.coordinates[dim]) 
		{
			idsLeft = searchNode(node->left, target, distanceTol, level+1);
			ids.insert(ids.begin(), idsLeft.begin(), idsLeft.end());
		}
		
		if (target.coordinates[dim] + distanceTol > node->point.coordinates[dim])
		{
			idsRight = searchNode(node->right, target, distanceTol, level+1);
			ids.insert(ids.begin(), idsRight.begin(), idsRight.end());
		}

		return ids;
	}


	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(struct Point target, float distanceTol)
	{
		return searchNode(root, target, distanceTol, 0);
	}
	

};




#endif