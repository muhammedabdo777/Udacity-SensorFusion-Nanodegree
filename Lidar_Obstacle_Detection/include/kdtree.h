#ifndef KDTREE_H
#define KDTREE_H

#include "render.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL) {}
};

struct KdTree
{
	Node *root;

	KdTree() : root(NULL) {}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the
		// root root must be passed by ref to save the change so recursiveInsert
		// take double pointer.
		recursiveInsert(&root, 0, point, id);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		recursiveSearch(root, 0, target, ids, distanceTol);
		return ids;
	}

private:
	void recursiveInsert(Node **node, int depth, std::vector<float> point, int id)
	{
		if (*node == nullptr)
		{ // create node in heap when it is empty
			*node = new Node(point, id);
		}
		else
		{
			auto cd = depth % 3; // Use 3 for 3D

			if (point[cd] < (*node)->point[cd])
			{
				// recursive call with left
				recursiveInsert(&((*node)->left), depth + 1, point, id);
			}
			else
			{
				// recursive call with right
				recursiveInsert(&((*node)->right), depth + 1, point, id);
			}
		}
	}
	/**
	 * @brief Check if the node is within the target box with distance tolerance
	 *
	 * @param node
	 * @param target
	 * @param distanceTol
	 * @return true
	 * @return false
	 */
	bool isWithinTargetBox(Node *node, std::vector<float> target,
						   float distanceTol)
	{
		if (((node->point[0] >= target[0] - distanceTol) &&
			 (node->point[0] <=
			  target[0] + distanceTol)) && // x axis right and left tolerance
			((node->point[1] >= target[1] - distanceTol) &&
			 (node->point[1] <=
			  target[1] + distanceTol)) && // y axis right and left tolerance.
			((node->point[2] >= target[2] - distanceTol) &&
			 (node->point[2] <=
			  target[2] + distanceTol))) // z axis right and left tolerance.
		{
			auto x = (node->point[0] - target[0]);
			auto y = (node->point[1] - target[1]);
			auto z = (node->point[2] - target[2]);
			auto distance = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));

			if (distance <= distanceTol)
			{
				return true;
			}
		}
		return false;
	}

	/**
	 * @brief Recursive search for the target point within distance tolerance
	 *
	 * @param node
	 * @param depth
	 * @param target
	 * @param ids
	 * @param distanceTol
	 */
	void recursiveSearch(Node *node, int depth, std::vector<float> target,
						 std::vector<int> &ids, float distanceTol)
	{
		if (node != NULL)
		{
			if (isWithinTargetBox(node, target, distanceTol))
			{
				ids.push_back(node->id);
				// std::cout << "id " << node->id << std::endl;
			}

			// go left or right recursive search
			auto cd = depth % 3;
			if ((target[cd] - distanceTol) < (node->point[cd]))
			{
				recursiveSearch(node->left, depth + 1, target, ids,
								distanceTol);
				// std::cout << "left with node id " << node->id << std::endl;
			}

			if ((target[cd] + distanceTol) > (node->point[cd]))
			{
				recursiveSearch(node->right, depth + 1, target, ids,
								distanceTol);
				// std::cout << "right with node id " << node->id << std::endl;
			}
		}
	}
};

#endif // KDTREE_H
