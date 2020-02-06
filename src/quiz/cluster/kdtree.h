/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node
{
    std::vector<float> point; // x, y
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

    void insertHelper(Node **node, int depth, std::vector<float> point, int id)
    {
        // the recursivness will end when all points are inserted (memory allocated for them <- Node)

        if (*node == nullptr)
        {
            // if the current point is not inserted then add node for it
            *node = new Node(point, id);
        }
        else
        {
            const int cd{ depth % 2 }; // to check if even or odd

            if (point[cd] < (*node)->point[cd])
            {
                // on the left
                insertHelper(&((*node)->left), depth+1, point, id);
            }
            else
            {
                // on the right
                insertHelper(&((*node)->right), depth+1, point, id);
            }
        }
    }

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

        // the logic to insert a point starts by checking the root of the tree.
        // At first point, the algorithm will detect that the current node (root at this instance)
        // is not allocated. Therefore, it will allocate memory for it and make it point to
        // the inserted point.

        // At second point, the algo will find that the current node (root at this instance) is
        // already allocated so it will compare the inserted point x-axis to it to determine at
        // which side it should be inserted then allocate memory (right/left node) for it.

        // At third point, the algo will find that the current node (root at this instance) is
        // already allocated so it will compare the inserted point x-axis to it to determine at
        // which side it should be inserted. if the 2nd right/left node already allocated with another
        // point then increment the depth one more and check against the 2nd node the location and so on.

        insertHelper(&root, 0, point, id);
	}

    float calcEuclideanDist(float dx, float dy)
    {
        return sqrt(dx*dx + dy*dy);
    }

    void searchHelper(std::vector<int> &ids, Node **node, int depth, std::vector<float> target, float distanceTol)
    {
        // terminating condition (check for the last node (point) in the tree)
        if (*node != nullptr)
        {
            const int index = depth % 2; // decide which axis to check against based on the depth level
            const float diff = (*node)->point[index] - target[index]; // measure the difference from the traget point

            // the size of the box is choosed as double of the distance tolerance
            if (diff < distanceTol)
            {
                const float dx = (*node)->point[0] - target[0];
                const float dy = (*node)->point[1] - target[1];
                const float euclDist = calcEuclideanDist(dx, dy);

                if (euclDist < distanceTol)
                {
                    // the point is inside.
                    ids.push_back( (*node)->id );
                }
            }

            if (target[index] > (*node)->point[index])
            {
                // branch to the right side of the node
                searchHelper(ids, &(*node)->right, depth+1, target, distanceTol);

                // if the target point is too close to the node then check also the other
                // side of the node (branch).
                if (diff < distanceTol)
                {
                    searchHelper(ids, &(*node)->left, depth+1, target, distanceTol);
                }
            }
            else
            {
                // branch to the left side of the node
                searchHelper(ids, &(*node)->left, depth+1, target, distanceTol);

                // if the target point is too close to the node then check also the other
                // side of the node (branch).
                if (diff < distanceTol)
                {
                    searchHelper(ids, &(*node)->right, depth+1, target, distanceTol);
                }
            }
        }
    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
        std::vector<int> ids;
        searchHelper(ids, &root, 0, target, distanceTol);
		return ids;
	}
	

};




