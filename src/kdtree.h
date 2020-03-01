#include <vector>
#include <cmath>

struct Node
{
    std::vector<float> point; // x, y, z
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template<int Dim>
struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{
        static_assert((Dim > 0) && (Dim <= 3), "Kd tree dimension is not within valid range");
    }

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
            const int cd{ depth % Dim }; // to check if even or odd

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

    float calcEuclideanDist(const std::vector<float> &delta)
    {
        float sumSqr = 0.0;

        for (size_t i = 0; i < delta.size(); ++i)
        {
            sumSqr += delta[i] * delta[i];
        }

        return std::sqrt(sumSqr);
    }

    bool isPointInsideBox(const std::vector<float> &point, const std::vector<float> &origin, float tol)
    {
        bool check = true;

        for (size_t i = 0; i < point.size(); ++i)
        {
            check &= (point[i] <= (origin[i] + tol)) && (point[i] >= (origin[i] - tol));
        }
        
        return check;
    }

    void searchHelper(
            std::vector<int> &ids,
            Node *p_node,
            int depth,
            const std::vector<float> &target,
            float distanceTol)
    {
        // terminating condition (check for the last node (point) in the tree)
        if (p_node != nullptr)
        {
            const bool isInsideBox = isPointInsideBox(p_node->point, target, distanceTol);

            // the size of the box is choosed as double of the distance tolerance
            if (isInsideBox)
            {
                std::vector<float> delta;
                for (size_t i = 0; i < Dim; i++)
                {
                    delta.push_back(p_node->point[i] - target[i]);
                }
                
                const float euclDist { calcEuclideanDist(delta) };

                if (euclDist < distanceTol)
                {
                    // the point is inside.
                    ids.push_back( p_node->id );
                }
            }

            // decide which axis to check against based on the depth level.
            const int index = depth % Dim;

            // measure the difference from the traget point.
            const float diff = p_node->point[index] - target[index];

            // if the target point is too close to the node then check also the other
            // side of the node (branch).
            if ((target[index] + distanceTol) > p_node->point[index])
            {
                searchHelper(ids, p_node->right, depth+1, target, distanceTol);
            }

            // if the target point is too close to the node then check also the other
            // side of the node (branch).
            if ((target[index] - distanceTol) < p_node->point[index])
            {
                searchHelper(ids, p_node->left, depth+1, target, distanceTol);
            }
        }
    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
        std::vector<int> ids;
        searchHelper(ids, root, 0, target, distanceTol);
		return ids;
	}

};
