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
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}



	void recursiveInsert(Node** node, uint depth, std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		if(*node == NULL)
		{
			*node = new Node(point,id);
		}
		else
		{
			uint cd = depth%2; //0 x, 1 y

			if(point[cd] < (((*node)->point[cd])))
			{
				recursiveInsert(&((*node)->left),depth+1,point,id);
			}
			else
			{
				recursiveInsert(&((*node)->right),depth+1,point,id);
			}

		}


	}


	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		recursiveInsert(&root,0,point,id);


	}



	void recursiveSearch(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if(node != NULL)
		{

			if((node->point[0]>=(target[0]-distanceTol) && node->point[0]<=(target[0]+distanceTol)) && 
				(node->point[1]>=(target[1]-distanceTol) && node->point[1]<=(target[1]+distanceTol))){

					float distance = sqrt(pow((node->point[0]-target[0]),2)+ pow((node->point[1]-target[1]),2));
					if (distance<=distanceTol)
						ids.push_back(node->id);
				}
				
			if((target[depth%2]-distanceTol)<node->point[depth%2])
				recursiveSearch(target,node->left,depth+1,distanceTol,ids);
			if((target[depth%2]+distanceTol)>node->point[depth%2])
				recursiveSearch(target,node->right,depth+1,distanceTol,ids);
		}


	}


	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		recursiveSearch(target,root,0,distanceTol,ids);
		return ids;
	}
	

};




