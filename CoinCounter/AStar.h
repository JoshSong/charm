
#pragma once

#include <map>
#include <queue>
#include <math.h>
#include "PointNode.h"
#include "Point.h"
#include "Item.h"


class AStar
{

public:

	// Constructor (stationary goal, i.e. bin)
	// radius is the radius of the coin being moved
	AStar(PointNode start, Point goal, float radius, std::vector<Item> obstacles);

	// Constructor (moving goal, i.e. going to coin) NOT USED
	AStar(PointNode start, Item goal, std::vector<Item> obstacles);

	// Returns trajectory
	std::vector<PointNode> search();

private:

	// *******************************
	// PRIVATE FUNCTIONS
	// *******************************

	bool processCurrentEntry();	

	// Returns the pointer to PointNode with index i
	// Note: pointer becomes invaild once queue memory is reallocated
	PointNode* getPointer(int i);

	// Explore more children for a PointNode. num_nodes is the number of children
	void explore(PointNode n);

	// Check if a point p at time t is valid (within boundary, no collision)
	bool isValid(Point p, LONGLONG t);

	// Checks point p at time t for collision with obstacles
	bool collision(Point p, LONGLONG t);

	// Return cost of path to a point (distance squared)
	float costToPath(int index);

	// Heuristic. In this case straight line distance squared to goal
	float costToGoalHeuristic(Point p, LONGLONG time);

	// Returns position of goal
	Point getGoalPos(LONGLONG time);

	// Returns false if the grid cell has already been visited
	// If not, return true and grid cell is set to be visited
	bool cellCheck(Point p);

	// *******************************
	// PRIVATE VARIABLES
	// *******************************

	PointNode root;
	Point goal_point;
	Item goal_item;
	float radius;	// Radius of coin being moved in mm
	int count;	// For indexing nodes
	std::vector<Item> obstacles;
	bool isDynamic;	// True for moving goal, i.e. chasing a coin.

	// Grid to prevent generating nodes in the same place
	std::vector< std::vector<int> > grid;

	// The nodes to be searched. 
	std::priority_queue<PointNode> queue;

	// Index of the queue entry currently being processed.
	int current_index;

	// True if a goal has been found, and false otherwise.
	bool goal_found;

	// Old nodes which have been explored/being currently explored
	// Key is index
	std::map<int, PointNode> old_nodes;

};