
#pragma once

#include <map>
#include <vector>
#include <Windows.h>

#include "Point.h"
#include "Globals.h"

/*
The PointNode class handles position in three dimensions: x, y, time

There is also some implementation of "parent", "cost", etc. 
This is for use in search algorithms e.g. A*

Author: Joshua Song 22/10/2013
*/
class PointNode
{

public:

	// *******************************
	// PUBLIC FUNCTIONS
	// *******************************

	PointNode();
	PointNode(const PointNode& n);	// Copy constructor
	const PointNode &PointNode::operator = (const PointNode& n); // = operator overload

	// Constructor
	PointNode(Point p, LONGLONG time, int index = -1, int parent_index = -1, float path_cost = 0, float heuristic_cost = 0);
	PointNode(float x, float y, LONGLONG time, int index = -1, int parent_index = -1, float path_cost = 0, float heuristic_cost = 0);

	// Comparator based on cost
	inline bool operator<(const PointNode& n) const {
		return path_cost + heuristic_cost > n.path_cost + n.heuristic_cost;
	}

	// Equal if points are equal, and time is equal
	bool equals(PointNode* n);

	// *******************************
	// PUBLIC VARIABLES
	// *******************************

	Point point;	// Point at time of creation of this
	LONGLONG time;	// milliseconds

	int index;
	int parent_index;	// Index of parent PointNode. -1 if no parent

	// Not very conventional, but cost to get from start node to 
	// here is stored in this variable
	float path_cost;
	float heuristic_cost;

};