#include "stdafx.h"
#include "PointNode.h"

using namespace std;

// Default constructor
PointNode::PointNode() {
	point = Point(0, 0);
	time = 0;
	index = 0;
	parent_index = -1;
	path_cost = 0;
	heuristic_cost = 0;
}

// Copy
PointNode::PointNode(const PointNode& p) {

	// allocate variables
	PointNode();

	// copy values
	operator = (p);
}

// Equals operator overload
const PointNode& PointNode::operator = (const PointNode& p) {

	// Copy stuff
	point = p.point;
	time = p.time;
	index = p.index;
	parent_index = p.parent_index;
	path_cost = p.path_cost;
	heuristic_cost = p.heuristic_cost;
	
	return *this;
}

// Constructor using Point argument
PointNode::PointNode(Point p, LONGLONG t, int i, int p_i, float pc, float hc) {
	point = p;
	time = t;
	index = i;
	parent_index = p_i;
	path_cost = pc;
	heuristic_cost = hc;
}

// Constructor using float arguments
PointNode::PointNode(float x, float y, LONGLONG t, int i, int p_i, float pc, float hc) {
	point = Point(x, y);
	time = t;
	index = i;
	parent_index = p_i;
	path_cost = pc;
	heuristic_cost = hc;
}

// Equal to another node if coordinates are equal & time is equal
bool PointNode::equals(PointNode* n) {
	return point.equals(n->point) && time == n->time;
}

/* StatPath: Vector containing all of the points in the stationary trajectory;
 * omega: Angular velocity of the plate. UNITS: rad/second
 * dt: Time step per sub-stroke. UNITS: milliseconds
 */


