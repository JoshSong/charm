#include "charm/shared/PointNode.h"

using namespace std;

PointNode::PointNode() :
	point(0, 0), index(0), parent_index(-1), path_cost(0), heuristic_cost(0) {
}

PointNode::PointNode(const PointNode& p) {
	PointNode();
	operator = (p);
}

const PointNode& PointNode::operator = (const PointNode& p) {
	point = p.point;
	time = p.time;
	index = p.index;
	parent_index = p.parent_index;
	path_cost = p.path_cost;
	heuristic_cost = p.heuristic_cost;
	return *this;
}

PointNode::PointNode(Point p, ros::Time t, int i, int p_i, float pc, float hc) :
	point(p), time(t), index(i), parent_index(p_i), path_cost(pc),
	heuristic_cost(hc) {
}

PointNode::PointNode(float x, float y, ros::Time t, int i, int p_i, float pc,
		float hc) :
	point(x, y), time(t), index(i), parent_index(p_i), path_cost(pc),
	heuristic_cost(hc) {
}

bool PointNode::equals(PointNode* n) {
	return point.equals(n->point) && time == n->time;
}

/* StatPath: Vector containing all of the points in the stationary trajectory;
 * omega: Angular velocity of the plate. UNITS: rad/second
 * dt: Time step per sub-stroke. UNITS: milliseconds
 */


