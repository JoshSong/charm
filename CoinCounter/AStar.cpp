#include "stdafx.h"
#include "AStar.h"

using namespace std;

// Constructor (stationary goal)
AStar::AStar(PointNode start, Point goal, float radius, vector<Item> obstacles) {
	this->root = start;
	this->root.index = 0;
	this->goal_point = goal;
	this->radius = radius;
	this->obstacles = obstacles;
	count = 1;
	isDynamic = false;

	// Initialise grid
	for (int i = 0; i < NUM_ROWS; i++) {
		grid.push_back(vector<int>());
		for (int j = 0; j < NUM_COLS; j ++) 
			grid[i].push_back(0);
	}
}

// Constructor (moving goal) (Not used...)
AStar::AStar(PointNode start, Item goal, vector<Item> obstacles) {
	this->root = start;
	this->root.index = 0;
	this->goal_item = goal;
	this->obstacles = obstacles;
	count = 1;
	isDynamic = true;

	// Initialise grid
	for (int i = 0; i < NUM_ROWS; i++) {
		for (int j = 0; j < NUM_COLS; j ++) 
			grid[i][j] = false;
	}
}

// Run A* search
vector<PointNode> AStar::search() {
	queue.push(root);
	goal_found = false;

	while (queue.size() > 0) {

		current_index = queue.top().index;
		old_nodes[current_index] = queue.top();
		queue.pop();
		if (goal_found = processCurrentEntry()) {

			// Found the goal. Goal index is stored in current_index.
			// Now work backward to find the path
			PointNode* p_current = getPointer(current_index);
			vector<PointNode> trajectory;
			while (p_current->parent_index != -1) {
				trajectory.insert(trajectory.begin(), *p_current);
				current_index = p_current->parent_index;
				p_current = getPointer(current_index);
			}

			// Once we reach here, the start node has been found
			trajectory.insert(trajectory.begin(), *p_current);

			return trajectory;
		}
	}

	// If we reached here, search failed
	vector<PointNode> empty;
	return empty;
}

// Process current entry from search queue, returns true 
// if goal found, false otherwise
bool AStar::processCurrentEntry() {

	PointNode* p_current = getPointer(current_index);

	// Explore children.
	// First check if we can reach the goal from here
	LONGLONG next_time = p_current->time + TIME_STEP;
	if (p_current->point.distance_squared(getGoalPos(next_time)) <= DIST_STEP * DIST_STEP) {
		count += 1;
		PointNode goal_node(getGoalPos(next_time), next_time, count, current_index);
		current_index = goal_node.index;
		old_nodes[current_index] = goal_node;
		return true;
	}

	// If not, explore the current node
	explore(*p_current);

	return false;
}

// Explore more children for a PointNode
void AStar::explore(PointNode n) {

	int nodes_added = 0;
	LONGLONG next_time = n.time + TIME_STEP;

	// Check if it is possible to make children in all 9 directions
	// (45 degrees)
	for (int i = 0; i < 8; i++) {

		// Get direction
		float angle = i * M_PI /4;
		float dx = DIST_STEP * cos(angle);
		float dy = DIST_STEP * sin(angle);

		// Distance is a bit longer for diagonal
		if (i % 2 == 1) {
			dx = dx * sqrt(2.f);
			dy = dy * sqrt(2.f);
		}

		// Make new point
		Point new_point(n.point.x + dx, n.point.y + dy);

		// If valid at next_time, create PointNode
		if (isValid(new_point, next_time) == true) {
			float path_cost = costToPath(n.index) + n.point.distance_squared(new_point);
			float heuristic_cost = costToGoalHeuristic(new_point, next_time); 
			queue.push(PointNode(new_point, next_time, count++, n.index, path_cost, heuristic_cost));
			nodes_added++;
		}
	}

	return;
}

// Return true if within turn table boundary and no collision
// with obstacle
bool AStar::isValid(Point p, LONGLONG time) {

	// Check if within boundary (turn table)
	if ((p.x * p.x) + (p.y * p.y) > TABLE_RADIUS * TABLE_RADIUS) {
		
		// If outside turn table, only allowed out if near the goal position
		if (p.distance_squared(goal_point) > BIN_RADIUS * BIN_RADIUS)
			return false;
	}

	// Check collision with obstacles
	if (collision(p, time) == true) 
		return false;

	// Make sure we haven't been here before
	if (cellCheck(p) == true)
		return true;
	else
		return false;
}

// Returns false if the grid cell has already been visited
// If not, return true and grid cell is set to be visited
bool AStar::cellCheck(Point p) {
	int row = (int) ((p.y + TABLE_RADIUS) / CELL_SIZE) + BORDER_CELLS;
	int col = (int) ((p.x + TABLE_RADIUS) / CELL_SIZE) + BORDER_CELLS;
	
	if (grid[row][col] == false) {
		grid[row][col] = true;
		return true;
	}
	else
		return false;
}

// Checks point p at time t for collision with obstacles
bool AStar::collision(Point p, LONGLONG time) {

	for (auto ob = obstacles.begin(); ob != obstacles.end(); ++ob) {
		if (ob->collision(p, time, MARGIN + radius) == true)
			return true;
	}
	return false;
}

// Return cost of path to a point (distance squared)
float AStar::costToPath(int index) {
	float cost = 0;
	PointNode* p_current = getPointer(index);
	while (p_current->parent_index != -1) {
		PointNode* p_parent = getPointer(p_current->parent_index);
		cost += p_current->point.distance_squared(p_parent->point);
		current_index = p_current->parent_index;
		p_current = p_parent;
	}
	return cost;
}

// Heuristic. In this case straight line distance squared to goal.
// Note: for moving goal, Uniform Cost search is used
float AStar::costToGoalHeuristic(Point p, LONGLONG t) {
	if (isDynamic == false)
		return p.distance_squared(getGoalPos(t));
	else
		return 0;
}

// Returns the pointer to PointNode with index i
PointNode* AStar::getPointer(int i) {
	return &old_nodes[i];
}

// Returns position of goal at time t
Point AStar::getGoalPos(LONGLONG t) {
	if (isDynamic == true)
		return goal_item.getPos(t);
	else
		return goal_point;
}