#include "charm/planner/AStar.h"

using namespace std;

AStar::AStar(PointNode start, Point goal, float radius, vector<Item> obstacles):
		root_(start), goal_point_(goal), radius_(radius), obstacles_(obstacles),
		count_(1), isDynamic_(false), goal_found_(false), current_index_(0) {
	root_.index = 0;

	// Initialise grid
	for (int i = 0; i < NUM_ROWS; i++) {
		grid_.push_back(vector<int>());
		for (int j = 0; j < NUM_COLS; j ++) 
			grid_[i].push_back(0);
	}
}

AStar::AStar(PointNode start, Item goal, vector<Item> obstacles) :
		root_(start), goal_item_(goal), obstacles_(obstacles), count_(1),
		isDynamic_(true) {
	root_.index = 0;

	// Initialise grid
	for (int i = 0; i < NUM_ROWS; i++) {
		for (int j = 0; j < NUM_COLS; j ++) 
			grid_[i][j] = false;
	}
}

vector<PointNode> AStar::search() {
	queue_.push(root_);
	goal_found_ = false;

	while (queue_.size() > 0) {

		current_index_ = queue_.top().index;
		old_nodes_[current_index_] = queue_.top();
		queue_.pop();
		goal_found_ = processCurrentEntry();
		if (goal_found_) {

			// Found the goal. Goal index is stored in current_index.
			// Now work backward to find the path
			PointNode* p_current = getPointer(current_index_);
			vector<PointNode> trajectory;
			while (p_current->parent_index != -1) {
				trajectory.insert(trajectory.begin(), *p_current);
				current_index_ = p_current->parent_index;
				p_current = getPointer(current_index_);
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

bool AStar::processCurrentEntry() {

	PointNode* p_current = getPointer(current_index_);

	// Explore children.
	// First check if we can reach the goal from here
	ros::Time next_time = p_current->time + ros::Duration(TIME_STEP * 0.001);
	if (p_current->point.distance_squared(getGoalPos(next_time)) <= DIST_STEP * DIST_STEP) {
		count_ += 1;
		PointNode goal_node(getGoalPos(next_time), next_time, count_, current_index_);
		current_index_ = goal_node.index;
		old_nodes_[current_index_] = goal_node;
		return true;
	}

	// If not, explore the current node
	explore(*p_current);

	return false;
}

void AStar::explore(PointNode n) {

	int nodes_added = 0;
	ros::Time next_time = n.time + ros::Duration(TIME_STEP * 0.001);

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
			queue_.push(PointNode(new_point, next_time, count_++, n.index, path_cost, heuristic_cost));
			nodes_added++;
		}
	}

	return;
}

bool AStar::isValid(Point p, ros::Time time) {

	// Check if within boundary (turn table)
	if ((p.x * p.x) + (p.y * p.y) > TABLE_RADIUS * TABLE_RADIUS) {
		
		// If outside turn table, only allowed out if near the goal position
		if (p.distance_squared(goal_point_) > BIN_RADIUS * BIN_RADIUS)
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

bool AStar::cellCheck(Point p) {
	int row = (int) ((p.y + TABLE_RADIUS) / CELL_SIZE) + BORDER_CELLS;
	int col = (int) ((p.x + TABLE_RADIUS) / CELL_SIZE) + BORDER_CELLS;
	
	if (grid_[row][col] == false) {
		grid_[row][col] = true;
		return true;
	}
	else
		return false;
}

bool AStar::collision(Point p, ros::Time time) {
	std::vector<Item>::iterator it;
	for (it = obstacles_.begin(); it != obstacles_.end(); ++it) {
		if (it->collision(p, time, MARGIN + radius_) == true)
			return true;
	}
	return false;
}

float AStar::costToPath(int index) {
	float cost = 0;
	PointNode* p_current = getPointer(index);
	while (p_current->parent_index != -1) {
		PointNode* p_parent = getPointer(p_current->parent_index);
		cost += p_current->point.distance_squared(p_parent->point);
		current_index_ = p_current->parent_index;
		p_current = p_parent;
	}
	return cost;
}

float AStar::costToGoalHeuristic(Point p, ros::Time t) {
	if (isDynamic_ == false)
		return p.distance_squared(getGoalPos(t));
	else
		return 0;
}

PointNode* AStar::getPointer(int i) {
	return &old_nodes_[i];
}

Point AStar::getGoalPos(ros::Time t) {
	if (isDynamic_ == true)
		return goal_item_.getPos(t);
	else
		return goal_point_;
}
