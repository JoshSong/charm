
#include "stdafx.h"
#include <iostream>
#include "MainPlanner.h"
using namespace std;

// Constructor
MainPlanner::MainPlanner() {
	//arm = Arm(220.5f, 190.f);
}

// Main control loop

void MainPlanner::go() {

	// Important note: findCoins() should be called before go()
	
	// Update current arm position
	Point pos = arm.getPos();

	// Initialise vectors to hold trajectories
	trajToBins = vector<vector<PointNode>>();
	trajToCoins = vector<vector<PointNode>>();

	// Add to time offset depending on how many items were detected
	PLAN_TIME_OFFSET = 50;
	LONGLONG plan_time_per_path = 3 * items.size();
	for (int i = items.size(); i > 0; i--) {
		PLAN_TIME_OFFSET += plan_time_per_path * i;
	}

	if (PLAN_TIME_OFFSET > 11000) {
		PLAN_TIME_OFFSET = 11000;
	}

	printf("Plan time offset: %d \n", PLAN_TIME_OFFSET);
	
	PointNode start_node(pos, millisecondsNow() + PLAN_TIME_OFFSET);

	LONGLONG plan_start_time = millisecondsNow();
	LONGLONG plan_end_time = plan_start_time + PLAN_TIME_OFFSET;

	// Call calcTrajectories, which finds paths (see header for description)
	// Keep planning trajectories until all paths for getting coins to bins are found
	// (i.e. calcTrajectories returns false, or time limit for planning is exceeded
	// or we have found paths for 5 coins
	while (calcTrajectories(start_node) && millisecondsNow() < (plan_end_time - 20) ) { //&& trajToBins.size() < 5) {
		int traj_index = trajToBins.size() - 1;
		int node_index = trajToBins[traj_index].size() - 1;
		PointNode prev_node = trajToBins[traj_index][node_index];
		start_node = PointNode(prev_node.point, prev_node.time + DROP_TIME);
	}

	std::cout << "Proccessing took " << millisecondsNow() - plan_start_time << "ms (" << PLAN_TIME_OFFSET << " allowed)." << std::endl;

	// Follow the trajectories
	int index = 0;
	while (index < trajToBins.size()) {

		// Get to coin
		arm.followTrajectory(trajToCoins[index]);
		
		// Lower arm
		arm.lowerArm();
		Sleep(DROP_TIME);

		// Get to bin
		arm.followTrajectory(trajToBins[index]);
		Sleep(MOTOR_DELAY);

		// Raise arm
		arm.raiseArm();

		index++;
	}


	// Move arm to default position
	arm.resetConfig();

}

bool MainPlanner::findCoins() {

	//describe the bins
	Point bin5c(-120, -25);
	Point bin10c(-105, 50);
	Point bin20c(-50, 110);
	Point bin50c(27, 115);
	Point bin1d(100, 75);
	Point bin2d(120, -8);
	Point binForeign(-85, -95);

	// Get items from image recognition
	items = vector<Item>();
	LONGLONG time = millisecondsNow();
	int mode  = ImageProcessor::MODE_NORMAL;
	std::vector<Object> coins;

	float tableSpeed = 0;
	LONGLONG t0 = imgProc.run(mode, 40, coins, tableSpeed);
	if (t0 < 0) {
		return false;
	}
	ROT_SPEED = tableSpeed * ROT_SPEED_FACTOR;
	Point goal;
	for (int i = 0; i<coins.size(); i++){

		Point pos(coins[i].x, coins[i].y);
		switch(coins[i].type){

		case(0):
			goal = bin5c;
			break;
		case(1):
			goal = bin10c;
			break;
		case(2):
			goal = bin20c;
			break;
		case(3):
			goal = bin50c;
			break;
		case(4):
			goal = bin1d;
			break;
		case(5):
			goal = bin2d;
			break;
		// Temporary: Don't use foreign coin bin, just ignore
		//case(-1):
			//goal = binForeign;
			//break;
		default:
			goal = Point(0, 0);

		}
		printf("Coin i %d, x: %f, y: %f, type: %d, goal x: %f, goal y: %f \n", i, pos.x, pos.y, coins[i].type, goal.x, goal.y);
		if (goal.x == 0 && goal.y == 0) {
			Item rubbish(coins[i].diameter/2.f, pos, t0);
			items.push_back(rubbish);
		}
		else {
			Item input_coin(coins[i].diameter/2.f, pos, goal, t0);
			items.push_back(input_coin);
		}
	}
	return true;
}

bool MainPlanner::calcTrajectories(PointNode start_node) {

	// Create temporary holders for trajectories
	vector<vector<PointNode>> tempTrajToCoins;
	vector<vector<PointNode>> tempTrajToBins;

	// Matches trajectory index to item index
	vector<int> trajToItemIndex;

	for (int i = 0; i < items.size(); i++) {

		// Of course, we only plan trajectories for items with a goal position
		if (items[i].hasGoal == false)
			continue;

		// Calculate trajectories to each coin from current position
		// This is relatively trivial, as no obstacle detection is required
		vector<PointNode> trajToCoin = trajToItem(items[i], start_node);
		
		// Get the last node in trajToCoin
		int last_index = trajToCoin.size() - 1;
		PointNode next_node(trajToCoin[last_index].point, trajToCoin[last_index].time + DROP_TIME);

		// All other items are obstacles
		vector<Item> obstacles;
		for (int j = 0; j < items.size(); j++) {
			if (j != i)
				obstacles.push_back(items[j]);
		}

		// Calculate trajectory from coin to bin
		AStar searcher(next_node, items[i].goal, items[i].size, obstacles);
		vector<PointNode> trajToBin = searcher.search();

		// This may not always work...
		if (trajToBin.size() == 0)
			continue;

		// Record trajectory
		tempTrajToCoins.push_back(trajToCoin);
		tempTrajToBins.push_back(trajToBin);

		// Record the index corresponding to the item index
		trajToItemIndex.push_back(i);
	}

	// If we found no trajectories, assume that there are no more coins left
	if (tempTrajToBins.size() == 0)
		return false;

	// Determine the best trajectory, i.e. the fastest
	LONGLONG least_time = (tempTrajToBins[0])[tempTrajToBins.size() - 1].time;
	int best_index = 0;
	for (int k = 1; k < tempTrajToBins.size(); k++) {
		LONGLONG this_time = (tempTrajToBins[k])[tempTrajToBins.size() - 1].time;
		if (this_time < least_time) {
			least_time = this_time;
			best_index = k;
		}
	}

	// Add this best trajectory to the overall path plan
	trajToCoins.push_back(tempTrajToCoins[best_index]);
	trajToBins.push_back(tempTrajToBins[best_index]);

	printf("x: %f, y: %f \n", items[trajToItemIndex[best_index]].goal.x,  items[trajToItemIndex[best_index]].goal.y);


	// Remove coin from items list
	items.erase(items.begin() + trajToItemIndex[best_index]);

	return true;
}

// Returns trajectory to get from current arm position to an item. Only two PointNodes.
vector<PointNode> MainPlanner::trajToItem(Item item, PointNode start_node) {

	// Create new trajectory and add start_node
	vector<PointNode> trajectory;
	trajectory.push_back(start_node);

	// Calculate speed based on TIME_STEP and DIST_STEP
	float speed = DIST_STEP/TIME_STEP;

	// Calculate time to get to coin. This is a rough estimate, as the coin is actually moving
	LONGLONG time1 = item.getPos(start_node.time).distance(start_node.point) / speed;
	time1 += DROP_TIME;

	PointNode target(item.getPos(start_node.time + time1 + ADJUST_FACTOR), start_node.time + time1);
	trajectory.push_back(target);

	return trajectory;
}