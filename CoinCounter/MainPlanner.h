

#pragma once

#include <vector>
#include <Windows.h>


#include "Item.h"
#include "Globals.h"

#include "NewArm.h"
#include "AStar.h"
#include "ImageProcessor.h"
#include "Writer.h"

class MainPlanner {

public:

	// *******************************
	// PUBLIC FUNCTIONS
	// *******************************

	// Constructor
	MainPlanner();

	// Find coins using image processing
	void findCoins();

	// Start the control loop
	// Ends once all coins are put in the bin (ideally)
	void go();


	// *******************************
	// PUBLIC VARIABLES
	// *******************************

	// Stores item data, including coins and chocolate
	std::vector<Item> items;

	// Handles kinematics and sending commands to Dynamixels
	NewArm arm;

	// Handles image processing
	ImageProcessor imgProc;


private:

	// *******************************
	// PRIVATE FUNCTIONS
	// *******************************

	// Calculates trajectories from current position to coin to bin for each coin
	// start_node must cointain time to start planning from and arm position at that time
	// returns true if a trajectory was found (and added to trajToCoins, trajToBins)
	// returns false otherwise (no more coins, we're done)
	bool calcTrajectories(PointNode start_node);

	// Returns trajectory to get from current arm position to an item. Only two PointNodes.
	std::vector<PointNode> MainPlanner::trajToItem(Item item, PointNode start_node);


	// *******************************
	// PRIVATE VARIABLES
	// *******************************

	// Current end effector position
	Point pos;

	// Stores trajectories
	std::vector<std::vector<PointNode>> trajToCoins;
	std::vector<std::vector<PointNode>> trajToBins;

	std::vector<std::vector<std::vector<PointNode> > > genDigits(float scale);
	std::vector<PointNode> genSeg(std::vector<PointNode> points, int n);
	void writeValue(std::vector<std::vector<int> > digits, float omega);
	std::vector<PointNode> get_rotated(std::vector<PointNode> statPath,
		float omega, int dt, LONGLONG t0, int offsetX, int offsetY);

};