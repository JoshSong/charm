#ifndef ITEM_H_
#define ITEM_H_

#include <ros/time.h>

#include "charm/shared/Globals.h"
#include "charm/shared/PointNode.h"
#include "charm/shared/Point.h"

/*
Item class represents coins/chocolate on rotating turntable

There is movement due to turntable rotation
To get position at time t use getPos(t)

*/
class Item {

public:

	// *******************************
	// PUBLIC FUNCTIONS
	// *******************************

	// Default constructor
	Item();

	// Constructor for rubbish items (no goal position)
	Item(float size, Point pos, ros::Time time);

	// Constructor for items with goal, i.e. coins
	Item(float size, Point pos, Point goal, ros::Time time);

	// Checks a point for collision, accounts for movement of
	// this item due to turntable rotation
	bool collision(Point p, ros::Time t, float margin);

	// Get position at time t (will be different to starting defined 
	// position due to turntable rotation
	Point getPos(ros::Time time);

	// *******************************
	// PUBLIC VARIABLES
	// *******************************

	float size;
	Point pos;
	bool hasGoal;
	Point goal;
	ros::Time creation_time;

private:

	// *******************************
	// PRIVATE VARIABLES
	// *******************************

	// Polar coordinates corresponding to time of creation
	float r;
	float theta;
};

#endif	// ITEM_H_
