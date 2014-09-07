
#pragma once

#include "Globals.h"
#include "PointNode.h"
#include "Point.h"

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
	Item(float size, Point pos, LONGLONG time);

	// Constructor for items with goal, i.e. coins
	Item(float size, Point pos, Point goal, LONGLONG time);

	// Checks a point for collision, accounts for movement of
	// this item due to turntable rotation
	bool collision(Point p, LONGLONG t, float margin);

	// Get position at time t (will be different to starting defined 
	// position due to turntable rotation
	Point getPos(LONGLONG time);

	// *******************************
	// PUBLIC VARIABLES
	// *******************************

	float size;
	Point pos;
	bool hasGoal;
	Point goal;
	LONGLONG creation_time;

private:

	// *******************************
	// PRIVATE VARIABLES
	// *******************************

	// Polar coordinates corresponding to time of creation
	float r;
	float theta;
};