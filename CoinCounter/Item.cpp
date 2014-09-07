

#include "stdafx.h"
#include "Item.h"

// Default constructor
Item::Item() {
	size = 0;
	pos= Point(0, 0);
	hasGoal = false;
	creation_time = 0;

	theta = 0;
	r = 0;
}

// Constructor for rubbish items (no goal position)
Item::Item(float s, Point p, LONGLONG time) {
	size = s;
	pos= p;
	hasGoal = false;
	creation_time = time;

	theta = ORIGIN.angle(p);
	r = ORIGIN.distance(p);
}

// Constructor for items with goal, i.e. coins
Item::Item(float s, Point p, Point g, LONGLONG time) {
	size = s;
	pos = p;
	goal = g;
	hasGoal = true;
	creation_time = time;

	theta = ORIGIN.angle(p);
	r = ORIGIN.distance(p);
}

// Checks a point for collision, accounts for movement of
// this item due to turntable rotation
bool Item::collision(Point p, LONGLONG t, float margin) {

	// Find obstacle location at time t
	Point new_point = getPos(t);

	// Check for collision
	if (new_point.distance_squared(p) < (size + margin) * (size + margin))
		return true;
	else
		return false;
}

// Get position at time t (will be different to starting defined 
// position due to turntable rotation
Point Item::getPos(LONGLONG t) {
	
	// Get change in time in seconds
	float dt = (float) (t - creation_time) * 0.001;

	float new_theta = theta + (dt * ROT_SPEED);
	Point new_point = Point(r * cos(new_theta), r * sin(new_theta));
	return new_point;
}