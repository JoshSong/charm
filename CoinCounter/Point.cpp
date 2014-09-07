#include "stdafx.h"
#include "Point.h"

using namespace std;

// Default constructor
Point::Point() {
	x = 0;
	y = 0;
}

// Copy
Point::Point(const Point& p) {

	// allocate variables
	Point();

	// copy values
	operator = (p);
}

// Overload equals operator
const Point &Point::operator = (const Point& p) {

	// Copy stuff
	x = p.x;
	y = p.y;

	return *this;
}

// Constructor
Point::Point(float arg_x, float arg_y) {
	x = arg_x;
	y = arg_y;
}

// Returns distance to other point
float Point::distance(Point p) const {
	float dx = p.x - x;
	float dy = p.y - y;
	return sqrt((dx * dx) + (dy * dy));
}

// Returns distance squared, faster calculation
float Point::distance_squared(Point p) const {
	float dx = p.x - x;
	float dy = p.y - y;
	return (dx * dx) + (dy * dy);
}

// Returns bearing of other point relative to right horizontal
// in radians, guaranteed < PI, > -PI
float Point::angle(Point p) const {
	float dx = p.x - x;
	float dy = p.y - y;

	float heading = 0;
	if (dx == 0)
		heading = M_PI / 2 * signum(dy);
	else if (dy == 0 && dx > 0)
		heading = 0;
	else if (dy == 0 && dx < 0)
		heading = M_PI;
	else if (dx > 0 && dy > 0)
		heading = atan(dy / dx);
	else if (dx > 0 && dy < 0)
		heading = -atan(-dy / dx);
	else if (dx < 0 && dy > 0)
		heading = M_PI - atan(dy / -dx);
	else if (dx < 0 && dy < 0)
		heading = -(M_PI - atan(-dy / -dx));

	return heading;
}

// Returns 1 if > 0, -1 if < 0, 0 if == 0
int Point::signum(float f) const {
	if (f > 0)
		return 1;
	if (f < 0) 
		return -1;
	return 0;
}

// Equal to another point if coordinates are equal
bool Point::equals(Point p) const {
	return x == p.x && y == p.y;
}
