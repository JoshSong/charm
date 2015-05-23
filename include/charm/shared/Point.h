#ifndef POINT_H_
#define POINT_H_

#define _USE_MATH_DEFINES

#include <math.h>

class Point
{

public:

	// *******************************
	// PUBLIC FUNCTIONS
	// *******************************

	// Stuff needed for use in vectors apparently
	Point();
	Point(const Point& p);	// Copy
	const Point& operator = (const Point& p);

	// Constructor
	Point(float x, float y);

	// Returns distance to other point
	float distance(Point p) const;

	// Returns distance squared, faster calculation
	float distance_squared(Point n) const;

	// Returns bearing of other point relative to right horizontal
	// in radians, guaranteed < PI, > -PI
	float angle(Point p) const;	

	// Returns 1 if > 0, -1 if < 0, 0 if == 0
	int signum(float f) const;

	// Equal if coordinates are equal
	bool equals(Point p) const;

	// *******************************
	// PUBLIC VARIABLES
	// *******************************

	float x;
	float y;


};

#endif	// POINT_H_
