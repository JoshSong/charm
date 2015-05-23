#include "charm/shared/Point.h"

using namespace std;

Point::Point() : x(0), y(0) {
}

Point::Point(const Point& p) {
	Point();
	operator = (p);
}

const Point &Point::operator = (const Point& p) {
	x = p.x;
	y = p.y;
	return *this;
}

Point::Point(float arg_x, float arg_y) : x(arg_x), y(arg_y) {
}

float Point::distance(Point p) const {
	float dx = p.x - x;
	float dy = p.y - y;
	return sqrt((dx * dx) + (dy * dy));
}

float Point::distance_squared(Point p) const {
	float dx = p.x - x;
	float dy = p.y - y;
	return (dx * dx) + (dy * dy);
}

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

int Point::signum(float f) const {
	if (f > 0)
		return 1;
	if (f < 0) 
		return -1;
	return 0;
}

bool Point::equals(Point p) const {
	return x == p.x && y == p.y;
}
