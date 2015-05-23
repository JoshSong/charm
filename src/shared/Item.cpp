#include "charm/shared/Item.h"

Item::Item() : size(0), pos(0, 0), hasGoal(false), creation_time(0), theta(0),
		r(0) {
}

Item::Item(float s, Point p, ros::Time time) :
		size(s), pos(p), hasGoal(false), creation_time(time),
		theta(ORIGIN.angle(p)), r(ORIGIN.distance(p)) {
}

Item::Item(float s, Point p, Point g, ros::Time time) :
	size(s), pos(p), goal(g), hasGoal(true), creation_time(time),
	theta(ORIGIN.angle(p)), r(ORIGIN.distance(p)) {

}

bool Item::collision(Point p, ros::Time t, float margin) {

	// Find obstacle location at time t
	Point new_point = getPos(t);

	// Check for collision
	if (new_point.distance_squared(p) < (size + margin) * (size + margin))
		return true;
	else
		return false;
}

Point Item::getPos(ros::Time t) {
	ros::Duration dur = t - creation_time;
	float new_theta = theta + (dur.toSec() * ROT_SPEED);
	Point new_point = Point(r * cos(new_theta), r * sin(new_theta));
	return new_point;
}
