
#pragma once

#define _USE_MATH_DEFINES

// Windows version
#include <windows.h>
#include <math.h>
#include <stdio.h>
#include <conio.h>
#include <vector>

#include "Vec2.h"
#include "dynamixel.h"
#include "Point.h"
#include "PointNode.h"

#pragma comment(lib, "dynamixel.lib")

#define PI	3.141592f

// Offset values
#define X_BASE				5.5
#define Y_BASE				279.0

// Arm lengths. L2 is when end point is lowered
#define L1					220.0
#define L2					180.0

// Arm physical limits
#define LOWER_LIMIT1		0.17
#define UPPER_LIMIT1		M_PI - 0.17
#define LOWER_LIMIT2		-5*M_PI/6
#define UPPER_LIMIT2		140*M_PI/180

// Control table address
enum ControlTable 
{
	// eeprom
	CT_MODELNUMBER_L = 0,
	CT_MODELNUMBER_H,
	CT_FIRMWAREVERSION,
	CT_ID,
	CT_BAUDRATE,
	CT_RETURNDELAY,
	CT_CWLIMIT_L,
	CT_CWLIMIT_H,
	CT_CCWLIMIT_L,
	CT_CCWLIMIT_H,
	CT_TEMPLIMIT = 11,
	CT_UNDERVOLTAGELIMIT,
	CT_OVERVOLTAGELIMIT,
	CT_MAXTORQUE_L,
	CT_MAXTORQUE_H,
	CT_STATUSLEVEL,
	CT_ALARMLED,
	CT_ALARMSHUTDOWN,

	// ram
	CT_TORQUEENABLE = 24,
	CT_LEDENABLE,
	CT_CWCOMPLIANCEMARGIN,
	CT_CCWCOMPLIANCEMARGIN,
	CT_CWCOMPLIANCESLOPE,
	CT_CCWCOMPLIANCESLOPE,
	CT_GOALPOSITION_L,
	CT_GOALPOSITION_H,
	CT_MOVINGSPEED_L,
	CT_MOVINGSPEED_H,
	CT_TORQUELIMIT_L,
	CT_TORQUELIMIT_H,
	CT_PRESENTPOSITION_L,
	CT_PRESENTPOSITION_H,
	CT_PRESENTSPEED_L,
	CT_PRESENTSPEED_H,
	CT_PRESENTLOAD_L,
	CT_PRESENTLOAD_H,
	CT_PRESENTVOLTAGE,
	CT_PRESENTTEMP,
	CT_REGISTERED,
	CT_MOVING = 46,
	CT_LOCK,
	CT_PUNCH_L,
	CT_PUNCH_H
};

class NewArm {

public:

	// Constructor
	NewArm();

	// Destructor
	~NewArm();

	// Start following a trajectory
	void followTrajectory(std::vector<PointNode> trajectory);

	// Dynamixel IDs, from base joint to end joint motors
	int id[3];

	// Get the position of the end effector
	Point getPos();

	// Lowers the end tip
	void lowerArm();

	// Lowers the end tip to writing height
	void lowerArmWrite();

	// Raises the end tip
	void raiseArm();

	void setEndAngle(float angle);

	// Move arm to a point
	void moveToPoint(vec2 pos);

	// Resets arm to a default position (off the turn table) 
	// and raises arm tip
	void resetConfig();

	// Moves into position for removing chocolates
	void chocMode();
	void exitChocMode();

	// Coordinates of where to go in reset position
	// Should be off the turn table, and not block camera
	int reset_x;
	int reset_y;

	// End effector servo angles in radians
	float lowered_angle;
	float lowered_angle_choc;
	float raised_angle;

	

private:

	// Inverse kinematics
	vec2 inverseKin(vec2 pos);

	// Returns coordinates of point a fraction between p1 and p2
	// 0 <= fraction <= 1
	vec2 interpolate(Point p1, Point p2, float fraction);

	// Converts joint angles to command values to send
	inline vec2 anglesToDynValues(vec2 a) {
		return vec2((240.f - a.x*180.f/M_PI) * 1023.f/300.f, (a.y*180.f/M_PI+150.f) * 1023.f/300.f);
	}

};

// Returns milliseconds to some reference point
LONGLONG millisecondsNow();