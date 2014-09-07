
#include "stdafx.h"
#include "NewArm.h"

using namespace std;

NewArm::NewArm() 
{

	// Set Dynamixel IDs
	// In order of joints from base to tip
	id[0] = 1;
	id[1] = 2;
	id[2] = 3;

	int baudnum = 1;

	///////// Open USB2Dynamixel ////////////
	if( dxl_initialize() == 0 )
	{
		std::printf( "Failed to open USB2Dynamixel!\n" );
		_getch();
	}
	std::printf( "Succeeded to open USB2Dynamixel!\n" );

	dxl_set_baud( baudnum );

	// Set speed and compliance slope to max
	dxl_write_byte(BROADCAST_ID, CT_CWCOMPLIANCESLOPE, 128);
	dxl_write_byte(BROADCAST_ID, CT_CCWCOMPLIANCESLOPE, 128);
	dxl_write_word(BROADCAST_ID, CT_MOVINGSPEED_L, 0);

	// Define reset position
	reset_x = 155;
	reset_y = 0;

	// Define end effector servo angles in radians
	lowered_angle = 0.98;
	raised_angle = 0;

	// Reset position
	resetConfig();

	// Debugging: turn off torque
	//dxl_write_word(BROADCAST_ID, CT_TORQUEENABLE, 0);

	Sleep(2000);
}

// Resets arm to a default position (off the turn table) 
// and raises arm tip
void NewArm::resetConfig() {
	moveToPoint(vec2(reset_x, reset_y));
	Sleep(300);
	raiseArm();
}

// Lowers the end tip
void NewArm::lowerArm() {
	setEndAngle(lowered_angle);
}

// Lowers the end tip for writing mode
void NewArm::lowerArmWrite() {
	dxl_write_word(id[2], CT_GOALPOSITION_L, 358);
}

// Raises the end tip
void NewArm::raiseArm() {
	setEndAngle(raised_angle);
}

// Change the end effector angle
void NewArm::setEndAngle(float angle) {
	int pos = (-angle+5*M_PI/6)*180*1023/(M_PI*300);
	dxl_write_word(id[2], CT_GOALPOSITION_L,pos);
}

// Move end effector to a point
void NewArm::moveToPoint(vec2 pos) {

	// Inverse kinematics
	vec2 angles = inverseKin(pos);

	// Ensure angles within limits
	
	if (angles.x < LOWER_LIMIT1) angles.x = LOWER_LIMIT1;
	else if (angles.x > UPPER_LIMIT1) angles.x = UPPER_LIMIT1;
	if (angles.y < LOWER_LIMIT2) angles.y = LOWER_LIMIT2;
	else if (angles.y > UPPER_LIMIT2) angles.y = UPPER_LIMIT2;
	
	vec2 dyn_vals = anglesToDynValues(angles);

	// Make syncwrite packet
	int GoalPos;
	dxl_set_txpacket_id(BROADCAST_ID);
	dxl_set_txpacket_instruction(INST_SYNC_WRITE);
	dxl_set_txpacket_parameter(0, CT_GOALPOSITION_L);
	dxl_set_txpacket_parameter(1, 2);
	int num_actuator = 2;
	for( int i = 0; i < num_actuator; i++ )
	{
		dxl_set_txpacket_parameter(2+3*i, id[i]);
		if (i == 0)
			GoalPos = dyn_vals.x;
		else
			GoalPos = dyn_vals.y;
		std::printf( "[%d]:%d ", id[i], GoalPos );
		dxl_set_txpacket_parameter(2+3*i+1, dxl_get_lowbyte(GoalPos));
		dxl_set_txpacket_parameter(2+3*i+2, dxl_get_highbyte(GoalPos));
	}
	std::printf( "\r" );
	dxl_set_txpacket_length((2+1)*num_actuator+4);
	dxl_txrx_packet();

	Sleep(10);
}

// Start following a trajectory
void NewArm::followTrajectory(std::vector<PointNode> trajectory) {

	// Move based on straight line from point to point
	int current = 0;	// Index of point 1
	int next;
	float frac;

	while(1)
	{
		
		// Figure out current path segment based on time
		LONGLONG time = millisecondsNow();
		next = current + 1;
		while (trajectory[next].time < time) {
			current++;
			if (current == trajectory.size() - 1) // At end of path
				return;
			next = current + 1;
		}

		// Determine where along point 1 to point 2 we should be based on time
		frac = ((float)(time - trajectory[current].time))/(trajectory[next].time - trajectory[current].time);
		vec2 pos = interpolate(trajectory[current].point, trajectory[next].point, frac);

		// Move
		moveToPoint(pos);
	}
}

// Destructor
NewArm::~NewArm() {
	dxl_terminate();
}

// Returns coordinates of point a fraction between p1 and p2
// 0 <= fraction <= 1
vec2 NewArm::interpolate(Point p1, Point p2, float fraction) {
	if (fraction < 0)
		fraction = 0;
	else if (fraction > 1) 
		fraction = 1;

	float bearing = p1.angle(p2);
	float dist = p1.distance(p2);
	float dx = dist * fraction * cos(bearing);
	float dy = dist * fraction * sin(bearing);
	return vec2(p1.x + dx, p1.y + dy);
}

// Inverse kinematics. Returns joint angles for end effector point
vec2 NewArm::inverseKin(vec2 pos)
{
	float d, beta, gamma, theta;
	float x_ra, y_ra;
	x_ra = pos.x + X_BASE;
	y_ra = pos.y + Y_BASE;
	d = sqrt(x_ra*x_ra + y_ra*y_ra);
	//std::printf("d = %f\n", d); 
	beta = acos((L1*L1+L2*L2-d*d)/(2*L1*L2));
	//std::printf("beta = %f\n", beta);
	gamma = asin(sin(beta)*L2/d);
	//std::printf("gamma : %f\n", gamma);

	theta = atan2(y_ra, x_ra);
	/*if (x_ra < 0)
		theta = M_PI + theta;

	else if(x_ra == 0 && y_ra>0)
		theta = M_PI/2;
	else if (x_ra == 0 && y_ra<0)
		theta = -M_PI/2;
	else
		theta = theta;
    */
	return vec2(theta - gamma, M_PI - beta);
}

// Returns milliseconds to some reference point
// http://gamedev.stackexchange.com/questions/26759/best-way-to-get-elapsed-time-in-miliseconds-in-windows
LONGLONG millisecondsNow() {
	static LARGE_INTEGER s_frequency;
	static BOOL s_use_qpc = QueryPerformanceFrequency(&s_frequency);
	if (s_use_qpc) {
		LARGE_INTEGER now;
		QueryPerformanceCounter(&now);
		return (1000LL * now.QuadPart) / s_frequency.QuadPart;
	} else {
		return GetTickCount();
	}
}

// Returns position of end efffector
Point NewArm::getPos() {

	// Read from dynamixels
	int d1 = dxl_read_word(id[0], CT_PRESENTPOSITION_L);
	while ( dxl_get_result() != COMM_RXSUCCESS ) {
		Sleep(100);
		d1 = dxl_read_word(id[0], CT_PRESENTPOSITION_L);
	}
	int d2 = dxl_read_word(id[1], CT_PRESENTPOSITION_L);
	while ( dxl_get_result() != COMM_RXSUCCESS ) {
		Sleep(10);
		d2 = dxl_read_word(id[1], CT_PRESENTPOSITION_L);
	}

	//NOTES: q1 and q2 are in radians 
	float q1 = ((d1*300.f/1023.f) - 240.f)*(-M_PI/180.f);
	float q2 = ((d2*300.f/1023.f) - 150.f)*(M_PI/180.f);

	// Forward kinematics
	float x = L1*cos(q1)+L2*cos(q1+q2) - X_BASE;
	float y = L1*sin(q1) + L2*sin(q1+q2) - Y_BASE;

	//printf("q1: %f, q2: %f, x: %f, y: %f \n", q1*180/M_PI, q2*180/M_PI, x, y);
	return Point(x, y); 
}

void NewArm::chocMode() {

	// Send arm to default position
	resetConfig();

	// Lower arm
	setEndAngle(1.38);
	Sleep(300);

	// Move to position
	dxl_write_word(id[0], CT_GOALPOSITION_L, 800);
	Sleep(400);

	// Reduce speed
	dxl_write_word(BROADCAST_ID, CT_MOVINGSPEED_L, 32);
	Sleep(10);
	dxl_write_word(id[1], CT_GOALPOSITION_L, 925);
	Sleep(300);
	dxl_write_word(id[0], CT_GOALPOSITION_L, 696);
	Sleep(200);
	
}

void NewArm::exitChocMode() {
	dxl_write_word(id[0], CT_GOALPOSITION_L, 718);
	Sleep(10);
	dxl_write_word(id[1], CT_GOALPOSITION_L, 985);
	Sleep(500);

	// Set speed back to normal
	dxl_write_word(BROADCAST_ID, CT_MOVINGSPEED_L, 0);

	dxl_write_word(id[0], CT_GOALPOSITION_L, 787);
	Sleep(500);
	dxl_write_word(id[1], CT_GOALPOSITION_L, 800);
	Sleep(500);
	raiseArm();
	Sleep(500);
	resetConfig();

	Sleep(10);
}