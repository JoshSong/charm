#ifndef GLOBALS_H_
#define GLOBALS_H_

#include "charm/shared/Point.h"


// Put global variables here
extern Point ORIGIN;

// In mm
extern float TABLE_RADIUS;
extern float BIN_RADIUS;
extern float MARGIN;	// Added to coin radius for collision detection in AStar

// In radians / s
extern float ROT_SPEED;	// Radians / 
extern float ROT_SPEED_FACTOR;	// Adjustment factor

extern int TIME_STEP;	// Milliseconds
extern float DIST_STEP;	// Distance arm can move in one time step

// Motion planning & motor control
extern int PLAN_TIME_OFFSET; // Milliseconds. Roughly the time required to plan the path
extern int DROP_TIME;	// Milliseconds, time required to lower arm tip
extern int MOTOR_DELAY;	// Account for slight lag in motor response to commands
extern int ADJUST_FACTOR;	// Use this to adjust if the arm fails to hit moving coins

// A* search stuff
extern float CELL_SIZE;		// Should be = DIST_STEP
extern int BORDER_CELLS;	// Add cells surrounding outside of table
extern int NUM_ROWS;
extern int NUM_COLS;

#endif	// GLOBALS_H_
