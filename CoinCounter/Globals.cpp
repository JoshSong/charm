
#include "stdafx.h"
#include "Globals.h"

// Put global variables here
Point ORIGIN(0,0);

// In mm
float TABLE_RADIUS = 95;
float BIN_RADIUS = 30;
float MARGIN = 8;		// Added to coin radius for collision detection in AStar

// In radians / s
float ROT_SPEED = 0;	// Radians / second, rotation speed of turntable
float ROT_SPEED_FACTOR = 1.0;	// Adjustment factor

LONGLONG TIME_STEP = 10;	// Milliseconds, for path planning. Decrease to be faster.
float DIST_STEP = 3.f;	// Approximate distance arm can move in one time step in mm. Path planning resolution.

// Motion planning stuff
LONGLONG PLAN_TIME_OFFSET = 500;	// Milliseconds
LONGLONG DROP_TIME = 200;	// Milliseconds, time required to drop arm
LONGLONG MOTOR_DELAY = 200;	// Account for slight lag in motor response to commands
LONGLONG ADJUST_FACTOR = 200;	// Use this to adjust if the arm fails to hit moving coins

// A* search stuff
// IMPORTANT: The grid should cover all the area the arm will move in
float CELL_SIZE = DIST_STEP;	// In mm
int BORDER_CELLS = 15;	// Add cells surrounding outside of table
int NUM_ROWS = ceil((2 * TABLE_RADIUS) / CELL_SIZE) + 2 * BORDER_CELLS;
int NUM_COLS = ceil((2 * TABLE_RADIUS) / CELL_SIZE) + 2 * BORDER_CELLS;
