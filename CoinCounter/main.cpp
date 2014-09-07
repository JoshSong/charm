#include <stdio.h>
#include "MainPlanner.h"



int main(int argc, char *argv[])
{
	MainPlanner planner;

	// Enter chocolate mode
	//planner.arm.chocMode();
	//Sleep(4000);
	//planner.arm.exitChocMode();

	// Coin sorting mode

	// Colour calibration

	if (argc == 2 && strcmp(argv[1], "train") == 0) {
		ImageProcessor i;
		std::vector<Object> coins; float speed;
		i.run(ImageProcessor::MODE_TRAINCLASSIFY, 10, coins, speed); 
		return 0;
	}

	printf("Place coins then press enter.");
	getchar();
	
	while (planner.findCoins()) {
		planner.go();	

		printf("Place coins then press enter.");
		getchar();
	} 

	return 0;
}