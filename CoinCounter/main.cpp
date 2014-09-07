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
	/*
	ImageProcessor i;
	std::vector<Object> coins; float speed;
	i.run(ImageProcessor::MODE_TRAINCLASSIFY, 5, coins, speed); 
	*/
	while (1) {
		printf("Place coins then press enter.");
		getchar();
		planner.findCoins();

		//if (planner.items.size() == 0)
		//	break;

		planner.go();		
	} 
	
	//planner.findCoins();
	
	// Writing mode
	//printf("Please attach pen.\n");
	//_getch();
	//planner.writeWord("M", Point(-80, 0));

	/*
	NewArm arm;
	arm.lowerArm();
	while (1) {
		arm.getPos();
		Sleep(200);
	}
	*/

	return 0;
}