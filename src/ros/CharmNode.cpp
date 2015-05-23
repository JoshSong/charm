#include <ros/ros.h>
#include <signal.h>

#include "charm/planner/MainPlanner.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "charm_node");
	ros::NodeHandle nh;

	MainPlanner planner;

	if (argc == 2 && strcmp(argv[1], "train") == 0) {
		ImageProcessor i;
		std::vector<Object> coins;
		float speed;
		ros::Time lastCamTime;
		i.run(ImageProcessor::MODE_TRAINCLASSIFY, 10, coins, speed, lastCamTime);
		return 0;
	}

	printf("Place coins then press enter.");
	getchar();

	while (planner.findCoins()) {
		planner.go();

		printf("Place coins then press enter.");
		getchar();
	}

	//ros::spin();
	return 0;
}
