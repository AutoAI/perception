#include "ros/ros.h"

#include "seeded_depth_map.h"
#include <time.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "run");
	ros::NodeHandle n;

	ros::spinOnce();

	if (ros::ok()) {
		srand(time(NULL));
		ROS_INFO("Running...");
		ROS_INFO("==========");
		SeededDepthMap seed;
		seed.doCorrespondence();
		ROS_INFO("==========");
		ROS_INFO("Successful");
	}
}
