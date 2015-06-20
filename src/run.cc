#include "ros/ros.h"

#include "seeded_depth_map.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "run");
	ros::NodeHandle n;

	ros::spinOnce();

	if (ros::ok()) {
		SeededDepthMap seed;
		seed.doCorrespondence();
		ROS_INFO("Running");
	}
}
