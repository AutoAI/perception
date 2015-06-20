#include "ros/ros.h"

#include "seeded_depth_map.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "seeded depth map");
	ros::NodeHandle n;

	ros::spinOnce();

	if (ros::ok()) {
		SeededDepthMap seed;
		seed.doCorrespondence();
		ROS_INFO("Running");
	}
}
