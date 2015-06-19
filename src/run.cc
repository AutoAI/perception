#include "ros/ros.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "Perception_Run");
	ros::NodeHandle n;

	ros::spinOnce();

	if (ros::ok()) {
		ROS_INFO("Running");
	}
}
