#include "ros/ros.h"

#include "nd_array.cpp"
#include "coordinate_list.cpp"

int main(int argc, char **argv) {
	ros::init(argc, argv, "Perception_Run");
	ros::NodeHandle n;

	ros::spinOnce();

	if (ros::ok()) {
		CoordinateList list(Cartesian);
		ROS_INFO("Running");
	}
}
