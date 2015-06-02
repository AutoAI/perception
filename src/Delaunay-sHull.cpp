#include "ros/ros.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "NdArray.cpp"

using namespace std;

int main(int argc, char **argv){ 
    ros::init(argc, argv, "Run");

	ros::NodeHandle n;

	ros::spinOnce();

	if (ros::ok()) {
            ROS_INFO("Parth Is good");
			// initialize random seed
			srand(time(NULL));
			// make a grid
			char d = 100;
			char s[2] = {d, d};
			NdArray<char> grid(2, s);
			return 0;
	}
}
