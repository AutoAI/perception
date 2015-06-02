#include "ros/ros.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include <functional>
#include <iterator>

#include "NdArray.cpp"

#define gridRes 100
#define numPoints 1000
#define pointRange 1000

using namespace std;

int main(int argc, char **argv){ 
	ros::init(argc, argv, "Run");
	ros::NodeHandle n;
	ros::spinOnce();
	if (ros::ok()) {
		// initialize random seed
		srand(time(NULL));

		// make some random points
		size_t s2[2] = {numPoints, 3};
		NdArray<float> points(2, s2);
		for(size_t i = 0; i+1 < numPoints*3; i+=3){
			points.set(i, static_cast<float>(rand())/(static_cast<float>(RAND_MAX/(2*pointRange)))-pointRange);
			points.set(i+1, static_cast<float>(rand())/(static_cast<float>(RAND_MAX/(2*pointRange)))-pointRange);
		}

		return 0;
	}
}