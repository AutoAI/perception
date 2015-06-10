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

		// choose seed point, sort others accorinding to distance from seed

		//create a ds where there are points xyz and a list of of triangls. Triangles will have a Point a, b, c

		// construct initial convex hull (right-hand), determine circum-circle center

		// sequentially insert points, adding edges from new point to 'visible' points on the convex hull


		// iteratively 'flip' triangles until no more triangles need be flipped


		return 0;
	}
}
