#include "ros/ros.h"

#include <time.h>
#include <vector>

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


		// make the triangulation


		return 0;
	}
}