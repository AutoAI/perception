#include "ros/ros.h"

#include "NdArray.cpp"
#include "CoordinateList.cpp"

using namespace std;

int main(int argc, char **argv){ 
    ros::init(argc, argv, "Perception_Run");
    ros::NodeHandle n;

    ros::spinOnce();

    if (ros::ok()) {
        CoordinateList list(Cartesian);
        ROS_INFO("Running");
    }
}