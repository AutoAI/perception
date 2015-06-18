#include "ros/ros.h"

#include "nd_array.cc"
#include "coordinate_list.cc"

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
