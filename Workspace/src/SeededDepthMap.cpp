#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv) {
    ros::init(argc,  argv, "driveai_vision");
    ros::NodeHandle n;

    while(ros::ok()) {
        std::stringstream ss;
        ROS_INFO("Hello");
        ros::spinOnce();
    }

    return 0;
}
