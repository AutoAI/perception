#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv){ 
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    ros::spinOnce();

    if (ros::ok()) {

        ROS_INFO("hello");


    }
}
