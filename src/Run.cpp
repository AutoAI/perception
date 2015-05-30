#include "Run.h"

#include "NdArray.cpp"
#include "CoordinateList.cpp"

using namespace std;

int main(int argc, char **argv){ 
    ros::init(argc, argv, "Run");
    ros::NodeHandle n;

    ros::spinOnce();

    if (ros::ok()) {
        CoordinateList list(Cartesian);
        ROS_INFO("Running");
    }
}
