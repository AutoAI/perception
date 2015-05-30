#include "Test.h"

int Test::add(int a, int b) {
    return a+b;
}

using namespace std;
void testNdArray() {
    char bounds[3] = {3, 3, 3};
    NdArray<string> array(3, bounds);

    char location[3] = {0, 0, 0};
    string value = "asdf";
    array.set(location, value);
    
    if (value.compare(array.get(location)) == 0) {
        ROS_INFO("PASS");
    } else {
        ROS_INFO("FAIL");
    }

    location[0] = 0;
    location[1] = 2;
    location[2] = 0;
    value = "sdflj";
    array.set(location, value);
    
    if (value.compare(array.get(location)) == 0) {
        ROS_INFO("PASS");
    } else {
        ROS_INFO("FAIL");
    }
    
    location[0] = 2;
    location[1] = 0;
    location[2] = 2;
    value = "sdflkjsd";
    array.set(location, value);
    
    if (value.compare(array.get(location)) == 0) {
        ROS_INFO("PASS");
    } else {
        ROS_INFO("FAIL");
    }

    location[0] = 2;
    location[1] = 1;
    location[2] = 1;
    value = "lkjsd";
    array.set(location, value);
    
    if (value.compare(array.get(location)) == 0) {
        ROS_INFO("PASS");
    } else {
        ROS_INFO("FAIL");
    }
    
    location[0] = 1;
    location[1] = 1;
    location[1] = 1;
    value = "sdflkjsd";
    array.set(location, value);
    
    if (value.compare(array.get(location)) == 0) {
        ROS_INFO("PASS");
    } else {
        ROS_INFO("FAIL");
    }

    location[0] = 1;
    location[1] = 2;
    location[2] = 1;
    value = "dflkjsd";
    array.set(location, value);
    
    if (value.compare(array.get(location)) == 0) {
        ROS_INFO("PASS");
    } else {
        ROS_INFO("FAIL");
    }
}

int main(int argc, char **argv){ 
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    ros::spinOnce();

    if (ros::ok()) {
        testNdArray();
    }
}
