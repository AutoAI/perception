#include "ros/ros.h"
#include "std_msgs/String.h"

#include "NdArray.cpp"
#include "Foo.cpp"
#include <string>

#include <sstream>

class Test{ 
    public:
        Test();
        int add(int a, int b);
};
