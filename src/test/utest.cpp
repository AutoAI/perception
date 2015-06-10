#include "ros/ros.h"
#include "ros/console.h"
#include <gtest/gtest.h>
#include <climits>

#include "../NdArray.cpp"
#include <string>

#include "../CoordinateList.cpp"

using namespace std;

TEST(NdArray, testCase1) {
    size_t bounds[3] = {3, 3, 3};
    NdArray<string> array(3, bounds);

    size_t location[3] = {0, 0, 0};
    string value = "asdf";
    array.set(location, value);
    EXPECT_EQ(value.compare(array.get(location)), 0);
}

TEST(NdArray, testCase2) {
    size_t bounds[3] = {3, 3, 3};
    NdArray<string> array(3, bounds);

    size_t location[3] = {0, 2, 0};
    string value = "sdflj";
    array.set(location, value);

    EXPECT_EQ(value.compare(array.get(location)), 0);
}

TEST(NdArray, testCase3) {
    size_t bounds[3] = {3, 3, 3};
    NdArray<string> array(3, bounds);

    size_t location[3] = {2, 0, 2};
    string value = "asldfkjsd";
    array.set(location, value);

    EXPECT_EQ(value.compare(array.get(location)), 0);
}

TEST(NdArray, testCase4) {
    size_t bounds[3] = {3, 3, 3};
    NdArray<string> array(3, bounds);

    size_t location[3] = {0, 0, 0};
    string value = "sdkjsfks";

    array.set(location, value);

    EXPECT_EQ(value.compare(array.get(location)), 0);

}

TEST(NdArray, testCase5) {
    size_t bounds[3] = {3, 3, 3};
    NdArray<string> array(3, bounds);

    size_t location[3] = {2, 0, 1};
    string value = "adfklsjdfksdflkjsdf";

    array.set(location, value);

    EXPECT_EQ(value.compare(array.get(location)), 0);
}

TEST(NdArray, testCase6) {
    size_t bounds[3] = {3, 3, 3};
    NdArray<string> array(3, bounds);

    size_t location[3] = {1, 3, 1};
    string value = "ksdflkjsdf";

    array.set(location, value);

    EXPECT_EQ(value.compare(array.get(location)), 0);
}

TEST(CoordinateList, toCartesian) {
    CoordinateList list(Spherical, 1);

    Triple coordinate1;
    coordinate1.x = 1;
    coordinate1.y = 3.14159;
    coordinate1.z = 0;

    list.set(0, coordinate1);

    list.toType(Cartesian);
    Triple a; 

    a = list.get(0);
    EXPECT_LT(abs(a.x-0), 0.0001);
    EXPECT_LT(abs(a.y+0), 0.0001);
    EXPECT_LT(abs(a.z-1), 0.0001);
}

TEST(CoordinateList, toSpherical) {
    CoordinateList list(Cartesian, 2);

    Triple coordinate1, coordinate2;
    coordinate1.x = 1;
    coordinate1.y = 0;
    coordinate1.z = 10;
    coordinate2.x = 1;
    coordinate2.y = 1;
    coordinate2.z = 1;

    list.set(0, coordinate1);
    list.set(1, coordinate2);
    
    list.toType(Spherical);
    Triple a; 

    a = list.get(0);
    EXPECT_LT(abs(a.x-sqrt(101)), 0.0001);
    EXPECT_LT(abs(a.y-0), 0.0001);
    EXPECT_LT(abs(a.z-acos(10/sqrt(101))), 0.0001);

    a = list.get(1);
    EXPECT_LT(abs(a.x-sqrt(3)), 0.0001);
    EXPECT_LT(abs(a.y-atan(1)), 0.0001);
    EXPECT_LT(abs(a.z-acos(1/sqrt(3))), 0.0001);

}

 
TEST(CoordinateList, toPerspective) {
    CoordinateList list(0, 2);

    Triple coordinate1, coordinate2;
    coordinate1.x = 1;
    coordinate1.y = 0;
    coordinate1.z = 10;
    coordinate2.x = 1;
    coordinate2.y = 1;
    coordinate2.z = 1;
    
    list.set(0, coordinate1);
    list.set(1, coordinate2);

    list.toType(Perspective);

    Triple a;
    
    a = list.get(0);
    EXPECT_LT(abs(a.x-F/10), 0.0001);
    EXPECT_LT(abs(a.y-0), 0.0001);
    EXPECT_LT(abs(a.z-K/10), 0.0001);
    
    a = list.get(1);
    EXPECT_LT(abs(a.x-F), 0.0001);
    EXPECT_LT(abs(a.y-F), 0.00001);
    EXPECT_LT(abs(a.z-K), 0.0001);
}

TEST(CoordinateList, testSize) {
    size_t t_length = 20;
    Triple t[t_length];

    CoordinateList test(0, t_length);
    EXPECT_EQ(t_length, test.getLength());
}

TEST(CoordinateList, testSort) {
    size_t t_length = 6;
    Triple t[t_length];

    for (size_t i = 0; i < t_length; i++) {
        t[i].x = float(rand()) / rand(); 
        t[i].y = float(rand()) / rand();
        t[i].z = float(rand()) / rand();
    }

    CoordinateList test(0, t_length);
    for (size_t i = 0; i < t_length; i++) {
        test.set(i, t[i]);
    }
    Triple c;
    c.x = float(rand()) / rand();
    c.y = float(rand()) / rand();
    c.z = float(rand()) / rand();

    test.sort(c);

    bool good = true;

    for(size_t i = 1; i < t_length; i++){
        float dxi = test.get(i).x - c.x;
        float dyi = test.get(i).y - c.y;
        float dximinus1 = test.get(i-1).x - c.x;
        float dyiminus1 = test.get(i-1).y - c.y;
        if(dxi*dxi+dyi*dyi < dximinus1*dximinus1+dyiminus1*dyiminus1){
            ROS_INFO("%f > %f", dxi*dxi+dyi*dyi, dximinus1*dximinus1+dyiminus1*dyiminus1);
            good = false;
        }
    }
    EXPECT_TRUE(good);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
