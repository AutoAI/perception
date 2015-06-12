#define private public

#include <stddef.h>

#include "ros/ros.h"
#include "ros/console.h"
#include <gtest/gtest.h>
#include <stddef.h>

#include <climits>
#include <string>
#include <vector>

#include "../CoordinateList.h"
#include "../NdArray.h"
#include "../Triple.h"
#include "../Triangle.h"
#include "../Mesh.h"

TEST(NdArray, testCase1) {
    unsigned long bounds[3] = {3, 3, 3};
    NdArray<string> array(3, bounds);

    unsigned long location[3] = {0, 0, 0};
    string value = "asdf";
    array.set(location, value);
    EXPECT_EQ(value.compare(array.get(location)), 0);
}

TEST(NdArray, testCase2) {
    unsigned long bounds[3] = {3, 3, 3};
    NdArray<string> array(3, bounds);

    unsigned long location[3] = {0, 2, 0};
    string value = "sdflj";
    array.set(location, value);

    EXPECT_EQ(value.compare(array.get(location)), 0);
}

TEST(NdArray, testCase3) {
    unsigned long bounds[3] = {3, 3, 3};
    NdArray<string> array(3, bounds);

    unsigned long location[3] = {2, 0, 2};
    string value = "asldfkjsd";
    array.set(location, value);

    EXPECT_EQ(value.compare(array.get(location)), 0);
}

TEST(NdArray, testCase4) {
    unsigned long bounds[3] = {3, 3, 3};
    NdArray<string> array(3, bounds);

    unsigned long location[3] = {0, 0, 0};
    string value = "sdkjsfks";

    array.set(location, value);

    EXPECT_EQ(value.compare(array.get(location)), 0);

}

TEST(NdArray, testCase5) {
    unsigned long bounds[3] = {3, 3, 3};
    NdArray<string> array(3, bounds);

    unsigned long location[3] = {2, 0, 1};
    string value = "adfklsjdfksdflkjsdf";

    array.set(location, value);

    EXPECT_EQ(value.compare(array.get(location)), 0);
}

TEST(NdArray, testCase6) {
    unsigned long bounds[3] = {3, 3, 3};
    NdArray<string> array(3, bounds);

    unsigned long location[3] = {1, 3, 1};
    string value = "ksdflkjsdf";

    array.set(location, value);

    EXPECT_EQ(value.compare(array.get(location)), 0);
}

TEST(CoordinateList, toCartesian) {
    CoordinateList list(Spherical, 1);

    Triple coordinate1(1, 3.14159, 0);

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

    Triple coordinate1(1, 0, 10);
	Triple coordinate2(1, 1, 1);
    //coordinate1.x = 1;
    //coordinate1.y = 0;
    //coordinate1.z = 10;
    //coordinate2.x = 1;
    //coordinate2.y = 1;
    //coordinate2.z = 1;

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

    Triple coordinate1(1, 0, 10);
	Triple coordinate2(1, 1, 1);
    
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
    unsigned long t_length = 20;
    Triple t[t_length];

    CoordinateList test(0, t_length);
    EXPECT_EQ(t_length, test.getLength());
}

TEST(CoordinateList, testSort) {
    unsigned long t_length = 6;
    Triple t[t_length];

    for (unsigned long i = 0; i < t_length; i++) {
        t[i].x = float(rand()) / rand(); 
        t[i].y = float(rand()) / rand();
        t[i].z = float(rand()) / rand();
    }

    CoordinateList test(0, t_length);
    for (unsigned long i = 0; i < t_length; i++) {
        test.set(i, t[i]);
    }
    Triple c (float(rand()) / rand(), float(rand()) / rand(), float(rand()) / rand());

    test.sort(c);

    bool good = true;

    for(unsigned long i = 1; i < t_length; i++){
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

TEST(Mesh, testConstructor) {
	EXPECT_THROW(Mesh(NULL), std::invalid_argument);
}

TEST(Mesh, testConstructor2) {
	EXPECT_THROW(Mesh(new CoordinateList(0, 0)), std::invalid_argument);
}

TEST(Mesh, det22) {
    float** matrix = new float*[2];
    for(int i = 0; i < 2; i++)
        matrix[i] = new float[2];
    matrix[0][0] = 3;
    matrix[0][1] = -32;
    matrix[1][0] = 2;
    matrix[1][1] = 1;

	float ans = 67;
    float out = Mesh::det(matrix, 2);

	EXPECT_LT(abs(out - ans), 0.0001);
}

TEST(Mesh, det33) {
    float** matrix = new float*[3];
    for(int i = 0; i < 3; i++)
        matrix[i] = new float[3];
    matrix[0][0] = 3;
    matrix[0][1] = 1;
    matrix[0][2] = 4;
    matrix[1][0] = 2;
    matrix[1][1] = 3;
    matrix[1][2] = 43;
    matrix[2][0] = 4;
    matrix[2][1] = -4;
    matrix[2][2] = 2;

	float ans = 622;
    float out = Mesh::det(matrix, 3);
	
	EXPECT_LT(abs(out - ans), 0.0001);
}

TEST(Mesh, det44) {
    float** matrix = new float*[4];
    for(int i = 0; i < 4; i++)
        matrix[i] = new float[4];
    matrix[0][0] = 1;
    matrix[0][1] = 2;
    matrix[0][2] = 0;
    matrix[0][3] = 1;
    matrix[1][0] = 2;
    matrix[1][1] = 1;
    matrix[1][2] = 1;
    matrix[1][3] = 0;
    matrix[2][0] = -1;
    matrix[2][1] = 1;
    matrix[2][2] = -2;
    matrix[2][3] = 1;
    matrix[3][0] = 1;
    matrix[3][1] = 1;
    matrix[3][2] = 2;
    matrix[3][3] = 2;

	float ans = 5;
    float out = Mesh::det(matrix, 4);

	EXPECT_LT(abs(out - ans), 0.0001);


}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
