#include <gtest/gtest.h>
#include <climits>

#include "../NdArray.cpp"
#include <string>

#include "../CoordinateList.cpp"

using namespace std;

TEST(NdArray, testCase1) {
    char bounds[3] = {3, 3, 3};
    NdArray<string> array(3, bounds);

    char location[3] = {0, 0, 0};
    string value = "asdf";
    array.set(location, value);
    EXPECT_EQ(value.compare(array.get(location)), 0);
}

TEST(NdArray, testCase2) {
    char bounds[3] = {3, 3, 3};
    NdArray<string> array(3, bounds);

    char location[3] = {0, 2, 0};
    string value = "sdflj";
    array.set(location, value);

    EXPECT_EQ(value.compare(array.get(location)), 0);
}

TEST(NdArray, testCase3) {
    char bounds[3] = {3, 3, 3};
    NdArray<string> array(3, bounds);

    char location[3] = {2, 0, 2};
    string value = "asldfkjsd";
    array.set(location, value);

    EXPECT_EQ(value.compare(array.get(location)), 0);
}

TEST(NdArray, testCase4) {
    char bounds[3] = {3, 3, 3};
    NdArray<string> array(3, bounds);

    char location[3] = {0, 0, 0};
    string value = "sdkjsfks";

    array.set(location, value);

    EXPECT_EQ(value.compare(array.get(location)), 0);

}

TEST(NdArray, testCase5) {
    char bounds[3] = {3, 3, 3};
    NdArray<string> array(3, bounds);

    char location[3] = {2, 0, 1};
    string value = "adfklsjdfksdflkjsdf";

    array.set(location, value);

    EXPECT_EQ(value.compare(array.get(location)), 0);
}

TEST(NdArray, testCase6) {
    char bounds[3] = {3, 3, 3};
    NdArray<string> array(3, bounds);

    char location[3] = {1, 3, 1};
    string value = "ksdflkjsdf";

    array.set(location, value);

    EXPECT_EQ(value.compare(array.get(location)), 0);
}

TEST (CoordinateList, stestCase1) {
    CoordinateList list(0);

    Coordinate coordinate1[3] = {1, 0, 10};
    list.addCoordinate(coordinate1);
    
    Coordinate coordinate2[3] = {1, 1, 1};
    list.addCoordinate(coordinate2);

    
    list.toType(Spherical);
    Coordinate* a; 
    a = list.getCoordinate(0);
    EXPECT_TRUE(a[0]-sqrt(101)<0.0001 && a[1]-0<0.0001 && a[2]-acos(10/sqrt(101))<0.0001);

    a = list.getCoordinate(1);
    EXPECT_TRUE(a[0]-sqrt(3)<0.0001 && a[1]-atan(1)<0.00001 && a[2]-acos(1/sqrt(3))<0.0001);

    list.toType(Perspective);
     a = list.getCoordinate(0);
    EXPECT_TRUE(a[0]-F/10<0.0001 && a[1]-0<0.0001 && a[2]-K/10<0.0001);
    
    a = list.getCoordinate(1);
    EXPECT_TRUE(a[0]-F <0.0001 && a[1]-F <0.00001 && a[2]-K <0.0001);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
