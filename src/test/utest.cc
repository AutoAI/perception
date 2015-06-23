#define private public // oh my god.

#include <stddef.h>

#include "ros/ros.h"
#include "ros/console.h"
#include <gtest/gtest.h>
#include <stddef.h>
#include <iostream>

#include <time.h>
#include <climits>
#include <string>
#include <vector>

#include "../coordinate_list.h"
#include "../nd_array.h"
#include "../triple.h"
#include "../triangle.h"
#include "../mesh.h"
#include "../mesh_triple.h"
#include "../global_constants.h"

using namespace std;

TEST(Triple, equalityTrue) {
	Triple a;
	Triple b;
	
	a.x = 1;
	a.y = 1;
	a.z = 1;

	b.x = 1;
	b.y = 1;
	b.z = 1;

	EXPECT_TRUE(a==b);
}

TEST(Triple, equalityFalse) {
	Triple a;
	Triple b;
	
	a.x = 1;
	a.y = 1;
	a.z = 1;

	b.x = 1;
	b.y = 2;
	b.z = 1;

	EXPECT_FALSE(a==b);
}

TEST(Triple, inequalityTrue) {
	Triple a(1, 1, 3);
	Triple b;

	b.x = 1;
	b.y = 1;
	b.z = 1;

	EXPECT_TRUE(a!=b);
}

TEST(Triple, inequalityFalse) {
	Triple a(1, 2, 2);
	Triple b(1, 2, 2);

	EXPECT_FALSE(a!=b);
}

TEST(Triangle, eqTrue) {
	MeshTriple t1(new Triple(1, 1, 1));
	MeshTriple t2(new Triple(1, 1, 2));
	MeshTriple t3(new Triple(1, 2, 2));

	MeshTriple t4(new Triple(1, 1, 1));
	MeshTriple t5(new Triple(1, 1, 2));
	MeshTriple t6(new Triple(1, 2, 2));

	Triangle tri1(&t1, &t2, &t3);
	Triangle tri2(&t4, &t5, &t6);

	EXPECT_TRUE(tri1 == tri2);
}

TEST(NdArray, testCase1) {
    unsigned long bounds[3] = {3, 3, 3};
    NdArray<float> array(3, bounds);

    unsigned long location[3] = {0, 2, 0};
    float value = 69.69;
    array.set(location, value);

    EXPECT_EQ(value, array.get(location));
}

TEST(NdArray, testCase2){
    unsigned long bounds[3] = {200, 100, 6};
    NdArray<float> array(3, bounds);

    unsigned long location[3] = {0, 1, 0};
    float value = 69.69;
    array.set(location, value);

    EXPECT_LT(abs(value - array.get(location)), 0.0001);
}

TEST(CoordinateList, toCartesian) {
    CoordinateList list(CoordinateList::SPHERICAL, 1);

    Triple coordinate1(1, 3.14159, 0);

    list.set(0, coordinate1);

    list.toType(CoordinateList::CARTESIAN);
    Triple a; 

    a = list.get(0);
    EXPECT_LT(abs(a.x-0), 0.0001);
    EXPECT_LT(abs(a.y+0), 0.0001);
    EXPECT_LT(abs(a.z-1), 0.0001);
}

TEST(CoordinateList, toSpherical) {
    CoordinateList list(CoordinateList::CARTESIAN, 2);

    Triple coordinate1(1, 0, 10);
	Triple coordinate2(1, 1, 1);

    list.set(0, coordinate1);
    list.set(1, coordinate2);
    
    list.toType(CoordinateList::SPHERICAL);
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
    CoordinateList list(CoordinateList::CARTESIAN, 2);

    Triple coordinate1(1, 0, 10);
	Triple coordinate2(1, 1, 1);
    
	list.set(0, coordinate1);
    list.set(1, coordinate2);

    list.toType(CoordinateList::PERSPECTIVE);

    Triple a;
    
    a = list.get(0);
    EXPECT_LT(abs(a.x-CameraConstants::F/10), 0.0001);
    EXPECT_LT(abs(a.y-0), 0.0001);
    EXPECT_LT(abs(a.z-CameraConstants::K/10), 0.0001);
    
    a = list.get(1);
    EXPECT_LT(abs(a.x-CameraConstants::F), 0.0001);
    EXPECT_LT(abs(a.y-CameraConstants::F), 0.00001);
    EXPECT_LT(abs(a.z-CameraConstants::K), 0.0001);
}

TEST(CoordinateList, testSize) {
    unsigned long t_length = 20;
    Triple t[t_length];

    CoordinateList test(CoordinateList::CARTESIAN, t_length);
    EXPECT_EQ(t_length, test.getLength());
}

TEST(CoordinateList, testSort) {
    unsigned long t_length = 1000;
    Triple t[t_length];

    for (unsigned long i = 0; i < t_length; i++) {
        t[i].x = float(rand()) / rand(); 
        t[i].y = float(rand()) / rand();
        t[i].z = float(rand()) / rand();
    }

    CoordinateList test(CoordinateList::CARTESIAN, t_length);
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
            good = false;
        }
    }
    EXPECT_TRUE(good);
}

TEST(CoordinateList, testSort2) {
    unsigned long t_length = 1000;
    Triple t[t_length];

    for (unsigned long i = 0; i < t_length; i++) {
        t[i].x = float(rand()) / rand(); 
        t[i].y = float(rand()) / rand();
        t[i].z = float(rand()) / rand();
    }

    CoordinateList test(CoordinateList::CARTESIAN, t_length);
    for (unsigned long i = 0; i < t_length; i++) {
        test.set(i, t[i]);
    }
    Triple c (float(rand()) / rand(), float(rand()) / rand(), float(rand()) / rand());

    test.sortThatDoesntWorkYet(c);

    bool good = true;

    for(unsigned long i = 1; i < t_length; i++){
        float dxi = test.get(i).x - c.x;
        float dyi = test.get(i).y - c.y;
        float dximinus1 = test.get(i-1).x - c.x;
        float dyiminus1 = test.get(i-1).y - c.y;
        if(dxi*dxi+dyi*dyi < dximinus1*dximinus1+dyiminus1*dyiminus1){
            good = false;
        }
    }
    EXPECT_TRUE(good);
}

TEST(Mesh, testConstructor) {
	EXPECT_THROW(Mesh(NULL), std::invalid_argument);
}

TEST(Mesh, testConstructor2) {
	EXPECT_THROW(Mesh(new CoordinateList(CoordinateList::CARTESIAN, 0)), std::invalid_argument);
}

TEST(Mesh, det2x2) {
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

TEST(Mesh, det3x3) {
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

TEST(Mesh, det4x4) {
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

TEST(Mesh, testIntersect){
    Triple t1(2, 2, 4);
    Triple t2(5, 3, 2);
    Triple t3(1, 4, 98);
    Triple t4(3, 3, 7);
    Triple t5(3, 2, -1);

    EXPECT_TRUE(Mesh::testIntersect(t1, t4, t3, t5));
    EXPECT_TRUE(Mesh::testIntersect(t3, t5, t1, t4));
    EXPECT_TRUE(Mesh::testIntersect(t4, t1, t5, t3));
    EXPECT_TRUE(Mesh::testIntersect(t5, t3, t4, t1));
    EXPECT_FALSE(Mesh::testIntersect(t1, t5, t4, t2));
    EXPECT_FALSE(Mesh::testIntersect(t4, t2, t1, t5));
    EXPECT_FALSE(Mesh::testIntersect(t5, t1, t2, t4));
    EXPECT_FALSE(Mesh::testIntersect(t2, t4, t5, t1));
}

TEST(Mesh, dist2) {
	Triple t1(0, 0, 0);
	Triple t2(0, 0, 1);

	EXPECT_EQ(Mesh::dist2(t1, t2), 0);
}

TEST(Mesh, dist23) {
	Triple t1(2, 2, 2);
	Triple t2(0, 0, 0);

	EXPECT_EQ(Mesh::dist2(t1, t2), 8);
}

TEST(Mesh, dist24) {
	Triple t1(3, 4, 9);
	Triple t2(0, 0, 0);

	EXPECT_EQ(Mesh::dist2(t1, t2), 25);
}

TEST(Mesh, inCircumCirc) {
    Triple* s1 = new Triple(0, 2, 0);
    Triple* s2 = new Triple(0, 0, 0);
    Triple* s3 = new Triple(2, 0, 0);

    Triple* t1 = new Triple(1, 1, 0);
    Triple* t2 = new Triple(2, 2, 0);
    Triple* t3 = new Triple(3, 3, 0);

    EXPECT_TRUE(Mesh::inCircumCirc(s1, s2, s3, t1));
    EXPECT_FALSE(Mesh::inCircumCirc(s1, s2, s3, t2));
    EXPECT_FALSE(Mesh::inCircumCirc(s1, s2, s3, t3));
}

TEST(Mesh, getNeighbors) {
    MeshTriple* m1 = new MeshTriple(new Triple(0, 0, 0));
    MeshTriple* m2 = new MeshTriple(new Triple(1, 1, 1));
    MeshTriple* m3 = new MeshTriple(new Triple(2, 0, 2));
    MeshTriple* m4 = new MeshTriple(new Triple(3, 1, 3));
    MeshTriple* m5 = new MeshTriple(new Triple(4, 0, 4));

    Triangle* t1 = new Triangle(m1, m2, m3);
    Triangle* t2 = new Triangle(m2, m3, m4);
    Triangle* t3 = new Triangle(m3, m4, m5);

    EXPECT_EQ(Mesh::getNeighbors(m1).size(), 2);
    EXPECT_EQ(Mesh::getNeighbors(m2).size(), 3);
    EXPECT_EQ(Mesh::getNeighbors(m3).size(), 4);
    EXPECT_EQ(Mesh::getNeighbors(m4).size(), 3);
    EXPECT_EQ(Mesh::getNeighbors(m5).size(), 2);

    EXPECT_EQ(Mesh::getNeighbors(t1).size(), 1);
    EXPECT_EQ(Mesh::getNeighbors(t2).size(), 2);
    EXPECT_EQ(Mesh::getNeighbors(t3).size(), 1);
}

TEST(Mesh, constructor){
	int length = 10;
	CoordinateList c(CoordinateList::CARTESIAN, length);

    for(int i = 0; i < length; i++) {
        Triple temp(((float)(rand())/(float)(RAND_MAX) - .5) * CameraConstants::S, ((float)(rand())/(float)(RAND_MAX) - .5) * CameraConstants::S * CameraConstants::YRES/CameraConstants::XRES, ((float)(rand())/(float)(RAND_MAX)) * CameraConstants::K);
        c.set(i, temp);
    }

	Mesh m(&c);
}

int main(int argc, char **argv) {
    srand(time(NULL));
    testing::InitGoogleTest(&argc, argv);
	for (int i = 0; i < 11; i++) cout << RUN_ALL_TESTS() << endl;
    return 0;
}
