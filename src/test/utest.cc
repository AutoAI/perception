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

TEST(Triple, equalityOperators) {
	Triple a(1, 1, 1);
	Triple b(1, 1, 1);

	EXPECT_TRUE(a==b);
    EXPECT_FALSE(a!=b);

    b.y = 2;

    EXPECT_FALSE(a==b);
    EXPECT_TRUE(a!=b);
}

TEST(MeshTriple, equalityOperators) {
    MeshTriple a(new Triple(1, 1, 1));
    MeshTriple b(new Triple(1, 1, 1));

    EXPECT_TRUE(a==b);
    EXPECT_FALSE(a!=b);

    b.triple -> y = 2;

    EXPECT_FALSE(a==b);
    EXPECT_TRUE(a!=b);
}

TEST(MeshTriple, isNeighbor_GetNeighbors) {
    // some triples
    Triple* t1 = new Triple(0, 0, 1);
    Triple* t2 = new Triple(0, 2, 2);
    Triple* t3 = new Triple(2, 2, 3);
    Triple* t4 = new Triple(2, 0, 4);
    Triple* t5 = new Triple(1, 1, 5);
    // their meshtriples
    MeshTriple* m1 = new MeshTriple(t1);
    MeshTriple* m2 = new MeshTriple(t2);
    MeshTriple* m3 = new MeshTriple(t3);
    MeshTriple* m4 = new MeshTriple(t4);
    MeshTriple* m5 = new MeshTriple(t5);
    // some triangles
    Triangle* tri1 = new Triangle(m1, m2, m5);
    Triangle* tri2 = new Triangle(m2, m3, m5);
    Triangle* tri3 = new Triangle(m3, m4, m5);
    Triangle* tri4 = new Triangle(m4, m1, m5);
    // tests
    EXPECT_TRUE(m1 -> isNeighbor(m4));
    EXPECT_TRUE(m1 -> isNeighbor(m2));
    EXPECT_TRUE(m1 -> isNeighbor(m5));
    EXPECT_TRUE(m2 -> isNeighbor(m1));
    EXPECT_TRUE(m2 -> isNeighbor(m3));
    EXPECT_TRUE(m2 -> isNeighbor(m5));
    EXPECT_TRUE(m3 -> isNeighbor(m2));
    EXPECT_TRUE(m3 -> isNeighbor(m4));
    EXPECT_TRUE(m3 -> isNeighbor(m5));
    EXPECT_TRUE(m4 -> isNeighbor(m3));
    EXPECT_TRUE(m4 -> isNeighbor(m1));
    EXPECT_TRUE(m4 -> isNeighbor(m5));
    EXPECT_TRUE(m5 -> isNeighbor(m1));
    EXPECT_TRUE(m5 -> isNeighbor(m2));
    EXPECT_TRUE(m5 -> isNeighbor(m3));
    EXPECT_TRUE(m5 -> isNeighbor(m4));
    EXPECT_FALSE(m1 -> isNeighbor(m3));
    EXPECT_FALSE(m2 -> isNeighbor(m4));
    EXPECT_FALSE(m3 -> isNeighbor(m1));
    EXPECT_FALSE(m4 -> isNeighbor(m2));
    EXPECT_EQ(m1 -> getNeighbors().size(), 3);
    EXPECT_EQ(m2 -> getNeighbors().size(), 3);
    EXPECT_EQ(m3 -> getNeighbors().size(), 3);
    EXPECT_EQ(m4 -> getNeighbors().size(), 3);
    EXPECT_EQ(m5 -> getNeighbors().size(), 4);
}

TEST(Triangle, equalityOperators) {
	MeshTriple t1(new Triple(1, 1, 1));
	MeshTriple t2(new Triple(1, 1, 2));
	MeshTriple t3(new Triple(1, 2, 2));

	MeshTriple t4(new Triple(1, 1, 1));
	MeshTriple t5(new Triple(1, 1, 2));
	MeshTriple t6(new Triple(1, 2, 2));

	Triangle tri1(&t1, &t2, &t3);
	Triangle tri2(&t4, &t5, &t6);

	EXPECT_TRUE(tri1 == tri2);
    EXPECT_FALSE(tri1 != tri2);

    t1.triple -> x = 69;

    EXPECT_FALSE(tri1 == tri2);
    EXPECT_TRUE(tri1 != tri2);
}

TEST(Triangle, isNeighbor_GetNeighbors) {
    // some triples
    Triple* t1 = new Triple(0, 0, 1);
    Triple* t2 = new Triple(0, 2, 2);
    Triple* t3 = new Triple(2, 2, 3);
    Triple* t4 = new Triple(2, 0, 4);
    Triple* t5 = new Triple(1, 1, 5);
    // their meshtriples
    MeshTriple* m1 = new MeshTriple(t1);
    MeshTriple* m2 = new MeshTriple(t2);
    MeshTriple* m3 = new MeshTriple(t3);
    MeshTriple* m4 = new MeshTriple(t4);
    MeshTriple* m5 = new MeshTriple(t5);
    // some triangles
    Triangle* tri1 = new Triangle(m1, m2, m5);
    Triangle* tri2 = new Triangle(m2, m3, m5);
    Triangle* tri3 = new Triangle(m3, m4, m5);
    Triangle* tri4 = new Triangle(m4, m1, m5);
    // tests
    EXPECT_TRUE(tri1 -> isNeighbor(tri4));
    EXPECT_TRUE(tri1 -> isNeighbor(tri2));
    EXPECT_TRUE(tri2 -> isNeighbor(tri1));
    EXPECT_TRUE(tri2 -> isNeighbor(tri3));
    EXPECT_TRUE(tri3 -> isNeighbor(tri2));
    EXPECT_TRUE(tri3 -> isNeighbor(tri4));
    EXPECT_TRUE(tri4 -> isNeighbor(tri3));
    EXPECT_TRUE(tri4 -> isNeighbor(tri1));
    EXPECT_FALSE(tri1 -> isNeighbor(tri3));
    EXPECT_FALSE(tri2 -> isNeighbor(tri4));
    EXPECT_FALSE(tri3 -> isNeighbor(tri1));
    EXPECT_FALSE(tri4 -> isNeighbor(tri2));
    EXPECT_EQ(tri1 -> getNeighbors().size(), 2);
    EXPECT_EQ(tri2 -> getNeighbors().size(), 2);
    EXPECT_EQ(tri3 -> getNeighbors().size(), 2);
    EXPECT_EQ(tri4 -> getNeighbors().size(), 2);
}

TEST(Triangle, getCircumCenter) {
    // right triangle - circumcenter on hypotenuse
    MeshTriple t1(new Triple(0, 0, 0));
    MeshTriple t2(new Triple(0, 1, 0));
    MeshTriple t3(new Triple(1, 0, 0));
    Triple c1(.5, .5, 0);

    Triangle tri1(&t1, &t2, &t3);

    EXPECT_TRUE(tri1.getCircumCenter() == c1);

    // acute triangle - circumcenter inside
    MeshTriple t4(new Triple(-1, 0, 0));
    MeshTriple t5(new Triple(1, 0, 0));
    MeshTriple t6(new Triple(0, 1.5, 0));
    Triple c2(0, 5.0/12.0, 0);

    Triangle tri2(&t4, &t5, &t6);

    EXPECT_TRUE(tri2.getCircumCenter() == c2);

    // obtuse triangle - circumcenter outside
    MeshTriple t7(new Triple(-1, 1, 0));
    MeshTriple t8(new Triple(0, 0, 0));
    MeshTriple t9(new Triple(1, 0, 0));
    Triple c3(.5, 1.5, 0);

    Triangle tri3(&t7, &t8, &t9);

    EXPECT_TRUE(tri3.getCircumCenter() == c3);
}

TEST(NdArray, generalTest) {
    unsigned long bounds[3] = {200, 100, 6};
    NdArray<float> array(3, bounds);

    unsigned long location[3] = {0, 2, 0};
    float value = 69.69;
    array.set(location, value);

    EXPECT_EQ(value, array.get(location));
    EXPECT_EQ(value, array.get(0, 2, 0));
    EXPECT_EQ(3, array.getNumDimensions());
    EXPECT_EQ(100, array.getDimensions()[1]);
    EXPECT_EQ(120000, array.size());
}

TEST(CoordinateList, toType_Cartesian) {
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

TEST(CoordinateList, toType_Spherical) {
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

TEST(CoordinateList, toType_Perspective) {
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

TEST(CoordinateList, testBucketSort) {
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

    test.bucketSort(c);

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

TEST(Mesh, testConstructorInvalidArgument) {
	EXPECT_THROW(Mesh(NULL), std::invalid_argument);
}

TEST(Mesh, testConstructorInvalidArgument2) {
	EXPECT_THROW(Mesh(new CoordinateList(CoordinateList::CARTESIAN, 0)), std::invalid_argument);
}

TEST(Mesh, determinant_2x2) {
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

TEST(Mesh, determinant_3x3) {
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

TEST(Mesh, determinant_4x4) {
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

    Triple t3(2, 2, 2);
    Triple t4(0, 0, 0);

    EXPECT_EQ(Mesh::dist2(t3, t4), 8);

    Triple t5(3, 4, 9);
    Triple t6(0, 0, 0);

    EXPECT_EQ(Mesh::dist2(t5, t6), 25);
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
    int result;
	for (int i = 0; i < 1; i++) {
        result += RUN_ALL_TESTS();
    }
    return result;
}
