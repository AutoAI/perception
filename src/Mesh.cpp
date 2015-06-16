// Mesh.cpp
// @author: Travis Vanderstad, Parth Mehrotra

// If you're looking for the documentation, its in Mesh.h

#pragma once

#define _USE_MATH_DEFINES

#include "ros/ros.h"

#include <stdexcept>
#include <math.h>
#include <stddef.h>
#include <iostream>

#include "CoordinateList.h"

#include "Mesh.h"
#include "MeshTriple.cpp"
#include "GlobalConstants.h"

using namespace std;

Mesh::Mesh(CoordinateList* cList){
	if (cList == NULL) {
		throw std::invalid_argument("CoordinateList must not be null");
	} else if(cList->getLength() < 3) {
		throw std::invalid_argument("CoordinateList must contain at least 3 points");
	}
	list = cList;
	// choose seed point, sort others accorinding to distance from seed
	MeshTriple* s = chooseSeed();
	(*list).sort(*(s -> triple));

	// construct initial convex hull (counter-clockwise)
	vector<MeshTriple*> hull(32);
	this->hull = hull;
	initHull(0, 1, 2);

	// sequentially insert points, adding edges from new point to 'visible' points on the convex hull
	for(unsigned long i = 0; i < list -> getLength(); i++){
		insertVert(list -> getPtr(i));
	}

	// iteratively 'flip' triangles until no more triangles need be flipped
	int maxIterations = 12;
	for(int i = 0; i < maxIterations; i++){
		int sumFlips = 0;
		for(int j = 0; j < tris.size(); j++){
			sumFlips += flip(tris[j]);
		}
		if(sumFlips == 0)
			break;
	}

	// set up the result - first, find max number of vert neighbors
	char maxNeighbors = 0;
	for(int i = 0; i < verts.size(); i++)
		maxNeighbors = (maxNeighbors > getNeighbors(verts[i]).size()) ? maxNeighbors : getNeighbors(verts[i]).size();

	// init the result
	unsigned long bounds[3] = {XRES, YRES, maxNeighbors};
	result = new NdArray<float>(3, bounds);

	// populate result
	for(int i = 0; i < XRES; i++)
		for(int j = 0; j < YRES; j++){
			MeshTriple temp = *(getNearest(*(new Triple(toImageX(i), toImageY(j), 0))));
			for(int k = 0; k < maxNeighbors; k++){
				unsigned long[3] setIndex = {i, j, k};
				if(k < temp.triangles.size())
					result.set(setIndex, temp.Triple -> z);
				else
					result.set(setIndex, -1);
			}
		}
}

MeshTriple* Mesh::chooseSeed(){
	// just give 'em any old seed
	return new MeshTriple((*list).getPtr(0));
}

void Mesh::initHull(unsigned long index0, unsigned long index1, unsigned long index2){
	// check angle 0 to see which way the verts should be ordered to make the triangle counter-clockwise
	float dTheta = atan2((*list).get(2).y-(*list).get(0).y, (*list).get(2).x-(*list).get(0).x)-atan2((*list).get(1).y-(*list).get(0).y, (*list).get(1).x-(*list).get(0).x);
	if(dTheta > 2*M_PI)
		dTheta -= M_PI;
	else if(dTheta < 0)
		dTheta += M_PI;
	bool increasing = dTheta < M_PI;

	// lets init that hull
	MeshTriple temp0((*list).getPtr(0));
	verts.push_back(&temp0);
	hull.push_back(&temp0);

	if(increasing){
		MeshTriple temp1((*list).getPtr(1));
		verts.push_back(&temp1);
		hull.push_back(&temp1);

		MeshTriple temp2((*list).getPtr(2));
		verts.push_back(&temp2);
		hull.push_back(&temp2);
	}else{
		MeshTriple temp2((*list).getPtr(2));
		verts.push_back(&temp2);
		hull.push_back(&temp2);

		MeshTriple temp1((*list).getPtr(1));
		verts.push_back(&temp1);
		hull.push_back(&temp1);
	}

	// make all them connections and ish
	Triangle *t = new Triangle(hull[0], hull[1], hull[2]);
	tris.push_back(t);
}

void Mesh::insertVert(Triple* v){
	// insert a meshtriple for the vert
	MeshTriple* t = new MeshTriple(v);
	// add any visible verts on the hull to a list. edges will be made to all of these
	// (remember where the most clockwise and most counter-clockwise verts are)
	vector<MeshTriple*> connectorTriples;
	int c;
	int cc;
	for(int i = 0; i < hull.size(); i++)
		if(isVisible(*(t -> triple), *(hull[i] -> triple))){
			connectorTriples.push_back(hull[i]);
			if((i == 0 && !isVisible(*(t -> triple), *(hull[hull.size()-1] -> triple))) || (i != 0 && !isVisible(*(t -> triple), *(hull[i-1] -> triple))))
				c = i;
			else if((i == hull.size()-1 && !isVisible(*(t -> triple), *(hull[0] -> triple))) || (i != 0 && !isVisible(*(t -> triple), *(hull[i+1] -> triple))))
				cc = i;
		}
	// make triangles, starting with the most clockwise pair of points and working counter-clockwise
	for(int i = c; (i+1)%hull.size() <= cc; i = (i+1)%hull.size())
		Triangle* temp = new Triangle(hull[i], hull[i+1], t);
	// trim the hull. of those verts visible to t, only the most clockwise and most counter-clockwise verts will remain
	int initialHullSize = hull.size();
	for(int i = (c+1)%initialHullSize; (i+1)%initialHullSize <= cc; cc = (cc-1+initialHullSize)%initialHullSize)
		hull.erase(hull.begin()+i);
	//insert the new point in-between the most clockwise and most counter-clockwise verts
	hull.insert(hull.begin()+cc, t);
}

void Mesh::removeTri(Triangle* t){
	// remove references to t from all its verts
	for(char i = 0; i < 3; i++)
		for(int j = 0; j < t -> points[i] -> triangles.size(); j++)
			if(t -> points[i] -> triangles[j] == t){
				t -> points[i] -> triangles.erase(t -> points[i] -> triangles.begin()+j);
				break;
			}
	// remove reference to t from this mesh
	for(unsigned long i = 0; i < tris.size(); i++)
		if(tris[i] == t){
			tris.erase(tris.begin()+i);
			return;
		}
}

// flips necessary edges of a triangle, returns number of edges flipped
int Mesh::flip(Triangle* t){
	int flipCount = 0;
	// for each neighbor
	vector<Triangle*> neighbors = getNeighbors(t);
	for(int i = 0; i < neighbors.size(); i++){
		Triangle* neighbor = neighbors[i];
	 	// find all points between the pair of triangles
	 	vector<MeshTriple*> allPoints;
	 	for(int i = 0; i < 3; i++){
	 		allPoints.push_back(t -> points[i]);
	 		allPoints.push_back(neighbor -> points[i]);
	 	}
	 	for(int i = 0; i < allPoints.size(); i++)
	 		for(int j = 0; j < i; j++)
	 			if(allPoints[i] == allPoints[j]){
	 				allPoints.erase(allPoints.begin()+i);
	 				i--;
	 			}
		// find the two points they have in common
	 	int i1, i2;
	 	bool n = false;
	 	for(int i = 0; i < allPoints.size(); i++)
	 		for(int j = 0; j < 3; j++)
	 			for(int k = 0; k < 3; k++)
	 				if(t -> points[j] == allPoints[i] && neighbor -> points[k] == allPoints[i]){
	 					if(n){
	 						i1 = i;
	 						n = true;
	 					}else
	 						i2 = i;
	 				}
	 	// find the two points they don't
	 	int i3, i4;
	 	for(int i = 0; i < allPoints.size(); i++)
	 		if(i != i1 && i != i2)
	 			if(n){
	 				i3 = i;
	 				n = false;
	 			}else
	 				i4 = i;
		// if the pair is not locally delaunay, flip it
	 	if(inCircumCirc(allPoints[i1] -> triple, allPoints[i2] -> triple, allPoints[i3] -> triple, allPoints[i4] -> triple)){
	 		removeTri(t);
	 		removeTri(neighbor);
	 		Triangle* new1 = new Triangle(allPoints[i1], allPoints[i3], allPoints[i4]);
	 		Triangle* new2 = new Triangle(allPoints[i2], allPoints[i3], allPoints[i4]);
	 		flipCount++;
	 	}
	}
	return flipCount;
}

vector<MeshTriple*> Mesh::getNeighbors(MeshTriple* t) {
	vector<Triangle*> neighborTriangles = t -> triangles;
	vector<MeshTriple*> result;
	// iterate over triangles
	for (int i = 0; i < neighborTriangles.size(); i++) {
		Triangle* tri = neighborTriangles[i];
		// iterate over each triangle's points
		for(int j = 0; j < 3; j++){
			bool good = true;
			// check if we already have that point
			for(int k = 0; k < result.size(); k++)
				if(result[k] == tri -> points[j]){
					good = false;
					break;
				}
			// if we don't, okay, let's add it
			if(good)
				result.push_back(tri -> points[j]);
		}
	}
}

vector<Triangle*> Mesh::getNeighbors(Triangle* t) {
	MeshTriple** points = t -> points;
	vector<Triangle*> result;
	// iterate over points
	for (int i = 0; i < 3; i++) {
		MeshTriple* mtrip = points[i];
		// iterate over each point's triangles
		for(int j = 0; j < mtrip -> triangles.size(); j++){
			bool good = true;
			// check if we already have that triangle
			for(int k = 0; k < result.size(); k++)
				if(result[k] == mtrip -> triangles[j]){
					good = false;
					break;
				}
			// if we don't, okay, let's add it
			if(good)
				result.push_back(mtrip -> triangles[j]);
		}
	}
	// if any triangles don't share 2 points with the first, they are not a neighbor
	for(int i = 0; i < result.size(); i++){
		Triangle* temp = result[i];
		int numPoints = 0;
		for(int j = 0; j < 3; j++)
			for(int k = 0; k < 3; k++)
				if(t -> points[j] == temp -> points[k])
					numPoints++;
		if(numPoints != 2){
			result.erase(result.begin()+i);
			i--;
		}
	}
	return result;
}

// is a point 'visible' from another? that is, does the line between them pass through the hull? this function answers these questions
bool Mesh::isVisible(Triple& a, Triple& d){
	bool result = true;
	if(testIntersect(*(hull[hull.size()-1] -> triple), *(hull[0] -> triple), a, d))
		return false;
	for(unsigned long i = 1; i < hull.size(); i++)
		if(testIntersect(*(hull[i-1] -> triple), *(hull[i] -> triple), a, d)){
			result = false;
			break;
		}
	return result;
}
	 
// helper function to find if line segments p1q1 and p2q2 intersect
// got this baby from www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
bool Mesh::testIntersect(Triple p1, Triple q1, Triple p2, Triple q2){
    // Find some orientations
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);
 
    // If you want to know what this does just go to the website where I got it they have more readable code
    return (o1 != o2 && o3 != o4)||(o1 == 0 && onSegment(p1, p2, q1))||(o2 == 0 && onSegment(p1, q2, q1))||(o3 == 0 && onSegment(p2, p1, q2))||(o4 == 0 && onSegment(p2, q1, q2));
}

// helper-helper function. dont worry about this one
bool Mesh::onSegment(Triple p, Triple q, Triple r){
    return q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) && q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y);
}

// dont worry about this one either
int Mesh::orientation(Triple p, Triple q, Triple r){
    int val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (val == 0)
    	return 0;
    return (val > 0)? 1: 2;
}

// formula I grabbed from https://www.cs.duke.edu/courses/fall08/cps230/Lectures/L-21.pdf
bool Mesh::inCircumCirc(Triple* t0, Triple* t1, Triple* t2, Triple* p){
	float** delta = new float*[4];
	for(int i = 0; i < 4; i++)
		delta[i] = new float[4];
	delta[0][0] = 1;
	delta[0][1] = 1;
	delta[0][2] = 1;
	delta[0][3] = 1;
	delta[1][0] = t0 -> x;
	delta[1][1] = t1 -> x;
	delta[1][2] = t2 -> x;
	delta[1][3] = p -> x;
	delta[2][0] = t0 -> y;
	delta[2][1] = t1 -> y;
	delta[2][2] = t2 -> y;
	delta[2][3] = p -> y;
	delta[3][0] = t0 -> x * t0 -> x + t0 -> y * t0 -> y;
	delta[3][1] = t1 -> x * t1 -> x + t1 -> y * t1 -> y;
	delta[3][2] = t2 -> x * t2 -> x + t2 -> y * t2 -> y;
	delta[3][3] = p -> x * p -> x + p -> y * p -> y;

	float** gamma = new float*[3];
	for(int i = 0; i < 4; i++)
		gamma[i] = new float[3];
	gamma[0][0] = 1;
	gamma[0][1] = 1;
	gamma[0][2] = 1;
	gamma[1][0] = t0 -> x;
	gamma[1][1] = t1 -> x;
	gamma[1][2] = t2 -> x;
	gamma[2][0] = t0 -> y;
	gamma[2][1] = t1 -> y;
	gamma[2][2] = t2 -> y;

	return det(delta, 4) * det(gamma, 3) < 0;
}

// computes a determinant using cofactor expansion (n!)
float Mesh::det(float** m, int n){
	if(n == 2)
		return m[0][0] * m[1][1] - m[0][1] * m[1][0];
	float*** sub = new float**[n];
	for(int i = 0; i < n; i++)
		sub[i] = new float*[n-1];
	for(int i = 0; i < n; i++)
		for(int j = 0; j < n-1; j++)
			sub[i][j] = new float[n-1];
	for(int i = 0; i < n; i++){
		int s = 0;
		for(int j = 0; j < n-1; j++){
			if(j == i)
				s++;
			for(int k = 1; k < n; k++)
				sub[i][j][k-1] = m[s][k];
			s++;
		}
	}

	float sum = 0;
	bool add = true;
	for(int i = 0; i < n; i++){
		if(add)
			sum += m[i][0] * det(sub[i], n-1);
		else
			sum -= m[i][0] * det(sub[i], n-1);
		add = !add;
	}
	return sum;
}

// returns a pointer to the nearest MeshTriple to a Triple
MeshTriple* Mesh::getNearest(Triple &t){
	MeshTriple* nearest = verts[0];
	for(int i = 1; i < verts.size(); i++)
		if(dist2(t, *(verts[i]->triple)) < dist2(t, *(nearest->triple)))
			nearest = verts[i];
	return nearest;
}

// returns the squared 2d distance between two Triples
float Mesh::dist2(Triple &a, Triple &b){
	return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y);
}

int Mesh::toPixelX(float x){
	return x * XRES/S + XRES/2;
}

int Mesh::toPixelY(float y){
	return y * XRES/S + YRES/2;
}

float Mesh::toImageX(int x){
	return (x-XRES/2)*S/XRES;
}

float Mesh::toImageY(int y){
	return (y-YRES/2)*S/XRES;
}