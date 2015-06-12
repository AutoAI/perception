// Mesh.cpp
// @author: Travis Vanderstad, Parth Mehrotra

// If you're looking for the documentation, its in Mesh.h

#pragma once

#define _USE_MATH_DEFINES

#include <math.h>

#include "CoordinateList.h"

#include "Mesh.h"

using namespace std;

Mesh::Mesh(CoordinateList* cList){
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

}

MeshTriple* Mesh::chooseSeed(){
	// just give 'em any old seed
	MeshTriple* returned = new MeshTriple();
	returned -> triple -> x = (*list).get(0).x;
	returned -> triple -> y = (*list).get(0).y;
	returned -> triple -> z = (*list).get(0).z;
	return returned;
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

	MeshTriple temp0;
	temp0.triple = (*list).getPtr(0);
	verts.push_back(&temp0);
	hull.push_back(&temp0);

	if(increasing){
		MeshTriple temp1;
		temp1.triple = (*list).getPtr(1);
		verts.push_back(&temp1);
		hull.push_back(&temp1);

		MeshTriple temp2;
		temp2.triple = (*list).getPtr(2);
		verts.push_back(&temp2);
		hull.push_back(&temp2);
	}else{
		MeshTriple temp2;
		temp2.triple = (*list).getPtr(2);
		verts.push_back(&temp2);
		hull.push_back(&temp2);

		MeshTriple temp1;
		temp1.triple = (*list).getPtr(1);
		verts.push_back(&temp1);
		hull.push_back(&temp1);
	}

	// make all them connections and ish
	Triangle *t = new Triangle(hull[0], hull[1], hull[2]);
	hull[0] -> triangles.push_back(t);
	hull[1] -> triangles.push_back(t);
	hull[2] -> triangles.push_back(t);
	tris.push_back(t);
}

void Mesh::insertVert(Triple* v){
	// insert a meshtriple for the vert
	MeshTriple* t = new MeshTriple();
	t -> triple = v;
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
	for(int i = c; (i+1)%hull.size() <= cc; i = (i+1)%hull.size()){
		Triangle* temp = new Triangle(hull[i], hull[i+1], t);
		hull[i] -> triangles.push_back(temp);
		hull[i+1] -> triangles.push_back(temp);
		t -> triangles.push_back(temp);
	}
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

 void Mesh::flipTriangle(Triangle* t){
 	
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
}

// is a point 'visible' from another? does the line between them pass through the hull? this function answers these questions
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
	return det(1, t0->x, t0->y, t0->x * t0->x + t0->y * t0->y, 1, t1->x, t1->y, t1->x * t1->x + t1->y * t1->y, 1, t2->x, t2->y, t2->x * t2->x + t2->y * t2->y, 1, p->x, p->y, p->x * p->x + p->y * p->y) * det(1, t0->x, t0->y, 1, t1->x, t1->y, 1, t2->x, t2->y) < 0;
}

// determinant of a 4x4 matrix; first row is a b c d; first column is a e i m
float Mesh::det(float a, float b, float c, float d, float e, float f, float g, float h, float i, float j, float k, float l, float m, float n, float o, float p){
	return a*det(f, g, h, j, k, l, n, o, p) + b*det(g, h, e, k, l, i, o, p, m) + c*det(h, e, f, l, i, j, p, m, n) + d*det(e, f, g, i, j, k, m, n, o);
}

// determinant of a 3x3 matrix; first row is a b c; first column is a d g
float Mesh::det(float a, float b, float c, float d, float e, float f, float g, float h, float i){
	return a*det(e, f, h, i) + b*det(f, d, i, g) + c*det(d, e, g, h);
}

// determinant of a 2x2 matrix; first row is a b; first column is a c
float Mesh::det(float a, float b, float c, float d){
	return a*d - b*c;
}