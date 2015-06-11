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

void Mesh::initHull(size_t index0, size_t index1, size_t index2){
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

// vector<Triple> Mesh::getNeighboringTriples(MeshTriple t) {
// 	vector<Triple> neighborTriangles = t.triangles;
// 	vector<triple> neighborPoints;
// 	for (int i = 0; i < neighborTriangles.size(); i++) {
		
// 	}
// }

void Mesh::insertVert(Triple t){
	
}

// is a point 'visible' from another? does the line between them pass through the hull? this function answers these questions
bool Mesh::isVisible(Triple& a, Triple& d){
	bool result = true;
	if(testIntersect(*(hull[hull.size()-1] -> triple), *(hull[0] -> triple), a, d))
		return false;
	for(size_t i = 1; i < hull.size(); i++)
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
