#pragma once

#include <vector>
#include <math.h>

#include "Triangle.cpp"
#include "MeshTriple.cpp"
#include "CoordinateList.cpp"

#include "Mesh.h"

#define start_size 5;

using namespace std;

Mesh::Mesh(CoordinateList list){
	// choose seed point, sort others accorinding to distance from seed
	Triple s = chooseSeed();
	list.sort(s);

	// construct initial convex hull (counter-clockwise)
	vector<MeshTriple> hull(32);
	this->hull = hull;
	Triangle t = initHull(0, 1, 2);

	// sequentially insert points, adding edges from new point to 'visible' points on the convex hull


	// iteratively 'flip' triangles until no more triangles need be flipped

}

Triple Mesh::chooseSeed(){
	// just give 'em any old seed
	return list[0];
}

Triangle Mesh::initHull(size_t index0, size_t index1, size_t index2){
	// check angle 0 to see which way the verts should be ordered to make the triangle counter-clockwise
	float dTheta = atan2(list[2].y-list[0].y, list[2].x-list[0].x)-atan2(list[1].y-list[0].y, list[1].x-list[0].x);
	if(dTheta > 2*PI)
		dTheta -= PI;
	else if(dTheta < 0)
		dTheta += PI;
	bool increasing = dTheta < PI;

	// lets init that hull
	hull.push_back(list[0]);
	if(increasing){
		hull.push_back(list[1]);
		hull.push_back(list[2]);
	}else{
		hull.push_back(list[2]);
		hull.push_back(list[1]);
	}
	Triangle result;
	result.points[0] = hull()
}

// is a point 'visible' from another? (does the line between them pass through the hull?) this function answers that question
bool Mesh::isVisible(Triple origin, Triple destination){
	bool result = true;
	if(testIntersect(hull[hull.size()-1], hull[0], origin, destination))
		return false;
	for(size_t i = 1; i < hull.size(); i++)
		if(testIntersect(hull[i-1], hull[i], origin, destination)){
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
    if (val == 0) return 0;
    return (val > 0)? 1: 2;
}

vector<Triple> Mesh::getNeighboringTriples(MeshTriple t) {
	vector<Triple> neighborTriangles = t.triangles;
	vector<triple> neighborPoints;
	for (int i = 0; i < neighborTriangles.size(); i++) {
		
	}
}
