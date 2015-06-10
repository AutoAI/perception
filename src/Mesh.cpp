#pragma once

#include <vector>

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
	vector<MeshTriple> hull(10);
	this->hull = hull;

	// sequentially insert points, adding edges from new point to 'visible' points on the convex hull


	// iteratively 'flip' triangles until no more triangles need be flipped
}