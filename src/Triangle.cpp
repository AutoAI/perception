#pragma once

#include "Triangle.h"

Triangle::Triangle(MeshTriple *v0, MeshTriple *v1, MeshTriple *v2) {
	points[0] = v0;
	points[1] = v1;
	points[2] = v2;
	v0 -> triangles.push_back(this);
	v1 -> triangles.push_back(this);
	v2 -> triangles.push_back(this);
}