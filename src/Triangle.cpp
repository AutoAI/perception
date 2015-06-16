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

bool Triangle::operator!=(const Triangle &tri1) {
	return !(*this == tri1);
}

bool Triangle::operator==(const Triangle &tri1) {
	bool index0 = false;
	bool index1 = false;
	bool index2 = false;
	for (int ours = 0; ours < 3; ours++) {
		if (points[ours] == tri1.points[0]) {
			index0 = true;
		} else if (points[ours] == tri1.points[1]) {
			index1 = true;
		} else if (points[ours] == tri1.points[2]) {
			index2 = true;
		}
	}

	return index0 && index1 && index2;
}
