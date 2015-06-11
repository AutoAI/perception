#pragma once

#include "Triangle.h"
#include "Triple.cpp"

bool Triangle::operator!=(const Triangle &tri1) {
	return !(this == tri1);
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
			index 2 = true;
		}
	}

	return index0 && index1 && index2;
}
