#pragma once

#include <vector>
#include "Triple.cpp"

class Triangle {
	public:
		Triple points[3];
		vector<Triangle> neighborTriangles();
};
