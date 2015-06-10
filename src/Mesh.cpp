#pragma once

#include <vector>
#include "Triangle.cpp"
#include "MeshTriple.cpp"
#include "CoordinateList.cpp"

class Mesh {
	public:
		Mesh(CoordinateList inputPoints);
		vector<MeshTriple> meshPoints;
};
