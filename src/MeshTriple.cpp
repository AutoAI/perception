#pragma once 

#include <vector>
#include "Triangle.cpp"

class MeshTriple : public Triple {
	public:
		vector<Triangle> neighborTriangles;
		vector<MeshTriple> neighborPoints;
};
