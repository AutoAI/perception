#pragma once

#include "Triangle.cpp"
#include "Triple.cpp"

class MeshTriple {
	public:
		vector<Triangle*> triangles;
		Triple* triple;
};
