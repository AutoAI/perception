#pragma once

#include <vector>

#include "triple.h"

class Triangle; // forward declaration. Triangle references MeshTriple, so we can't just #include (reinstate after removing superh.h)

class MeshTriple {
	public:
		MeshTriple(Triple* triple);
		std::vector<Triangle*> triangles;
		Triple* triple;
		bool operator==(const MeshTriple &t1);
		bool operator!=(const MeshTriple &t1);
};
