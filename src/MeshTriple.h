#pragma once

#include <vector>

#include "Triple.h"

// class Triangle; // forward declaration. Triangle references MeshTriple, so we can't just #include

class MeshTriple {
	public:
		vector<Triangle*> triangles;
		Triple* triple;
};
