
#pragma once
#ifndef SRC_MESH_TRIPLE_H_
#define SRC_MESH_TRIPLE_H_

#include <vector>

#include "triple.h"

// forward declaration. Triangle references MeshTriple, so we can't just
// #include (reinstate after removing superh.h)
class Triangle;

class MeshTriple {
 public:
	explicit MeshTriple(Triple* triple);
	std::vector<Triangle*> triangles;
	Triple* triple;
	bool operator==(const MeshTriple &t1);
	bool operator!=(const MeshTriple &t1);
};

#endif  // SRC_MESH_TRIPLE_H_

