
#pragma once
#ifndef SRC_TRIANGLE_H_
#define SRC_TRIANGLE_H_

#include "mesh_triple.h"

class Triangle {
	public:
		Triangle(MeshTriple *v0, MeshTriple *v1, MeshTriple *v2);
		MeshTriple* points[3];
		bool operator==(const Triangle &t1);
		bool operator!=(const Triangle &t1);
};

#endif  // SRC_TRIANGLE_H_

