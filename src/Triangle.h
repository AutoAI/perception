#pragma once

#include "MeshTriple.h"

class Triangle {
	public:
		Triangle(MeshTriple *v0, MeshTriple *v1, MeshTriple *v2);
		MeshTriple* points[3];
		bool operator==(const Triangle &t1);
		bool operator!=(const Triangle &t1);
};
