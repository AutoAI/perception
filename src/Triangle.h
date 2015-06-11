#pragma once

class Triangle {
	public:
		Triangle(MeshTriple *v0, MeshTriple *v1, MeshTriple *v2);
		MeshTriple* points[3];
};
