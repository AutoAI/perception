#pragma once

// class MeshTriple; // forward declaration. MeshTriple references Triangle, so we can't just #include

class Triangle {
	public:
		Triangle(MeshTriple *v0, MeshTriple *v1, MeshTriple *v2);
		MeshTriple* points[3];
};
