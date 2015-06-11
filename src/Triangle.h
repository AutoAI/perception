#pragma once

class Triangle {
	public:
		Triangle(MeshTriple& v1, MeshTriple& v2 MeshTriple& v3);
		MeshTriple& points[3];
		bool operator==(const Triple &t1);
		bool operator!=(const Triple &t1);
};
