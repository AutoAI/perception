#pragma once

class Triangle {
	public:
		MeshTriple points[3];
		bool operator==(const Triple &t1);
		bool operator!=(const Triple &t1);
};
