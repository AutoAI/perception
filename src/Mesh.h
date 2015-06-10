#pragma once

class Mesh {
	public:
		Mesh(CoordinateList c);

	private:
		vector<MeshTriple> hull;
};
