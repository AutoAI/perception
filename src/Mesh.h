#pragma once

class Mesh {
	public:
		Mesh(CoordinateList c);

	private:
		vector<Triple> hull;
		vector<MeshTriple> verts;
		vector<Triangle> tris;
};
