#pragma once

class Mesh {
	public:
		Mesh(CoordinateList c);
		vector<MeshTriple> getNeighboringTriples(MeshTriple t);

	private:
		vector<Triple> hull;
		vector<MeshTriple> verts;
		vector<Triangle> tris;
};
