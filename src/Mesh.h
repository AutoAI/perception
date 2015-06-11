// Mesh.h
// @author: Travis Vanderstad, Parth Mehrotra

// Mesh represents a triangular mesh. Stores a list of triangles (see Triangle.h) and vertices (see MeshTriple.h)
// triangles store references to their verts and verts to their triangels

#pragma once

#include <vector>

#include "Triple.h"
#include "Triangle.h"
#include "MeshTriple.h"

#define start_size 5;

class Mesh {
	public:
		Mesh(CoordinateList* cList);
		vector<MeshTriple> getNeighboringTriples(MeshTriple t);

	private:
		vector<MeshTriple*> hull;
		vector<MeshTriple*> verts;
		vector<Triangle*> tris;
		CoordinateList* list;

		MeshTriple* chooseSeed();
		void initHull(size_t index0, size_t index1, size_t index2);
		//vector<Triple> getNeighboringTriples(MeshTriple t);
		void insertVert(Triple t);
		bool isVisible(Triple& a, Triple& d);
		bool testIntersect(Triple p1, Triple q1, Triple p2, Triple q2);
		bool onSegment(Triple p, Triple q, Triple r);
		int orientation(Triple p, Triple q, Triple r);
};
