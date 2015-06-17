// Mesh.h
// @author: Travis Vanderstad, Parth Mehrotra

// Mesh represents a triangular mesh. Stores a list of triangles (see Triangle.h) and vertices (see MeshTriple.h)
// triangles store references to their verts and verts to their triangels

#pragma once

#include <vector>

#include "Triple.h"
#include "Triangle.h"
#include "MeshTriple.h"
#include "NdArray.h"

#define start_size 5;

class Mesh {
	public:
		// makes the triangulation. boom.
		Mesh(CoordinateList* cList);

		// result of the triangulation: an image full of interpolation ranges
		NdArray<float>* result;

	private:
		// hull for sweeping (the sweep-hull. s-hull. more on this at www.s-hull.org/)
		vector<MeshTriple*> hull;

		// all the verts in the triangulation
		vector<MeshTriple*> verts;

		// all the triangles in the triangulation
		vector<Triangle*> tris;

		// all coordinates that are
		CoordinateList* list;

		// pick a seed point for the triangulation
		MeshTriple* chooseSeed();

		// generate the initial hull
		void initHull(size_t index0, size_t index1, size_t index2);

		// add a vertex to the triangulation. this is called for each entry in list
		void insertVert(Triple* v);

		// do what needs to be done to remove a triangle
		void removeTri(Triangle* t);

		// get a list of a vertex's neighboring vertices
		vector<MeshTriple*> getNeighbors(MeshTriple* t);

		// get a list of a triangle's neighboring triangles
		vector<Triangle*> getNeighbors(Triangle* t);

		// determines if one triple is "visible" to another through the hull (does a line between them intersect the hull lines)
		bool isVisible(Triple& a, Triple& d);

		// helper function for above function
		static bool testIntersect(Triple p1, Triple q1, Triple p2, Triple q2);

		// helper-helper function for above function
		static bool onSegment(Triple p, Triple q, Triple r);

		// helper-helper function for function above above function
		static int orientation(Triple p, Triple q, Triple r);

		static bool inCircumCirc(Triple* t0, Triple* t1, Triple* t2, Triple* p);
	
		static float det(float** in_matrix, int n);

		int flip(Triangle* t);

		// populated with data from all neighbors of the triangulation's nearest vert
		NdArray<float>* data;

		MeshTriple* getNearest(Triple &t);

		static float dist2(Triple &a, Triple &b);

		static int toPixelX(float x);

		static int toPixelY(float y);

		static float toImageX(int x);

		static float toImageY(int y);
};
