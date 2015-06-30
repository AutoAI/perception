/*
 * Copyright (c) 2015, DriveAI
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of DriveAI nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
* Mesh.h
* @author: Travis Vanderstad, Parth Mehrotra
*
* Mesh represents a triangular mesh. Stores a list of triangles (see Triangle.h) and vertices (see MeshTriple.h)
* triangles store references to their verts and verts to their triangels
*/

#pragma once

#include <vector>

#include "triple.h"
#include "triangle.h"
#include "mesh_triple.h"
#include "nd_array.h"

#define start_size 5;

class Mesh {
public:

	/**
	* makes the triangulation. boom.
	*
	* @param cList pointer to a CoordinateList
	* @return
	*/
	Mesh(CoordinateList* cList);

	/**
	* result of the triangulation: a 2d array full of interpolation ranges
	*/
	NdArray<float>* result;

	/**
	* converts a u-coordinate to a pixel x-value
	*
	* @param x u-coordinate to be converted
	* @return pixel x-value equivalent of parameter
	*/
	static int toPixelX(float x);

	static int toPixelY(float y);

	static float toImageX(int x);

	static float toImageY(int y);

private:
	/**
	* hull for sweeping (the sweep-hull. s-hull. more on this at www.s-hull.org/)
	*/
	std::vector<MeshTriple*> hull;

	/**
	* all the verts in the triangulation
	*/
	std::vector<MeshTriple*> verts;

	/**
	* all the triangles in the triangulation
	*/
	std::vector<Triangle*> tris;

	/**
	* all coordinates that are to be added to the triangulation
	*/
	CoordinateList* list;

	/**
	* populated with per-pixel data from all neighbors of the triangulation's vert that is closest to each pixel
	*/
	NdArray<float>* data;

	/**
	* pick a seed point for the triangulation
	*
	* @return A pointer to a MeshTriple to begin triangulation
	*/
	MeshTriple* chooseSeed();

	/**
	* generate the initial hull
	*
	* @param index0 index of the first Triple
	* @param index1 index of the second Triple
	* @param index2 index of the third Triple
	*/
	void initHull(size_t index0, size_t index1, size_t index2);

	/**
	* add a vertex to the triangulation. this is called for each entry in list
	*
	* @param v pointer to the vertex being inserted
	*/
	void insertVert(Triple* v);

	/**
	* determines if one triple is "visible" to another through the hull (does a line between them intersect the hull lines)
	*
	* @param a first Triple
	* @param d second Triple
	*/
	bool isVisible(Triple& a, Triple& d);

	/**
	* tests if the lines formed by two pairs of Triples intersects
	*
	* @param p1 first point on line 1
	* @param q1 first point on line 2
	* @param p2 second point on line 1
	* @param q2 second point on line 2
	*/
	static bool testIntersect(Triple p1, Triple q1, Triple p2, Triple q2);

	/**
	* not really sure what this does. helps testIntersect
	*/
	static int orientation(Triple p, Triple q, Triple r);

	/**
	* tests whether a point is on the line defined by two other points. not sure which is which. helps testIntersect
	*/
	static bool onSegment(Triple p, Triple q, Triple r);

	/**
	* tests whether a hypothetical triangle spanning the parameters is 'good' (i.e. it contains no points in the mesh)
	*/
	bool goodTri(Triple* t0, Triple* t1, Triple* t2);

	/**
	* tests whether the point p lies in the triangle spanned by t0, t1, and t2
	*/
	static bool inTri(Triple* t0, Triple* t1, Triple* t2, Triple* p);

	/**
	* no idea what this thing does. helps inTri
	*/
	static float sign (Triple* p1, Triple* p2, Triple* p3);

	/**
	* flips at most one edge of a triangle if there are any non-locally-delaunay edges. returns the number of edges flipped (0 or 1)
	*/
	int flip(Triangle* t);

	/**
	* get a list of a triangle's neighboring triangles
	*
	* @param t triangle to find neighbors from
	* @return vector of neighboring Triangles
	*/
	static std::vector<Triangle*> getNeighbors(Triangle* t);

	/**
	* tests whethera point p is in the circumcircle of the triangle spanned by t0, t1, and t2
	*/
	static bool inCircumCirc(Triple* t0, Triple* t1, Triple* t2, Triple* p);

	static float det(float** in_matrix, int n);

	/**
	* removes a triangle from the mesh's list of triangles as well as the meshtriples that reference it (its corners)
	*
	* @param t pointer to triangle being removed
	*/
	void removeTri(Triangle* t);

	/**
	* get a list of a vertex's neighboring vertices
	*
	* @param t point to find neighbors from
	* @return vector of neighboring points
	*/
	static std::vector<MeshTriple*> getNeighbors(MeshTriple* t);

	/**
	* returns a reference to the nearest point in the mesh to the parameter
	*/
	MeshTriple* getNearest(Triple &t);

	/**
	* fills in the voronoi region of the MeshTriple (the region of all points closer to this MeshTriple than any other)
	*/
	void fillRegion(MeshTriple* m);

	/**
	* fills in a triangle of the data NdArray with the z-values of a MeshTriple and its neighbors
	*/
	void fillTri(Triple &a, Triple &b, Triple &c, MeshTriple* m);

	/*
	* determines whether an x,y pixel is in the triangle spanned by 3 triples
	*/
	bool inTri(Triple &t0, Triple &t1, Triple &t2, int x, int y);

	// helper function for inTri above
	float sign (int x, int y, Triple &p2, Triple &p3);

	/**
	* returns the squared distance between two points. this is slightly easier than the distance because you dont have to take square roots,
	* and because distance squared is strictly increasing with distance, if you're going to compare distances you might as well compare squared distances
	*/
	static float dist2(Triple &a, Triple &b);
};
