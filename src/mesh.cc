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
 * 3. Neither the name of whiskey-foxtrot nor the names of its
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

// Mesh.cpp
// @author: Travis Vanderstad, Parth Mehrotra

// If you're looking for the documentation, its in Mesh.h

#define _USE_MATH_DEFINES

#include <math.h>
#include <stddef.h>

#include <algorithm>
#include <iostream>
#include <stdexcept>

#include "ros/ros.h"

#include "coordinate_list.h"
#include "mesh.h"
#include "mesh_triple.cc"
#include "global_constants.h"

using namespace std;

Mesh::Mesh(CoordinateList* cList) {
	if (cList == NULL) {
		throw std::invalid_argument("CoordinateList must not be null");
	} else if (cList->getLength() < 3) {
		throw std::invalid_argument("CoordinateList must contain at least 3 points");
	}
	list = cList;
	// choose seed point, sort others accorinding to distance from seed
	MeshTriple* s = chooseSeed();
	(*list).sort(*(s -> triple));

	// construct initial convex hull (counter-clockwise)
	vector<MeshTriple*> hull;
	this->hull = hull;
	initHull(0, 1, 2);

	// sequentially insert points, adding edges from new point to 'visible'
	// points on the convex hull
	for(uint64_t i = 3; i < list -> getLength(); i++) {
		insertVert(list -> getPtr(i));
	}

	// iteratively 'flip' triangles until no more triangles need be flipped
	int maxIterations = 12;
	int sumFlips;
	for(int i = 0; sumFlips != 0; i++) {
		sumFlips = 0;
		for(int j = 0; j < tris.size(); j++) {
			sumFlips += flip(tris[j]);
		}
		if(i == maxIterations - 1) {
			break;
		}
	}

	// print triangles (temporary)
	Triple *t0, *t1, *t2;
	for(int i = 0; i < tris.size(); i++) {
		t0 = tris[i] -> points[0] -> triple;
		t1 = tris[i] -> points[1] -> triple;
		t2 = tris[i] -> points[2] -> triple;
	}

	// set up the data array - first, find max number of vert neighbors
	char maxNeighbors = 0;
	for(int i = 0; i < verts.size(); i++) {
		maxNeighbors = (maxNeighbors > getNeighbors(verts[i]).size())
			? maxNeighbors
			: getNeighbors(verts[i]).size();
	}

	// init the data
	uint64_t bounds[3] =
		{CameraConstants::XRES, CameraConstants::YRES, maxNeighbors+1};
	data = new NdArray<float>(3, bounds);

	// populate data
	for(int i = 0; i < CameraConstants::XRES; i++) {
		for(int j = 0; j < CameraConstants::YRES; j++) {
			MeshTriple* temp = getNearest(*(new Triple(toImageX(i), toImageY(j), 0)));
			uint64_t setIndex[3] = {i, j, 0};
			data -> set(setIndex, temp -> triple -> z);
			for(int k = 1; k < maxNeighbors+1; k++) {
				uint64_t setIndexTemp[3] = {i, j, k};
				if(k-1 < getNeighbors(temp).size()) {
					data -> set(setIndexTemp, getNeighbors(temp)[k-1] -> triple -> z);
				} else {
					data -> set(setIndexTemp, -1);
				}
			}
		}
	}

	// init result
	uint64_t bounds2[3] = {CameraConstants::XRES, CameraConstants::YRES, 2};
	result = new NdArray<float>(3, bounds2);

	// calculate result
	for(int i = 0; i < CameraConstants::XRES; i++) {
		for(int j = 0; j < CameraConstants::YRES; j++) {
			float max = -1;
			float min = CameraConstants::K;
			// TODO should rename k
			for(int k = 0; k < maxNeighbors+1; k++) {
				uint64_t getIndex[3] = {i, j, k};
				if(data -> get(getIndex) == -1) {
					break;
				}
				min = (min < data -> get(getIndex)) ? min : data -> get(getIndex);
				max = (max > data -> get(getIndex)) ? max : data -> get(getIndex);
			}
			uint64_t setIndex[3] = {i, j, 0};
			result -> set(setIndex, min);
			uint64_t setIndex2[3] = {i, j, 1};
			result -> set(setIndex2, max);
		}
	}

	// // print the result
	// for(int i = 0; i < CameraConstants::XRES; i++) {
	// 	for(int j = 0; j < CameraConstants::YRES; j++) {
	// 		uint64_t getIndex[3] = {i, j, 0};
	// 		cout << result -> get(getIndex) << endl;
	// 		uint64_t getIndex2[3] = {i, j, 1};
	// 		cout << result -> get(getIndex2) << endl;
	// 	}
	// }
}

MeshTriple* Mesh::chooseSeed() {
	// just give 'em any old seed
	return new MeshTriple((*list).getPtr(0));
}

void Mesh::initHull(uint64_t index0, uint64_t index1, uint64_t index2) {
	// check angle 0 to see which way the verts should be ordered to make
	// the triangle counter-clockwise
	float dTheta = atan2(
		(*list).get(2).y - (*list).get(0).y,
		(*list).get(2).x - (*list).get(0).x) -
			atan2((*list).get(1).y - (*list).get(0).y,
		(*list).get(1).x - (*list).get(0).x);
	if(dTheta > 2*M_PI) {
		dTheta -= M_PI;
	} else if (dTheta < 0) {
		dTheta += M_PI;
	}
	bool increasing = dTheta < M_PI;

	// lets init that hull
	MeshTriple* temp0 = new MeshTriple((*list).getPtr(0));
	verts.push_back(temp0);
	hull.push_back(temp0);

	if(increasing) {
		MeshTriple* temp1 = new MeshTriple((*list).getPtr(1));
		verts.push_back(temp1);
		hull.push_back(temp1);

		MeshTriple* temp2 = new MeshTriple((*list).getPtr(2));
		verts.push_back(temp2);
		hull.push_back(temp2);
	} else {
		MeshTriple* temp2 = new MeshTriple((*list).getPtr(2));
		verts.push_back(temp2);
		hull.push_back(temp2);

		MeshTriple* temp1 = new MeshTriple((*list).getPtr(1));
		verts.push_back(temp1);
		hull.push_back(temp1);
	}

	// make all them connections and ish
	Triangle* t = new Triangle(hull[0], hull[1], hull[2]);
	tris.push_back(t);
}

void Mesh::insertVert(Triple* v) {
	// insert a meshtriple for the vert
	MeshTriple* t = new MeshTriple(v);

	// add any visible verts on the hull to a list.
	// edges will be made to all of these
	// (remember where the most clockwise and most counter-clockwise verts are)
	vector<MeshTriple*> connectorTriples;
	bool visibilities[hull.size()];
	int c;
	int cc;
	for(int i = 0; i < hull.size(); i++) {
		if(isVisible(*(t -> triple), *(hull[i] -> triple))) {
			connectorTriples.push_back(hull[i]);
			visibilities[i] = true;
		} else {
			visibilities[i] = false;
		}
	}

	for(int i = 0; i < hull.size(); i++) {
		if(visibilities[i] && !visibilities[(i+1)%hull.size()]) {
			cc = i;
		} else if (visibilities[(i+1)%hull.size()] && !visibilities[i]) {
			c = (i+1)%hull.size();
		}
	}

	// make triangles, starting with the most clockwise pair of points
	// and working counter-clockwise
	for(int i = c; i != cc; i = (i+1)%hull.size()) {
		Triangle* temp = new Triangle(hull[i], hull[(i+1)%hull.size()], t);
		tris.push_back(temp);
	}

	// trim the hull. of those verts visible to t, only the most clockwise
	// and most counter-clockwise verts will remain
	int i = cc - 1;
	while(true) {
		if(i == -1) {
			i = hull.size() - 1;
		}
		if(i == c) {
			break;
		}
		hull.erase(hull.begin()+i);
		if(i < c) {
			c--;
		}
		if(i < cc) {
			cc--;
		}
		i--;
	}

	// insert the new point in-between the most clockwise and most
	// counter-clockwise verts
	hull.insert(hull.begin()+cc, t);
}

void Mesh::removeTri(Triangle* t) {
	// remove references to t from all its verts
	for(char i = 0; i < 3; i++) {
		for(int j = 0; j < t -> points[i] -> triangles.size(); j++) {
			if(t -> points[i] -> triangles[j] == t) {
				t -> points[i] -> triangles.erase(t -> points[i] -> triangles.begin()+j);
				break;
			}
		}
	}
	// remove reference to t from this mesh
	for(uint64_t i = 0; i < tris.size(); i++) {
		if(tris[i] == t) {
			tris.erase(tris.begin()+i);
			return;
		}
	}
}

// flips necessary edges of a triangle, returns number of edges flipped
int Mesh::flip(Triangle* t) {
	// for each neighbor...
	vector<Triangle*> neighbors = getNeighbors(t);
	for(int i = 0; i < neighbors.size(); i++) {
		Triangle* neighbor = neighbors[i];

		// find all points between the pair of triangles
		vector<MeshTriple*> allPoints;
		for(int j = 0; j < 3; j++) {
			allPoints.push_back(t -> points[j]);
			allPoints.push_back(neighbor -> points[j]);
		}
		for(int j = 0; j < allPoints.size(); j++) {
			for(int k = 0; k < j; k++) {
				if(allPoints[j] == allPoints[k]) {
					allPoints.erase(allPoints.begin()+j);
					j--;
				}
			}
		}

		// find the two points they have in common
		int i1 = -1;
		int i2 = -1;
		bool found[4] = {false, false, false, false};
		for(int j = 0; j < allPoints.size(); j++) {
			for(int k = 0; k < 3; k++) {
				if(t -> points[k] == allPoints[j]) {
					found[j] = true;
				}
			}
		}

		for(int j = 0; j < allPoints.size(); j++) {
			for(int k = 0; k < 3; k++) {
				if(found[j] && neighbor -> points[k] == allPoints[j]) {
					if(i1 == -1) {
						i1 = j;
					} else {
						i2 = j;
					}
				}
			}
		}

		// find the two points they don't have in common
		int i3 = -1;
		int i4 = -1;
		bool n = true;
		for(int j = 0; j < allPoints.size(); j++) {
			if(j != i1 && j != i2) {
				if(n) {
					i3 = j;
					n = false;
				} else {
					i4 = j;
				}
			}
		}

		// if the pair is not locally delaunay, flip it and stop
		// (we'll come back for the rest in the next iteration)
		if(inCircumCirc(allPoints[i1] -> triple,
				allPoints[i2] -> triple,
				allPoints[i3] -> triple,
				allPoints[i4] -> triple)) {
			removeTri(t);
			removeTri(neighbor);
			Triangle* new1 = new Triangle(allPoints[i1], allPoints[i3], allPoints[i4]);
			Triangle* new2 = new Triangle(allPoints[i2], allPoints[i3], allPoints[i4]);
			tris.push_back(new1);
			tris.push_back(new2);

			return 1;
		}
	}
	return 0;
}

vector<MeshTriple*> Mesh::getNeighbors(MeshTriple* t) {
	vector<Triangle*> neighborTriangles = t -> triangles;
	vector<MeshTriple*> result;
	// iterate over triangles
	for (int i = 0; i < neighborTriangles.size(); i++) {
		Triangle* tri = neighborTriangles[i];
		// iterate over each triangle's points
		for(int j = 0; j < 3; j++) {
			bool good = true;
			// check if we already have that point
			for(int k = 0; k < result.size(); k++) {
				if(result[k] == tri -> points[j]) {
					good = false;
					break;
				}
			}

			// if we don't, okay, let's add it
			if(good) {
				result.push_back(tri -> points[j]);
			}
		}
	}
	// remove this meshtriple from the result
	for(int i = 0; i < result.size(); i++) {
		if(result[i] == t) {
			result.erase(result.begin()+i);
			break;
		}
	}
	return result;
}

vector<Triangle*> Mesh::getNeighbors(Triangle* t) {
	MeshTriple** points = t -> points;
	vector<Triangle*> data;
	// iterate over points
	for (int i = 0; i < 3; i++) {
		MeshTriple* mtrip = points[i];
		// iterate over each point's triangles
		for(int j = 0; j < mtrip -> triangles.size(); j++) {
			bool good = true;
			// check if we already have that triangle
			for(int k = 0; k < data.size(); k++) {
				if(data[k] == mtrip -> triangles[j]) {
					good = false;
					break;
				}
			}

			// if we don't, okay, let's add it
			if(good) {
				data.push_back(mtrip -> triangles[j]);
			}
		}
	}
	// if any triangles don't share 2 points with the first,
	// they are not a neighbor
	for(int i = 0; i < data.size(); i++) {
		Triangle* temp = data[i];
		int numPoints = 0;
		for(int j = 0; j < 3; j++) {
			for(int k = 0; k < 3; k++) {
				if(t -> points[j] == temp -> points[k]) {
					numPoints++;
				}
			}
		}
		if(numPoints != 2) {
			data.erase(data.begin()+i);
			i--;
		}
	}
	return data;
}

// is a point 'visible' from another?
// that is, does the line between them pass through the hull?
// this function answers these questions
bool Mesh::isVisible(Triple& a, Triple& d) {
	if(!((a == *(hull[hull.size()-1] -> triple)) ||
			((d == *(hull[hull.size()-1] -> triple))) ||
			(a == *(hull[0] -> triple)) ||
			((d == *(hull[0] -> triple)))) &&
			testIntersect(*(hull[hull.size()-1] -> triple),
				*(hull[0] -> triple), a, d)) {
		return false;
	}

	for(uint64_t i = 1; i < hull.size(); i++) {
		if(!((a == *(hull[i-1] -> triple)) ||
				((d == *(hull[i-1] -> triple))) ||
				(a == *(hull[i] -> triple)) ||
				((d == *(hull[i] -> triple)))) &&
				testIntersect(*(hull[i-1] -> triple),
					*(hull[i] -> triple), a, d)) {
			return false;
		}
	}
	return true;
}

// helper function to find if line segments p1q1 and p2q2 intersect
// got this baby from:
// 	www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
bool Mesh::testIntersect(Triple p1, Triple q1, Triple p2, Triple q2) {
	// Find some orientations
	int o1 = orientation(p1, q1, p2);
	int o2 = orientation(p1, q1, q2);
	int o3 = orientation(p2, q2, p1);
	int o4 = orientation(p2, q2, q1);

	// If you want to know what this does just go to the website
	// where I got it they have more readable code
	return (o1 != o2 && o3 != o4) || (o1 == 0 && onSegment(p1, p2, q1)) || (o2 == 0 && onSegment(p1, q2, q1)) || (o3 == 0 && onSegment(p2, p1, q2)) || (o4 == 0 && onSegment(p2, q1, q2));
}

// helper-helper function. dont worry about this one
bool Mesh::onSegment(Triple p, Triple q, Triple r) {
	return q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) && q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y);
}

// don't worry about this one either
int Mesh::orientation(Triple p, Triple q, Triple r) {
	int val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
	if (val == 0) {
		return 0;
	}
	return (val > 0)? 1: 2;
}

// formula I grabbed from https://www.cs.duke.edu/courses/fall08/cps230/Lectures/L-21.pdf
bool Mesh::inCircumCirc(Triple* t0, Triple* t1, Triple* t2, Triple* p) {
	float** delta = new float*[4];
	for(int i = 0; i < 4; i++) {
		delta[i] = new float[4];
	}
	delta[0][0] = 1;
	delta[0][1] = 1;
	delta[0][2] = 1;
	delta[0][3] = 1;
	delta[1][0] = t0 -> x;
	delta[1][1] = t1 -> x;
	delta[1][2] = t2 -> x;
	delta[1][3] = p -> x;
	delta[2][0] = t0 -> y;
	delta[2][1] = t1 -> y;
	delta[2][2] = t2 -> y;
	delta[2][3] = p -> y;
	delta[3][0] = t0 -> x * t0 -> x + t0 -> y * t0 -> y;
	delta[3][1] = t1 -> x * t1 -> x + t1 -> y * t1 -> y;
	delta[3][2] = t2 -> x * t2 -> x + t2 -> y * t2 -> y;
	delta[3][3] = p -> x * p -> x + p -> y * p -> y;

	float** gamma = new float*[3];
	for(int i = 0; i < 3; i++) {
		gamma[i] = new float[3];
	}
	gamma[0][0] = 1;
	gamma[0][1] = 1;
	gamma[0][2] = 1;
	gamma[1][0] = t0 -> x;
	gamma[1][1] = t1 -> x;
	gamma[1][2] = t2 -> x;
	gamma[2][0] = t0 -> y;
	gamma[2][1] = t1 -> y;
	gamma[2][2] = t2 -> y;

	return det(delta, 4) * det(gamma, 3) < 0;
}

// computes a determinant using cofactor expansion (n!)
float Mesh::det(float** m, int n) {
	if(n == 2) {
		return m[0][0] * m[1][1] - m[0][1] * m[1][0];
	}

	float*** sub = new float**[n];
	for(int i = 0; i < n; i++) {
		sub[i] = new float*[n-1];
	}

	for(int i = 0; i < n; i++) {
		for(int j = 0; j < n-1; j++) {
			sub[i][j] = new float[n-1];
		}
	}

	for(int i = 0; i < n; i++) {
		int s = 0;
		for(int j = 0; j < n-1; j++) {
			if(j == i) {
				s++;
			}

			for(int k = 1; k < n; k++) {
				sub[i][j][k-1] = m[s][k];
			}
			s++;
		}
	}

	float sum = 0;
	bool add = true;
	for(int i = 0; i < n; i++) {
		if(add) {
			sum += m[i][0] * det(sub[i], n-1);
		} else {
			sum -= m[i][0] * det(sub[i], n-1);
		}
		add = !add;
	}
	return sum;
}

// returns a pointer to the nearest MeshTriple to a Triple
MeshTriple* Mesh::getNearest(Triple &t) {
	MeshTriple* nearest = verts[0];
	for(int i = 1; i < verts.size(); i++) {
		if(dist2(t, *(verts[i]->triple)) < dist2(t, *(nearest->triple))) {
			nearest = verts[i];
		}
	}
	return nearest;
}

// TODO Not dist2, travis that's not okay
// returns the squared 2d distance between two Triples
float Mesh::dist2(Triple &a, Triple &b) {
	return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y);
}

int Mesh::toPixelX(float x) {
	return x * CameraConstants::XRES / CameraConstants::S + CameraConstants::XRES / 2;
}

int Mesh::toPixelY(float y) {
	return y * CameraConstants::XRES / CameraConstants::S + CameraConstants::YRES / 2;
}

float Mesh::toImageX(int x) {
	return (x - CameraConstants::XRES / 2) * CameraConstants::S / CameraConstants::XRES;
}

float Mesh::toImageY(int y) {
	return (y - CameraConstants::YRES / 2) * CameraConstants::S / CameraConstants::XRES;
}
