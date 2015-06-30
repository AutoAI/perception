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

// Mesh.cpp
// @author: Travis Vanderstad, Parth Mehrotra

// If you're looking for the documentation, its in Mesh.h

#define _USE_MATH_DEFINES

#include "ros/ros.h"

#include <stdexcept>
#include <math.h>
#include <stddef.h>
#include <iostream>

#include "coordinate_list.h"

#include "mesh.h"
#include "mesh_triple.cc"
#include "global_constants.h"

using namespace std;

Mesh::Mesh(CoordinateList* cList) {
	bool debug = true;

	if (cList == NULL) {
		throw std::invalid_argument("CoordinateList must not be null");
	} else if(cList->getLength() < 3) {
		throw std::invalid_argument("CoordinateList must contain at least 3 points");
	}
	list = cList;

	// choose seed point, sort others accorinding to distance from seed
	if(debug) {
		ROS_INFO("choosing seed...");
	}
	MeshTriple* s = chooseSeed();
	(*list).bucketSort(*(s -> triple));
	if(debug) {
		ROS_INFO("\tseed chosen");
	}

	// construct initial convex hull (counter-clockwise)
	if(debug) {
		ROS_INFO("initializing hull...");
	}
	vector<MeshTriple*> hull;
	this->hull = hull;
	initHull(0, 1, 2);
	if(debug) {
		ROS_INFO("\thull initialized");
	}

	// Insert each vert, flipping after each
	if(debug) {
		ROS_INFO("inserting points...");
	}
	for(unsigned long i = 3; i < list -> getLength(); i++) {
		insertVert(list -> getPtr(i));
		// definitely flip the triangles of the most recently added meshtriple
		vector<Triangle*> trisToFlip = verts[verts.size()-1] -> triangles;
		// as long as we have tris to flip...
		while(trisToFlip.size() > 0) {
			// flip the most recent triangle we set out to flip, and if we had to flip it,
			// add its neighbors to the tris we might have to flip and we'll come back to it again when we're done with those
			if(flip(trisToFlip[trisToFlip.size()-1]) == 1) {
				vector<Triangle*> trisToAdd = trisToFlip[trisToFlip.size()-1] -> getNeighbors();
				for(int j = 0; j < trisToAdd.size(); j++) {
					trisToFlip.push_back(trisToAdd[j]);
				}
			// otherwise, remove it from our list 
			} else {
				trisToFlip.erase(trisToFlip.begin()+trisToFlip.size()-1);
			}
		}
		// TODO: use a stack instead, because that's basically what the list is
	}
	if(debug) {
		ROS_INFO("\tpoints inserted");
	}
	// INSERT THEN FLIP
	// // sequentially insert points, adding edges from new point to 'visible' points on the convex hull
	// if(debug) {
	// 	ROS_INFO("inserting points...");
	// }
	// for(unsigned long i = 3; i < list -> getLength(); i++) {
	// 	insertVert(list -> getPtr(i));
	// }
	// if(debug) {
	// 	ROS_INFO("\tpoints inserted");
	// }

	// // iteratively 'flip' triangles until no more triangles need be flipped
	// if(debug) {
	// 	ROS_INFO("flipping edges...");
	// }
	// int maxIterations = 12;
	// int sumFlips;
	// for(int i = 0; sumFlips != 0; i++) {
	// 	sumFlips = 0;
	// 	for(int j = 0; j < tris.size(); j++) {
	// 		sumFlips += flip(tris[j]);
	// 	}
	// 	ROS_INFO("flipped: %d", sumFlips);
	// 	if(i == maxIterations - 1){
	// 		// break;
	// 	}
	// }
	// if(debug) {
	// 	ROS_INFO("\tedges flipped");
	// }

	// set up the data array - first, find max number of vert neighbors
	if(debug) {
		ROS_INFO("allocating space for data...");
	}
	int maxNeighbors = 0;
	for(int i = 0; i < verts.size(); i++) {
		maxNeighbors = (maxNeighbors > verts[i] -> getNeighbors().size()) ? maxNeighbors : verts[i] -> getNeighbors().size();
	}
	if(debug) {
		ROS_INFO("data array dimensions: %zu, %d, %d, %d", sizeof(float), CameraConstants::XRES, CameraConstants::YRES, maxNeighbors+1);
		ROS_INFO("bytes to allocate: %zu", sizeof(float)*CameraConstants::XRES*CameraConstants::YRES*(maxNeighbors+1));
	}
	// init the data
	unsigned long bounds[3] = {CameraConstants::XRES, CameraConstants::YRES, maxNeighbors+1};
	data = new NdArray<float>(3, bounds);
	if(debug) {
		ROS_INFO("\tallocated space for data");
	}

	// populate data
	if(debug) {
		ROS_INFO("generating data...");
	}
	for(int i = 0; i < verts.size(); i++) {
		fillRegion(verts[i]);
	}
	if(debug) {
		ROS_INFO("\tgenerated data");
	}
	if(debug) {
		ROS_INFO("\tdata generated");
	}

	// init result
	if(debug) {
		ROS_INFO("generating result...");
	}
	unsigned long bounds2[3] = {CameraConstants::XRES, CameraConstants::YRES, 2};
	result = new NdArray<float>(3, bounds2);

	// calculate result
	for(int i = 0; i < CameraConstants::XRES; i++) {
		for(int j = 0; j < CameraConstants::YRES; j++) {
			float max = -1;
			float min = CameraConstants::K;
			//TODO should rename k
			for(int k = 0; k < maxNeighbors+1; k++) {
				if(data -> get(i, j, k) == -1) {
					break;
				}
				min = (min < data -> get(i, j, k)) ? min : data -> get(i, j, k);
				max = (max > data -> get(i, j, k)) ? max : data -> get(i, j, k);
			}
			result -> set(i, j, 0, min);
			result -> set(i, j, 1, max);
		}
	}
	if(debug) {
		ROS_INFO("\tresult generated");
	}
}

MeshTriple* Mesh::chooseSeed() {
	// just give 'em any old seed
	return new MeshTriple((*list).getPtr(0));
}

void Mesh::initHull(unsigned long index0, unsigned long index1, unsigned long index2) {
	// check angle 0 to see which way the verts should be ordered to make the triangle counter-clockwise
	float dTheta = atan2((*list).get(2).y-(*list).get(0).y, (*list).get(2).x-(*list).get(0).x)-atan2((*list).get(1).y-(*list).get(0).y, (*list).get(1).x-(*list).get(0).x);
	if(dTheta > 2*M_PI) {
		dTheta -= M_PI;
	} else if(dTheta < 0) {
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

	// add any visible verts on the hull to a list. edges will be made to all of these
	// (remember where the most clockwise and most counter-clockwise verts are)
	vector<MeshTriple*> connectorTriples;
	bool visibilities[hull.size()];
	int c; // index of clockwise-most vert
	int cc; // index of counter-clockwise-most vert
	for(int i = 0; i < hull.size(); i++) {
		if(isVisible(*v, *(hull[i] -> triple))) {
			connectorTriples.push_back(hull[i]);
			visibilities[i] = true;
		} else {
			visibilities[i] = false;
		}
	}

	bool allVisible = true;
	for(int i = 0; i < hull.size(); i++) {
		if(!visibilities[i]){
			allVisible = false;
			break;
		}
	}

	// if they're all visible, find the triangle that doesn't work and find c and cc that way
	if(allVisible) {
		if(!goodTri(connectorTriples[0] -> triple, connectorTriples[hull.size()-1] -> triple, v)) {
			c = hull.size() - 1;
			cc = 0;
		} else {
			for(int i = 1; i < hull.size(); i++) {
				if(!goodTri(connectorTriples[i-1] -> triple, connectorTriples[i] -> triple, v)) {
					c = i - 1;
					cc = i;
					break;
				}
			}
		}
	} else { // otherwise, find the c and cc such that they are visible but the next c or cc is not
		for(int i = 0; i < hull.size(); i++) {
			if(visibilities[i] && !visibilities[(i+1)%hull.size()]) {
				cc = i;
			} else if(visibilities[(i+1)%hull.size()] && !visibilities[i]) {
				c = (i+1)%hull.size();
			}
		}
	}

	// make triangles, starting with the most clockwise pair of points and working counter-clockwise
	int numTrisAdded = 0;
	for(int i = c; i != cc; i = (i+1)%hull.size()) {
		Triangle* temp = new Triangle(hull[i], hull[(i+1)%hull.size()], t);
		tris.push_back(temp);
		numTrisAdded++;
	}

	// trim the hull. of those verts visible to t, only the most clockwise and most counter-clockwise verts will remain
	int i = cc - 1;
	while(true){
		if(i == -1){
			i = hull.size() - 1;
		}
		if(i == c){
			break;
		}
		hull.erase(hull.begin()+i);
		if(i < c){
			c--;
		}
		if(i < cc){
			cc--;
		}
		i--;
	}

	// add the vert to our list of verts
	verts.push_back(t);

	// insert the new point in-between the most clockwise and most counter-clockwise verts
	hull.insert(hull.begin()+cc, t);
}

// is a point 'visible' from another? that is, does the line between them pass through the hull? this function answers these questions
bool Mesh::isVisible(Triple& a, Triple& d) {
	if(!((a == *(hull[hull.size()-1] -> triple)) || ((d == *(hull[hull.size()-1] -> triple))) || (a == *(hull[0] -> triple)) || ((d == *(hull[0] -> triple)))) && testIntersect(*(hull[hull.size()-1] -> triple), *(hull[0] -> triple), a, d)) {
		return false;
	}

	for(unsigned long i = 1; i < hull.size(); i++) {
		if(!((a == *(hull[i-1] -> triple)) || ((d == *(hull[i-1] -> triple))) || (a == *(hull[i] -> triple)) || ((d == *(hull[i] -> triple)))) && testIntersect(*(hull[i-1] -> triple), *(hull[i] -> triple), a, d)) { 
			return false;
		}
	}
	return true;
}

// helper function to find if line segments p1q1 and p2q2 intersect
// got this baby from www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
bool Mesh::testIntersect(Triple p1, Triple q1, Triple p2, Triple q2) {
	// Find some orientations
	int o1 = orientation(p1, q1, p2);
	int o2 = orientation(p1, q1, q2);
	int o3 = orientation(p2, q2, p1);
	int o4 = orientation(p2, q2, q1);

	// If you want to know what this does just go to the website where I got it they have more readable code
	return (o1 != o2 && o3 != o4)||(o1 == 0 && onSegment(p1, p2, q1))||(o2 == 0 && onSegment(p1, q2, q1))||(o3 == 0 && onSegment(p2, p1, q2))||(o4 == 0 && onSegment(p2, q1, q2));
}

// don't worry about this one either
int Mesh::orientation(Triple p, Triple q, Triple r) {
	int val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
	if (val == 0) {
		return 0;
	}
	return (val > 0)? 1: 2;
}

// helper-helper function. dont worry about this one
bool Mesh::onSegment(Triple p, Triple q, Triple r) {
	return q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) && q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y);
}

// is this tri good?
bool Mesh::goodTri(Triple* t0, Triple* t1, Triple* t2){
	for(int i = 0; i < verts.size(); i++) {
		if(inTri(t0, t1, t2, verts[i] -> triple)) {
			return false;
		}
	}
	return true;
}

// code I grabbed from http://stackoverflow.com/questions/2049582/how-to-determine-a-point-in-a-triangle
bool Mesh::inTri(Triple* t0, Triple* t1, Triple* t2, Triple* p){
	bool b1, b2, b3;

    b1 = sign(p, t0, t1) < 0.0f;
    b2 = sign(p, t1, t2) < 0.0f;
    b3 = sign(p, t2, t0) < 0.0f;

    return ((b1 == b2) && (b2 == b3));
}

// helper function for inTri
float Mesh::sign (Triple* p1, Triple* p2, Triple* p3){
    return (p1 -> x - p3 -> x) * (p2 -> y - p3 -> y) - (p2 -> x - p3 -> x) * (p1 -> y - p3 -> y);
}

// flips necessary edges of a triangle, returns number of edges flipped
int Mesh::flip(Triangle* t) {
	// for each neighbor...
	vector<Triangle*> neighbors = t -> getNeighbors();
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
				if(t -> points[k] == allPoints[j]){
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

		// if the pair is not locally delaunay, flip it and stop (we'll come back for the rest in the next iteration)
		if(inCircumCirc(allPoints[i1] -> triple, allPoints[i2] -> triple, allPoints[i3] -> triple, allPoints[i4] -> triple)) {
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
	for(unsigned long i = 0; i < tris.size(); i++) {
		if(tris[i] == t) {
			tris.erase(tris.begin()+i);
			return;
		}
	}
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

// returns the squared 2d distance between two Triples
float Mesh::dist2(Triple &a, Triple &b) {
	return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y);
}

// fill in the triangles spanned by the meshtriple and the circumcenters of two neighboring triangles containing the meshtriple
// does not fill anything in for meshtriples on the hull
void Mesh::fillRegion(MeshTriple* m) {
	// if its on the hull, like, just, no.
	for(int i = 0; i < hull.size(); i++) {
		if(m == hull[i]) {
			return;
		}
	}
	int n = m -> triangles.size();
	// find pairs of neighboring triangles and fill them
	for(int i = 0; i < n; i++) {
		for(int j = i+1; j < n; j++) {
			if(m -> triangles[i] -> isNeighbor(m -> triangles[j])) {
				Triple temp1 = m -> triangles[i] -> getCircumCenter();
            	Triple temp2 = m -> triangles[j] -> getCircumCenter();
            	fillTri(*(m -> triple), temp1, temp2, m);
			}
		}
	}
}

void Mesh::fillTri(Triple &a, Triple &b, Triple &c, MeshTriple* m) {
	int minx, miny, maxx, maxy;
	vector<MeshTriple*> neighbors = m -> getNeighbors();
	int n = neighbors.size();
	float tempData[n];
	for(int i = 0; i < n; i++) {
		tempData[i] = neighbors[i] -> triple -> z;
	}
	minx = (int)std::min(std::min(a.x, b.x), c.x);
	miny = (int)std::min(std::min(a.y, b.y), c.y);
	maxx = (int)std::max(std::max(a.x, b.x), c.x);
	maxy = (int)std::max(std::max(a.y, b.y), c.y);
	int start = minx;
	for(int i = miny; i < maxy; i++) {
		int j = start;
		int temp = j-1;
		while(j >= minx && !inTri(a, b, c, temp, i)) {
			j--;
		}
		start = j;
		while(j < maxx && inTri(a, b, c, j, i)) {
			for(int k = 0; k < n; k++) {
				data -> set(j, i, k, tempData[k]);
			}
			for(int k = n; k < data -> getDimensions()[2]; k++) {
				data -> set(j, i, k, -1);
			}
		}
	}
}

// overloaded from above for faster use of pixel values rather than triples
bool Mesh::inTri(Triple &t0, Triple &t1, Triple &t2, int x, int y){
	bool b1, b2, b3;

    b1 = sign(x, y, t0, t1) < 0.0f;
    b2 = sign(x, y, t1, t2) < 0.0f;
    b3 = sign(x, y, t2, t0) < 0.0f;

    return ((b1 == b2) && (b2 == b3));
}

// helper function for inTri above
float Mesh::sign (int x, int y, Triple &p2, Triple &p3){
    return (toImageX(x) - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (toImageY(y) - p3.y);
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
