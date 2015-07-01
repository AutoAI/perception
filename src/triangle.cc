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

#include "triangle.h"

Triangle::Triangle(MeshTriple *v0, MeshTriple *v1, MeshTriple *v2) {
	points[0] = v0;
	points[1] = v1;
	points[2] = v2;
	v0 -> triangles.push_back(this);
	v1 -> triangles.push_back(this);
	v2 -> triangles.push_back(this);
}

Triple Triangle::getCircumCenter() {
	float a, b, c, d, e, f;
	a = points[0] -> triple -> x;
	b = points[0] -> triple -> y;
	c = points[1] -> triple -> x;
	d = points[1] -> triple -> y;
	e = points[2] -> triple -> x;
	f = points[2] -> triple -> y;
	float p = f*f + e*e - b*b - a*a;
	float q = 2*(c*(f-b)-a*f+b*e+d*(a-e));
	Triple ret(-(d*p-b*(f*f+e*e)+(b*b+a*a)*f+(c*c+d*d)*(b-f))/q, (c*p-a*(f*f+e*e)+(b*b+a*a)*e+(d*d+c*c)*(a-e))/q, 0);
	return ret;
}

bool Triangle::isNeighbor(Triangle* t) {
	// check if they have exactly two points in common
	int common = 0;
	for(int i = 0; i < 3; i++) {
		for(int j = 0; j < 3; j++) {
			if (points[i] == t -> points[j]) {
				common++;
				break;
			}
		}
	}
	return common == 2;
}

std::vector<Triangle*> Triangle::getNeighbors() {
	std::vector<Triangle*> data;
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
	// if any triangles don't share 2 points with the first, they are not a neighbor
	for(int i = 0; i < data.size(); i++) {
		if(!isNeighbor(data[i])) {
			data.erase(data.begin()+i);
			i--;
		}
	}
	return data;
}

// bool Triangle::operator==(const Triangle& tri1) {
// 	bool arr[3] = {false};
// 	bool alreadyfound[3] = {false};
// 	for (int ours = 0; ours < 3; ours++) {
// 		if (*points[ours] == *tri1.points[0] && !alreadyfound[0]) {
// 			arr[ours] = true;
// 			alreadyfound[0]= true;
// 		}
// 		else if(*points[ours] == *tri1.points[1] && !alreadyfound[1]){
// 			arr[ours] = true;
// 			alreadyfound[1] = true;
// 		}
// 		else if(*points[ours] == *tri1.points[2] && !alreadyfound[2]){
// 			arr[ours] = true;
// 			alreadyfound[2] = true;
// 		}
// 	}

// 	return arr[0] && arr[1] && arr[2];
// }


// bool Triangle::operator!=(const Triangle& tri1){
// 	return !(*this == tri1);
// }
