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

Triple Triangle::getCenter() {
	Triple returned;
	returned.x = (points[0] -> triple -> x + points[1] -> triple -> x + points[2] -> triple -> x)/3;
	returned.y = (points[0] -> triple -> y + points[1] -> triple -> y + points[2] -> triple -> y)/3;
	returned.z = (points[0] -> triple -> z + points[1] -> triple -> z + points[2] -> triple -> z)/3;
	return returned;
}

bool Triangle::operator==(const Triangle& tri1) {
	bool arr[3] = {false};
	bool alreadyfound[3] = {false};
	for (int ours = 0; ours < 3; ours++) {
		if (*points[ours] == *tri1.points[0] && !alreadyfound[0]) {
			arr[ours] = true;
			alreadyfound[0]= true;
		}
		else if(*points[ours] == *tri1.points[1] && !alreadyfound[1]){
			arr[ours] = true;
			alreadyfound[1] = true;
		}
		else if(*points[ours] == *tri1.points[2] && !alreadyfound[2]){
			arr[ours] = true;
			alreadyfound[2] = true;
		}
	}

	return arr[0] && arr[1] && arr[2];
}


bool Triangle::operator!=(const Triangle& tri1){
	return !(*this == tri1);
}
