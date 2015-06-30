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

#include "mesh_triple.h"

MeshTriple::MeshTriple(Triple* triple) {
	this -> triple = triple;
}

bool MeshTriple::isNeighbor(MeshTriple* m) {
	for(int i = 0; i < triangles.size(); i++) {
		for(int j = 0; j < m -> triangles.size(); j++) {
			if (triangles[i] == m -> triangles[j]) {
				return true;
			}
		}
	}
	return false;
}

std::vector<MeshTriple*> MeshTriple::getNeighbors() {
	std::vector<MeshTriple*> result;
	// iterate over triangles
	for (int i = 0; i < triangles.size(); i++) {
		Triangle* tri = triangles[i];
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
	for(int i = 0; i < result.size(); i++){
		if(result[i] == this){
			result.erase(result.begin()+i);
			break;
		}
	}
	return result;
}

bool MeshTriple::operator==(const MeshTriple &t1) {
	return (*(this->triple) == *(t1.triple));
}

bool MeshTriple::operator!=(const MeshTriple &t1) {
	return !(*this == t1);
}
