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

#include "seeded_depth_map.h"
#include "coordinate_list.h"
#include "global_constants.h"
#include "mesh.h"

#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

SeededDepthMap::SeededDepthMap(){}

void SeededDepthMap::doCorrespondence(){
	Image left(fileConstants::left);
	Image right(fileConstants::right);

	int xres = left.getWidth();
	int yres = left.getHeight();

	Mesh mesh(getLidarData());
	NdArray<float> bounds = *(mesh.result);

	float f = CameraConstants::F;
	float l = CameraConstants::L;

	unsigned long dimensions[2] = {xres, yres}
	result = new NdArray<float>(2, dimensions);

	for(int v = 0; v < yres; v++) {
		for(int ul = 0; ul < xres; ul++) {
			unsigned long indexmin[3] = {ul, v, 0};
			float zmin = bounds.get(indexmin);
			unsigned long indexmax[3] = {ul, v, 1};
			float zmax = bounds.get(indexmax);
			float bestZ;
			int bestBadness = INT_MAX;
			for(int ur = ceil(ul - (f*l/zmin)); f*l/(ul - ur) < zmax; ur++){
				tempBadness = calcBadness(left, right, v, ul, ur)
				if(tempBadness < bestBadness){
					bestBadness = tempBadness;
					bestZ = f*l/(ul-ur);
				}
			}
			unsigned long setindex[2] = {ul, v};
			result.set(setindex, bestZ);
		}
	}
}

int SeededDepthMap::calcBadness(Image left, Image right, int v, int ul, int ur){
	unsigned long index[3] = {ul, v, 0};
	char al = left.get(index);
	index[2] = 1;
	char bl = left.get(index);
	index[2] = 2;
	char cl = left.get(index);

	unsigned long index2[3] = {ur, v, 0};
	char ar = left.get(index2);
	index[2] = 1;
	char br = left.get(index2);
	index[2] = 2;
	char cr = left.get(index2);

	return ((int)al - (int)ar)*((int)al - (int)ar) + ((int)bl - (int)br)*((int)bl - (int)br) + ((int)cl - (int)cr)*((int)cl - (int)cr);
}

CoordinateList SeededDepthMap::getLidarData(int resolution){
	srand(time(NULL));
	bitmap_image  depth(fileConstants::depth);
	int xres = image.width();
	int yres = image.height();
	
	int count = 0;
	char val;
	CoordinateList list(CoordinateList::CARTESIAN, resolution);

	int xrand;
	int yrand;

	unsigned char red;
	unsigned char green;
	unsigned char blue;

	while (count < resolution) {
		xrand = (rand() % xres)
		yrand = (rand() % yres)
		depth.get_pixel(xrand, yrand, red, green, blue);
		val = red;
		Triple coord(xrand, yrand, val);
		list.set(count, coord);
		count++;
	}

	return list;
}
