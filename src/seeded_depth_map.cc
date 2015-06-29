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

#include "ros/ros.h"
#include "seeded_depth_map.h"
#include "coordinate_list.h"
#include "global_constants.h"
#include "mesh.h"

#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

namespace fileConstants {
	std::string left =  "src/perception/src/test/test_data/left.bmp";
	std::string right = "src/perception/src/test/test_data/right.bmp";
	std::string depth = "src/perception/src/test/test_data/depth.bmp";
}

void SeededDepthMap::saveImage(NdArray<float> &c, std::string filename) {
	int height = CameraConstants::YRES;
	int width = CameraConstants::XRES;
	bitmap_image image(width, height);
	for (int x = 0; x < width; x++) {
		for (int y = 0; y < height; y++) {
			float r = c.get(x, y, 0);
			float g = c.get(x, y, 1);
			image.set_pixel(x, y, char(1/r), char(1/g), 0);
		}
	}
	image.save_image(filename);
}

void SeededDepthMap::doCorrespondence(){
	bitmap_image left(fileConstants::left);
	bitmap_image right(fileConstants::right);

	int xres = left.width();
	int yres = left.height();

	CoordinateList c = getLidarData(10000);

	Mesh mesh(&c);

	NdArray<float> bounds = *(mesh.result);

	saveImage(bounds, "bounds.bmp");

	float f = CameraConstants::F;
	float l = CameraConstants::L;

	unsigned long dimensions[2] = {xres, yres};
	result = new NdArray<float>(2, dimensions);

	ROS_INFO("doing correspondence...");
	bool print = true;
	ROS_INFO("for v");
	for(int v = 0; v < yres; v++) {
		ROS_INFO("v = %d", v);
		ROS_INFO("for ul");
		for(int ul = 0; ul < xres; ul++) {
			ROS_INFO("ul = %d", ul);
			float zmin = bounds.get(ul, v, 0);
			float zmax = bounds.get(ul, v, 1);
			float bestZ;
			int bestBadness = INT_MAX;
			ROS_INFO("for ur");
			for(int ur = ceil(ul - (f*l/zmin)); f*l/(ul - ur) < zmax && ur < ul; ur++){
				ROS_INFO("ur = %d", ur);
				if(print){
					ROS_INFO("f: %f, l: %f", f, l);
					ROS_INFO("zmin: %f, zmax: %f",zmin, zmax);
					ROS_INFO("ul: %d, ur: %d, xres: %d", ul, ur, xres);
					ROS_INFO("------------------");
					print = false;
				}
				int tempBadness = calcBadness(left, right, v, ul, ur);
				if(tempBadness < bestBadness) {
					bestBadness = tempBadness;
					bestZ = f*l/(ul-ur);
				}
			}
			ROS_INFO("/for ur");
			result -> set(ul, v, bestZ);
		}
		ROS_INFO("/for ul");
	}
	ROS_INFO("/for v");
	ROS_INFO("\tcorrespondence done");
}

SeededDepthMap::SeededDepthMap(){} 

int SeededDepthMap::calcBadness(bitmap_image left, bitmap_image right, int v, int ul, int ur){
	unsigned char red1, red2, grn1, grn2, blu1, blu2;
	left.get_pixel((unsigned int)ul, (unsigned int)v, red1, grn1, blu1);
	right.get_pixel((unsigned int)ur, (unsigned int)v, red2, grn2, blu2);
	return ((int)red1-(int)red2)*((int)red1-(int)red2)+((int)blu1-(int)blu2)*((int)blu1-(int)blu2)+((int)grn1-(int)grn2)*((int)grn1-(int)grn2);
}

CoordinateList SeededDepthMap::getLidarData(int num_samples){
	bitmap_image depth_map(fileConstants::depth);
	int xres = depth_map.width();
	int yres = depth_map.height();
	
	CoordinateList list(CoordinateList::PERSPECTIVE, num_samples);

	float xrand;
	float yrand;
	unsigned char red;
	unsigned char green;
	unsigned char blue;
	for(int i = 0; i < num_samples; i++) {
		xrand = rand() % xres;
		yrand = rand() % yres;
		depth_map.get_pixel(xrand, yrand, red, green, blue);

		xrand = Mesh::toImageX(xrand);
		yrand = Mesh::toImageY(yrand);
		Triple coord(xrand, yrand, 1.0/float(red));
		list.set(i, coord);
	}

	return list;
}
