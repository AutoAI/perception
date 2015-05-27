
#define InterpolatedImage char***
#define Coordinate double
#define Image char**

#include <iostream>
#include "NdArray.cpp"
#include "CoordinateList.cpp"
#include "DepthMap.cpp"

using namespace std;

// Camera focal length in meters
const Coordinate FOCAL_LENGTH = 0.035;
// Pixels per u-coordinate (per meter): 75590.55118811 for a 1" 1080p sensor
const Coordinate c = 75590.55;
// Range constant (farthest range that can be represented): zw = k
const Coordinate k = 120;

const Coordinate rightCameraPosition[] = {1, 0, 0};
const Coordinate leftCameraPosition[] = {-1, 0, 0};

// Camera Resultions
const int xResolution = 1920;
const int yResolution = 1080;

/**
 * Conducts a stereo correspondence seeded by lidar data
 * 
 * @param left  left image to be used in correspondence
 * @param right right image to be used in correspondence
 * @param lidar lidar data to be used as the seed
 * @return a Depth Map of the scene from which the data was captured
 */
CoordinateList seededCorrespondence(Image left, Image right, CoordinateList lidar);

/**
 * Allocates and populates a left image by interpolating input values
 * @param left  left image to be used in correspondence
 * @param right right image to be used in correspondence
 * @param lidar lidar data to be used as the seed
 * @return an x-by-y-by-2 array where [x][y][0] is the smallest disparity to test at [x][y] and [x][y][1] is the largest
 */
InterpolatedImage interpolateLeft(Image left, Image right, CoordinateList seed);

/**
 * Tests disparity values in left and right images as specified by a left interpolated image
 * @param interpolatedLeft  InterpolatedImage that specifies disparity values to be tested
 * @param left              left image to be used in correspondence
 * @param right             right image to be used in correspondence
 * @return a type-2 CoordinateList of the depth values determined by the algorithm
 */
CoordinateList resolveDepthsLeft(InterpolatedImage interpolatedLeft, Image left, Image right);

CoordinateList seededCorrespondence(Image left, Image right, CoordinateList lidar) {
    lidar.toType(3, leftCameraPosition);
    InterpolatedImage interpolatedLeft = interpolateLeft(left, right, lidar);
    CoordinateList depthLeft = resolveDepthsLeft(interpolatedLeft, left, right);
    return depthLeft;
}

InterpolatedImage interpolateLeft(Image left, Image right, CoordinateList seed) {
    InterpolatedImage returnedImage = new char**[xResolution]; //[yResolution][2];

    return returnedImage;
}

CoordinateList resolveDepthsLeft(InterpolatedImage interpolatedLeft, Image left, Image right) {
    CoordinateList returnedList;

    return returnedList;
}

int main() {
    cout << "SeededDepthMap" << endl;
    return 0;
}

