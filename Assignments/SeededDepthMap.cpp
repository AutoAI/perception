
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

CoordinateList seededCorrespondence(Image left, Image right, CoordinateList lidar);
InterpolatedImage interpolateLeft(Image left, Image right, CoordinateList seed);
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
//    NdArray<int> myArray;
    return 0;
}

