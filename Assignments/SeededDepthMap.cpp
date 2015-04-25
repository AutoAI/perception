#define Coordinate double
#define Image char[][][]

// camera focal length in meters
const double FOCAL_LENGTH = 0.035;
// pixels per u-coordinate (per meter): 75590.5511811 for a 1" 1080p sensor
const double c = 75590.55;
// range constant (farthest range that can be represented): zw = k
const double k = 120;
// camera resolutions
const int xResolution = 1920;
const int yResolution = 1080;
// coordinate triples that specify, in world (x,y,z) coordinates, the offset of the cameras relative to the lidar scanner
const Coordinate[] rightCameraPosition;
const Coordinate[] leftCameraPosition;

/**
 * Conducts a stereo correspondence seeded by lidar data
 * 
 * @param left  left image to be used in correspondence
 * @param right right image to be used in correspondence
 * @param lidar lidar data to be used as the seed
 * @return a Depth Map of the scene from which the data was captured
 */
CoordinateList seededCorrespondence(Image left, Image right, CoordinateList lidar)
{
    lidar.toType(3, leftCameraPosition);
    InterpolatedImage interpolatedLeft = interpolateLeft(left, right, lidar);
    CoordinateList depthLeft = resolveDepthsLeft(interpolatedImageLeft, left, right);
    return depthLeft;
}

/**
 * Allocates and populates a left image by interpolating input values
 * @param left  left image to be used in correspondence
 * @param right right image to be used in correspondence
 * @param lidar lidar data to be used as the seed
 * @return an x-by-y-by-2 array where [x][y][0] is the smallest disparity to test at [x][y] and [x][y][1] is the largest
 */
InterpolatedImage interpolateLeft(Image left, Image right, CoordinateList seed)
{
    Image returnedImage = new char[xResolution][yResolution][2];
    // @TODO: contrive a kickass acceleration structure for interpolation (probably a hash table because they're kickass)
    return returnedImage;
}

/**
 * Tests disparity values in left and right images as specified by a left interpolated image
 * @param interpolatedLeft  InterpolatedImage that specifies disparity values to be tested
 * @param left              left image to be used in correspondence
 * @param right             right image to be used in correspondence
 * @return a type-2 CoordinateList of the depth values determined by the algorithm
 */
CoordinateList resolveDepthsLeft(InterpolatedImage interpolatedLeft, Image left, Image right)
{
    CoordinateList returnedList = new CoordinateList();
    // @TODO: test the disparity values
    return returnedList;
}
