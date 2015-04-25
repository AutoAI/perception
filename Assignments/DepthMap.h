// represents an image with per-pixel depth data in addition to color data
class DepthMap {
    public:
        // the color data as [x][y][r/g/b]
        char[][][] colorMap;
        // the depth data as [x][y]; depth is stored as k/z where k is 'the range constant' and z is the perpendicular distance from the camera to the point captured by the pixel
        float[][] depthMap;
        // constructs the depth map by allocating the arrays above
        DepthMap(double x, double y);
};
