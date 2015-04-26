
#pragma once

class DepthMap {
    public:
        char*** colorMap;
        float** depthMap;

        DepthMap(Coordinate x, Coordinate y);
};

