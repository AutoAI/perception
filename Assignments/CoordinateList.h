#define Coordinate double

// represents a list of the coordinates of 3d points, in one of several coordinate systems
class CoordinateList {
    public:
        // List of all coordinate triples
        List<Coordinates[]> coordinates;
        
        // Type of coordinates:
        // 0 = cartesian (x, y, z)
        // 1 = spherical (r, theta, roe)
        // 2 = perspective (u, v, w)
        char type;
        
        /**
         * Converts the coordinate triples to the type specified using the same origin
         * 
         * @param type  the type to convert the coordinates to
         */
        void toType(int type);
        
        /**
         * Converts the coodinates triples to the type specified using a new origin, offset from the previous origin by the given (x,y,z) triple
         * 
         * @param type      the type to convert the coordinates to
         * @param offset    the offset in world coordinates from the existing origin to the new origin
         */
        void toType(int type, Coordinate[] offset);
        
        /**
         * Makes and returns a copy of this CoordinateList
         */
        CoordinateList clone();
};
