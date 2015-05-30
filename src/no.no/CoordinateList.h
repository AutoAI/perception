#include <vector>
#include <cmath>
#define Cartesian 0
#define Spherical 1
#define Perspective 2
#define K 120
#define F 0.035
#define C 75590.55
#define Coordinate float

using namespace std;

/**
* List of all coordinate triples
*/
class CoordinateList {
    public:

        // List of all coordinate triples
        vector<Coordinate*> coordinates;

        /*
        *  Type of coordinates:
        *  0 = cartesian (x, y, z)
        *  1 = spherical (r, theta, roe)
        *  2 = perspective (u, v, w)
        */
        char type;
        CoordinateList(char type);
        void addCoordinate(Coordinate* coordinate);

        /**
        * Converts the coordinate triples to the type specified using the same origin
        *
        * @param type the type to convert the coordinates to
        */
        void toType(char newType);

        //TODO pass a length
         /**
         * Converts the coodinates triples to the type specified using a new origin, offset from the previous origin by the given (x,y,z) triple
         * 
         * @param type      the type to convert the coordinates to
         * @param offset    the offset in world coordinates from the existing origin to the new origin. It's always of length 3
         */
        void toType(char newType, Coordinate* offset);
        Coordinate* getCoordinate(int index);
        CoordinateList clone();
    private:
        void toCartesian();
};
