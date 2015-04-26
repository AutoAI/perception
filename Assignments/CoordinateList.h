#include <vector>
#include <cmath>
#define Coordinate double
#define Cartesian 0
#define Spherical 1
#define Perspective 2
#define k 0
#define f 0
#define c 0

using namespace std;

class CoordinateList {
    public:
        vector<Coordinate*> coordinates;
        char type;
        void addCoordinate(Coordinate coordinate);
        void toType(char newType);
        void toType(char newType, const Coordinate* offset);
        CoordinateList clone();
};

