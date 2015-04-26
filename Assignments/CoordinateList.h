#include <vector>

using namespace std;

class CoordinateList {
    public:
        vector<Coordinate*> coordinates;
        char type;
        void toType(int type);

        //TODO pass a length
        void toType(int type, const Coordinate* offset);
};

