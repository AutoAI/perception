#include <iostream>
#include <string>

#include "NdArray.cpp"
#include "CoordinateList.cpp"

using namespace std;

void testNdArray() {
    char bounds[3] = {3, 3, 3};
    NdArray<string> array(3, bounds);

    char location[3] = {0, 0, 0};
    string value = "asdf";
    array.set(location, value);
    
    if (value.compare(array.get(location)) == 0) {
        cout << "PASS" << endl;
    } else {
        cout << "FAIL" << endl;
    }

    location[0] = 0;
    location[1] = 2;
    location[2] = 0;
    value = "sdflj";
    array.set(location, value);
    
    if (value.compare(array.get(location)) == 0) {
        cout << "PASS" << endl;
    } else {
        cout << "FAIL" << endl;
    }
    
    location[0] = 2;
    location[1] = 0;
    location[2] = 2;
    value = "sdflkjsd";
    array.set(location, value);
    
    if (value.compare(array.get(location)) == 0) {
        cout << "PASS" << endl;
    } else {
        cout << "FAIL" << endl;
    }

    location[0] = 2;
    location[1] = 1;
    location[2] = 1;
    value = "lkjsd";
    array.set(location, value);
    
    if (value.compare(array.get(location)) == 0) {
        cout << "PASS" << endl;
    } else {
        cout << "FAIL" << endl;
    }
    
    location[0] = 1;
    location[1] = 1;
    location[1] = 1;
    value = "sdflkjsd";
    array.set(location, value);
    
    if (value.compare(array.get(location)) == 0) {
        cout << "PASS" << endl;
    } else {
        cout << "FAIL" << endl;
    }

    location[0] = 1;
    location[1] = 2;
    location[2] = 1;
    value = "dflkjsd";
    array.set(location, value);
    
    if (value.compare(array.get(location)) == 0) {
        cout << "PASS" << endl;
    } else {
        cout << "FAIL" << endl;
    }
    cout << endl;
}

void testCoordinateList() {
    CoordinateList list(Cartesian);

    Coordinate coordinate1[3] = {1, 0, 0};
    list.addCoordinate(coordinate1);
    
    Coordinate coordinate2[3] = {1, 1, 1};
    list.addCoordinate(coordinate2);

    for (int x = 0; x < 2; x++) {
        for (int y = 0; y < 3; y++) {
            Coordinate* a = list.getCoordinate(x);
            cout << a[y] << " ";
        }
        cout << endl;
    }

    list.toType(1);

    for (int x = 0; x < 2; x++) {
        for (int y = 0; y < 3; y++) {
            Coordinate* a = list.getCoordinate(x);
            cout << a[y] << " ";
        }
        cout << endl;
    }

    list.toType(2);

    for (int x = 0; x < 2; x++) {
        for (int y = 0; y < 3; y++) {
            Coordinate* a = list.getCoordinate(x);
            cout << a[y] << " ";
        }
        cout << endl;
    }
    cout << endl;
}

int main() {
    testNdArray();
    testCoordinateList();

}
