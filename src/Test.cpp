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
    cout << "CoordinateList Test\n\n";

    CoordinateList list(Cartesian);

    Coordinate coordinate1[3] = {1, 0, 10};
    list.addCoordinate(coordinate1);
    
    Coordinate coordinate2[3] = {1, 1, 1};
    list.addCoordinate(coordinate2);

    
    list.toType(Spherical);
    Coordinate* a; 
    a = list.getCoordinate(0);
    if(abs(a[0]-sqrt(101))<0.0001 && abs(a[1]-0)<0.0001 && abs(a[2]-acos(10/sqrt(101)))<0.0001)
    {
        cout << "PASS\n";
    }

    a = list.getCoordinate(1);
    if(abs(a[0]-sqrt(3))<0.0001 && abs(a[1]-atan(1))<0.00001 && abs(a[2]-acos(1/sqrt(3)))<0.0001)
    {
        cout << "PASS\n";
    }
    


    
    cout << endl;
    
    list.toType(Perspective);
     a = list.getCoordinate(0);
    if(abs(a[0]-F/10)<0.0001 && abs(a[1]-0)<0.0001 && abs(a[2]-K/10)<0.0001)
    {
      cout << "PASS\n";
    }
    
    a = list.getCoordinate(1);
    if(abs(a[0]-F) <0.0001 && abs(a[1]-F) <0.00001 && abs(a[2]-K) <0.0001)
    {
       cout << "PASS\n";
    }
}

int main() {
    testNdArray();
    testCoordinateList();

}
