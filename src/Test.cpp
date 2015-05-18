#include <iostream>
#include "NdArray.cpp"
#include <string>

using namespace std;

int main() {
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
    value = "sdflkj";
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
}
