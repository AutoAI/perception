//
//  Main.cpp
//

#include <iostream>
#include "image.cc"
#include <fstream>

using namespace std;

int main() {
    Image img("test/test_data/test.bmp");
    img.write("test/test_data/output.bmp");
    return 0;
}
