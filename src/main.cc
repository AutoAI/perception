//
//  Main.cpp
//

#include <iostream>
#include "image.cc"
#include <fstream>

using namespace std;

int main() {

    Image img("test.bmp");
    img.write("output.bmp");
    return 0;

}
