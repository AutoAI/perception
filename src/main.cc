//
//  Main.cpp
//

#include <iostream>
#include "Image.cpp"
#include <fstream>

using namespace std;

int main() {

    MImage img("test.bmp");
    img.write("output.bmp");
    return 0;

}
