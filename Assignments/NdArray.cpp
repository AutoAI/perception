
#pragma once

#include "NdArray.h"

template<typename T>
NdArray<T>::NdArray(char numDimensions, char* dimensions) {
    sizes = new unsigned long[numDimensions];
    length = 1;
    for (char i = 0; i < numDimensions; i++) {
        if (i == 0) {
            sizes[i] = 1;
        } else {
            sizes[i] = dimensions[i - 1] * sizes[i - 1];
        }
        this->numDimensions = numDimensions;
        array = new T[length];
    }
}

template<typename T>
void NdArray<T>::set(char* dimensions, T value) {
    char index = 0;
    for (char i = 0; i < numDimensions; i++) {
        index += dimensions[i] * sizes[i];
    }
    array[index] = value;
}



