// NdArray.cpp
// @author: Travis Vanderstad, Steven Russo

// If you're looking for the documentation, its in NdArray.h

#pragma once

#include "Triple.h"
#include "NdArray.h"
#include <string>

template<typename T>
NdArray<T>::NdArray(char numDimensions, unsigned long* dimensions) {
    sizes = new unsigned long[numDimensions];
    length = 1;
    for (char i = 0; i < numDimensions; i++) {
        length *= dimensions[i];
        if (i == 0) {
            sizes[i] = 1;
        } else {
            sizes[i] = dimensions[i - 1] * sizes[i - 1];
        }
    }
    this->numDimensions = numDimensions;
    array = new T[length];
}

template<typename T>
void NdArray<T>::set(unsigned long* dimensions, T value) {
    char index = 0;
    for (char i = 0; i < numDimensions; i++) {
        index += dimensions[i] * sizes[i];
    }
    array[index] = value;
}

template<typename T>
void NdArray<T>::set(unsigned long index, T value) {
    array[index] = value;
}

template<typename T>
T NdArray<T>::get(unsigned long* dimensions) {
    char index = 0;
    for (char i = 0; i < numDimensions; i++) {
        index += dimensions[i] * sizes[i];
    }
    return array[index];
}

template<typename T>
T NdArray<T>::get(unsigned long index) {
    return array[index];
}

template class NdArray<std::string>;
template class NdArray<float>;
template class NdArray<Triple>;
