// NdArray.cpp
// @author: Travis Vanderstad, Steven Russo

// If you're looking for the documentation, its in NdArray.h

#include <stdint.h>

#include <string>

#include "ros/ros.h"

#include "triple.h"
#include "nd_array.h"

template<typename T>
NdArray<T>::NdArray(char numDimensions, uint64_t* dimensions) {
	sizes = new uint64_t[numDimensions];
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
void NdArray<T>::set(uint64_t* dimensions, T value) {
	uint64_t index = 0;
	for (char i = 0; i < numDimensions; i++) {
		index += dimensions[i] * sizes[i];
	}
	array[index] = value;
}

template<typename T>
void NdArray<T>::set(uint64_t index, T value) {
	array[index] = value;
}

template<typename T>
T NdArray<T>::get(uint64_t* dimensions) {
	uint64_t index = 0;
	for (char i = 0; i < numDimensions; i++) {
		index += dimensions[i] * sizes[i];
	}
	return array[index];
}

template<typename T>
T NdArray<T>::get(uint64_t index) {
	return array[index];
}

template class NdArray<std::string>;
template class NdArray<float>;
template class NdArray<Triple>;
