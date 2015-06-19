// NdArray.cpp
// @author: Travis Vanderstad, Steven Russo

// NdArray represents an n-dimensional array of whatever type you like

#pragma once
#ifndef SRC_ND_ARRAY_H_
#define SRC_ND_ARRAY_H_

#include <stdint.h>

template <typename T>
class NdArray {
 public:
	T* array;
	uint64_t* sizes;
	uint64_t length;
	char numDimensions;

	NdArray(char numDimensions, uint64_t* dimensions);

	void set(uint64_t* dimensions, T value);
	void set(uint64_t index, T value);

	T get(uint64_t* dimensions);
	T get(uint64_t index);
	// TODO: make things that should be private private and make
	// necessary setters + getters
};

#endif  // SRC_ND_ARRAY_H_

