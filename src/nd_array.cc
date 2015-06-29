/*
 * Copyright (c) 2015, DriveAI
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of DriveAI nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
	this -> dimensions = dimensions;
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
char NdArray<T>::getNumDimensions() {
	return numDimensions;
}

template<typename T>
uint64_t* NdArray<T>::getDimensions() {
	return dimensions;
}

template<typename T>
uint64_t NdArray<T>::size() {
	return length;
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
void NdArray<T>::set(uint64_t i, uint64_t j, T value) {
	uint64_t index = i + j * sizes[1];
	array[index] = value;
}

template<typename T>
void NdArray<T>::set(uint64_t i, uint64_t j, uint64_t k, T value) {
	uint64_t index = i + j * sizes[1] + k * sizes[2];
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

template<typename T>
T NdArray<T>::get(uint64_t i, uint64_t j) {
	uint64_t index = i + j * sizes[1];
	return array[index];
}

template<typename T>
T NdArray<T>::get(uint64_t i, uint64_t j, uint64_t k) {
	uint64_t index = i + j * sizes[1] + k * sizes[2];
	return array[index];
}

template class NdArray<std::string>;
template class NdArray<float>;
template class NdArray<Triple>;
