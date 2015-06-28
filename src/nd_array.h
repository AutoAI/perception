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

// NdArray represents an n-dimensional array of whatever type you like

#pragma once
#ifndef SRC_ND_ARRAY_H_
#define SRC_ND_ARRAY_H_

#include <stdint.h>

template <typename T>
class NdArray {
public:

	NdArray(char numDimensions, uint64_t* dimensions);

	char getNumDimensions();
	uint64_t* getDimensions();
	uint64_t size();

	void set(uint64_t* dimensions, T value);
	void set(uint64_t index, T value);
	void set(uint64_t i, uint64_t j, T value);
	void set(uint64_t i, uint64_t j, uint64_t k, T value);

	T get(uint64_t* dimensions);
	T get(uint64_t index);
	T get(uint64_t i, uint64_t j);
	T get(uint64_t i, uint64_t j, uint64_t k);
	
private:

	T* array;
	uint64_t* sizes;
	uint64_t* dimensions;
	uint64_t length;
	char numDimensions;
};

#endif  // SRC_ND_ARRAY_H_

