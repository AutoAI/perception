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
 * 3. Neither the name of whiskey-foxtrot nor the names of its
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

/**
* CoordinateList.h
*
* CoordinateList is a list of coordinates
* Supports changing coodrinate system of all coordinates and sorting base on 2D distance from a point
*
* @author: Travis Vanderstad, Parth Mehrotra, Matthew Bowyer
*/

#pragma once
#ifndef SRC_COORDINATE_LIST_H_
#define SRC_COORDINATE_LIST_H_

#include <stdint.h>

#include <vector>
#include <algorithm>

#include "triple.h"
#include "global_constants.h"

/** List of all coordinate triples */
class CoordinateList {
	public:
		enum ListType {
			CARTESIAN,
			SPHERICAL,
			PERSPECTIVE
		};

		ListType type; /** type of the coordinate (as #define'd) */

		/**
		* Creates a CoordinateList using a type and an initial length
		*
		* @param type a char that represents the type of the CoordinateList (Cartesian, Spherical, or Perspective)
		* @param length the length of the initial list
		*/
		CoordinateList(ListType type, uint64_t length);

		/**
		* converts all coordinates to the specified type
		*
		* @param newType type to convert to
		*/
		void toType(ListType newType);

		/**
		* converts all coodinates to the specified type using an offset to the new origin
		*
		* @param newType new type for the list
		* @param offset a Triple offset to specify the new origin
		*/
		void toType(ListType newType, Triple offset);

		/**
		* set a particular element by its list index
		*
		* @param index the index that you're setting
		* @param value the Triple that it's being set too
		*/
		void set(uint64_t index, Triple value);

		/**
		* returns a particular element by its list index
		*
		* @param index returns Triple at index
		* @return Triple at that index
		*/
		Triple get(uint64_t index);

		/**
		* returns a reference to a particular element by its list index
		*
		* @param index returns a pointer to the triple at the specified index
		* @return Triple returns a pointer to the triple at the specified index
		*/
		Triple* getPtr(uint64_t index);

		/**
		* performs an 0(n) bucket sort. good for when the list is a mess
		* metric is vector distance to parameter in 2D
		*
		* @param t Triple that the whole list will be sorted against
		*/
		void bucketSort(Triple t);

		/**
		* performs an n^2 insertion sort. good for when the list is mostly in order
		* metric is vector distance to parameter in 2D
		*
		* @param t Triple that the whole list will be sorted against
		*/
		void sort(Triple t);

		/**
		* length of the current coordinate list
		*
		* @return length of the current coordinate list
		*/
		uint64_t getLength();

	private:
		/**
		* changes all the triples in the list to be of type Cartesian
		*/
		void toCartesian();

		/**
		* the capacity of the coordinates (not the number of actual elements stored)
		*/
		uint64_t length;

		/**
		* the data stored in the structure
		*/
		Triple* coordinates;

		/**
		* helper function to compute squared distance between two points in 2D
		*/
		float dist2(Triple a, Triple b);

		/**
		* function to print distances (for sort debugging)
		*/
		void log_distances(Triple origin);
};

#endif  // SRC_COORDINATE_LIST_H_

