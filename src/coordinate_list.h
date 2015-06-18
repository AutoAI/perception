/**
* CoordinateList.h
*
* CoordinateList is a list of coordinates
* Supports changing coodrinate system of all coordinates and sorting base on 2D distance from a point
*
* @author: Travis Vanderstad, Parth Mehrotra, Matthew Bowyer
*/

#pragma once

#include <vector>
#include "triple.h"
#include "global_constants.h"

#define Cartesian 0
#define Spherical 1
#define Perspective 2

using namespace std;

/** List of all coordinate triples */
class CoordinateList {
    public:
        char type; /** type of the coordinate (as #define'd) */

		/**
		* Creates a CoordinateList using a type and an initial length
		* 
		* @param type a char that represents the type of the CoordinateList (Cartesian, Spherical, or Perspective)
		* @param length the length of the initial list
		*/
        CoordinateList(char type, unsigned long length);

        /** 
		* converts all coordinates to the specified type 
		* 
		* @param newType type to convert to
		*/
        void toType(char newType);

        /**
		* converts all coodinates to the specified type using an offset to the new origin
		*
		* @param newType new type for the list
		* @param offset a Triple offset to specify the new origin
		*/
        void toType(char newType, Triple offset);

        /**
		* set a particular element by its list index
		*
		* @param index the index that you're setting
		* @param value the Triple that it's being set too
		*/
        void set(unsigned long index, Triple value);

        /**
		* returns a particular element by its list index
		*
		* @param index returns Triple at index
		* @return Triple at that index
		*/
        Triple get(unsigned long index);

        /**
		* returns a reference to a particular element by its list index
		*
		* @param index returns a pointer to the triple at the specified index
		* @return Triple returns a pointer to the triple at the specified index
		*/
        Triple* getPtr(unsigned long index);

        /**
		* performs an 0(n) bucket sort. good for when the list is a mess
		* metric is vector distance to parameter in 2D
		* 
		* @param t Triple that the whole list will be sorted against
		*/
        void sortThatDoesntWorkYet(Triple t);

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
        unsigned long getLength();

    private:
		/**
		* changes all the triples in the list to be of type Cartesian
		*/
        void toCartesian();

        /**
		* the capacity of the coordinates (not the number of actual elements stored)
		*/
        unsigned long length;

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