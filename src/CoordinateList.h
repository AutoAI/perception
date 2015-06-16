// CoordinateList.h
// @author: Travis Vanderstad, Parth Mehrotra, Matthew Bowyer

// CoordinateList is a list of coordinates
// Supports changing coodrinate system of all coordinates and sorting base on 2D distance from a point

#pragma once

#include <vector>
#include "Triple.h"
#include "GlobalConstants.h"

#define Cartesian 0
#define Spherical 1
#define Perspective 2

using namespace std;

//List of all coordinate triples
class CoordinateList {
    public:
        // type of the coodinates (as #define'd)
        char type;

        // contruct using a type and initial length
        CoordinateList(char type, unsigned long length);

        // converts all coordinates to the specified type
        void toType(char newType);

        // converts all coodinates to the specified type using an offset to the new origin
        void toType(char newType, Triple offset);

        // set a particular element by its list index
        void set(unsigned long index, Triple value);

        // returns a particular element by its list index
        Triple get(unsigned long index);

        // returns a reference to a particular element by its list index
        Triple* getPtr(unsigned long index);

        // performs an 0(n) bucket sort. good for when the list is a mess
        // metric is vector distance to parameter in 2D
        void sortThatDoesntWorkYet(Triple t);

        // performs an n^2 insertion sort. good for when the list is mostly in order
        // metric is vector distance to parameter in 2D
        void sort(Triple t);

        unsigned long getLength();
    private:
        // helper method for toType()'s
        void toCartesian();

        // the capacity of the coordinates (not the number of actual elements stored)
        unsigned long length;

        // the data stored in the structure
        Triple* coordinates;

        // helper function to compute squared distance between two points in 2D
        float dist2(Triple a, Triple b);

        // function to print distances (for sort debugging)
        void log_distances(Triple origin);
};
