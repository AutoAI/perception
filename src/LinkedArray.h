// LinkedArray.cpp
// @author: Travis Vanderstad

// LinkedArray is a hybrid LinkedList-ArrayList that supports both types of indexing
// This allows for fast reordering and random access, but the list is fixed in size

#pragma once

// represents a 3d data point
struct Triple {
    float x;
    float y;
    float z;
};

class LinkedArray {
    public:
        // the capacity of the array (not the number of actual elements stored)
        size_t length;

        // origin for comparisons (sorts will be based on distance to this point)
        Triple origin;

        // all you need to make one is the length. the length cannont be changed after making it
        LinkedArray(size_t length);

        // set a particular element by its list index
        void set(size_t index, Triple value);

        // returns a particular element by its list index
        Triple get(size_t index);

        // performs an nlogn iterative merge sort. good for when the array is a mess
        // metric is vector distance to parameter to 2D
        void sort();

        // performs an n^2 insertion sort. good for when the list is mostly in order
        // metric is vector distance to parameter to 2D
        void resort(Triple t);

        size_t getLength();
    private:
        // the capacity of the array (not the number of actual elements stored)
        size_t length;

        // the data stored in the structure
        Triple* array;

        // helper function to merge [l..m] and [m+1..r]
        void merge(size_t l, size_t m, size_t r);

        // helper function to compute squared distance between two points in 2D
        float dist2(Triple a, Triple b);
};

