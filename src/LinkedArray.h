// LinkedArray.cpp
// Travis Vanderstad

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

        // the index of the current element for iterating. use reset to make it the first element again
        size_t state;

        // all you need to make one is the length. the length cannont be changed after making it
        LinkedArray(size_t length);

        // set a particular element by its list index
        void set(size_t index, Triple value);

        // set a particular element by its array index
        void setAbsolute(size_t index, Triple value);

        // reset the iteration to the beginning. you should probably reset after making any changes
        void reset();

        // returns a particular element by its list index
        Triple get(size_t index);

        // returns a particular element by its array index
        Triple getAbsolute(size_t index);

        // to iterate, call this. returns the curent element and increments state
        Triple next();

        // performs an nlogn iterative merge sort. good for when the array is a mess
        // metric is vector distance to parameter to 2D
        void sort(Triple t);

        // performs an n^2 insertion sort. good for when the list is mostly in order
        // metric is vector distance to parameter to 2D
        void resort(Triple t);

        // straightens out the array so that the ith element of the list is array[i] (for 0(1) access in order)
        void straighten();

    private:
        // the data stored in the structure
        Triple* array;
        
        // an array of indices. the element after array[i] is array[pointers[i]]
        size_t* pointers;
        
        // the index of the first element
        size_t head;

        // a value for temporary use. assumed to be garbage at any time
        size_t swap;
};

