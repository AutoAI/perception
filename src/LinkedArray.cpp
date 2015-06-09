// LinkedArray.cpp
// Travis Vanderstad

// If you're looking for the comments, they're in LinkedArray.h

#pragma once

#include <algorithm>
#include <limits>
#include <math.h>

#include "LinkedArray.h"
#include "NdArray.cpp"

LinkedArray::LinkedArray(size_t length) {
    array = new Triple[length];
}

size_t LinkedArray::getLength() {
    return length;
}

void LinkedArray::set(size_t index, Triple value) {
    array[index] = value;
}

Triple LinkedArray::get(size_t index) {
    return array[index];
}

void LinkedArray::sort(Triple origin) {
    size_t num_buckets = 100;
    // Pre-calculate all distances (so we dont have to do it every comparison)
    float distances[length];
    for(size_t i = 0; i < length; i++)
        distances[i] = dist2(origin, array[i]);
    // Find min and max distances (for bucket partitioning)
    float max = 0;
    float min = std::numeric_limits<float>::max();
    for(size_t i = 0; i < length; i++){
        max = (max<distances[i])?distances[i]:max;
        min = (min>distances[i])?distances[i]:min;
    }
    // Figure out how big each bucket is going to be
    size_t bucket_sizes[num_buckets];
    for(size_t i = 0; i < num_buckets; i++)
        bucket_sizes[i] = 0;
    float d = (max - min)/num_buckets;
    for(size_t i = 0; i < length; i++){
        bucket_sizes[(size_t)floor((distances[i]-min)/d)]++;
    }
    // Figure out how big the largest bucket will be
    size_t max_size = 0;
    for(size_t i = 0; i < length; i++){
        max_size = (max_size<bucket_sizes[i])?bucket_sizes[i]:max_size;
    }
    // Set up the buckets as an NdArray
    size_t bounds[2] = {num_buckets, max_size};
    NdArray<Triple> buckets(2, bounds);
    // Put things in the buckets
    size_t indices [num_buckets];
    for(size_t i = 0; i < num_buckets; i++)
        indices[i] = 0;
    for(size_t i = 0; i < length; i++){
        size_t index[2] = {floor((distances[i]-min)/d), indices[i]++};
        buckets.set(index, array[i]);
    }
    // Copy the things back into the original array
    size_t i = 0;
    for(size_t j = 0; j < num_buckets; j++)
        for(size_t k = 0; k < bucket_sizes[j]; k++){
            size_t index[2] = {i, j};
            array[i++] = buckets.get(index);
        }
    // The array is now mostly in order. Do an insertion sort.
    resort(origin);
}

float LinkedArray::dist2(Triple a, Triple b){
    return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y);
}

void LinkedArray::resort(Triple origin) {
    
}