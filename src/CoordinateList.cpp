// CoordinateList.cpp
// Travis Vanderstad

// If you're looking for the comments, they're in CoordinateList.h

#pragma once

#include "ros/ros.h"
#include "ros/console.h"

#include <stdio.h>
#include <algorithm>
#include <limits>
#include <math.h>
#include <string.h>

#include "NdArray.cpp"
#include "CoordinateList.h"

using namespace std;

CoordinateList::CoordinateList(char type, size_t length) {
    this->type = type;
    this->length = length;
    coordinates = new Triple[length];
}

void CoordinateList::toType(char newType) {
    Triple d;
    d.x = d.y = d.z = 0;
    this->toType(newType, d); 
}

void CoordinateList::toCartesian(){
    if(type==Cartesian){
        return;
    }

    float r, theta, phi;
    float u, v, w;
    float x, y, z;

    for(int i = 0; i < length; i++) {
        if(type==Spherical){
            r = coordinates[i].x;
            theta =  coordinates[i].y;
            phi = coordinates[i].z;
            
            x = r*sin(phi)*cos(theta);
            y = r*sin(phi)*sin(theta);
            z = r*cos(phi);

            ROS_INFO("if i=%i, x=%f, y=%f, z=%f", i, x, y, z);
        }else{
            u = coordinates[i].x;
            v = coordinates[i].y;
            w = coordinates[i].z;

            z = K/w;
            y = v*z/F;
            x = u*z/F;
            ROS_INFO("else i=%i, x=%f, y=%f, z=%f", i, x, y, z);

        }
        coordinates[i].x = x;
        coordinates[i].y = y;
        coordinates[i].z = z;
    }
    type = Cartesian;
}

void CoordinateList::toType(char newType, Triple offset) {

	float r, theta, phi;
	float u, v, w;
	float x, y, z;

    if(newType==type)
        return;

    this->toCartesian();
    for(int i = 0; i < length; i++){
       
        coordinates[i].x += offset.x;
        coordinates[i].y += offset.y;
        coordinates[i].z += offset.z;
        
        if(newType==Spherical){
            r = sqrt(coordinates[i].x*coordinates[i].x + coordinates[i].y*coordinates[i].y + coordinates[i].z*coordinates[i].z);
            theta = atan(coordinates[i].y/coordinates[i].x);
            phi = acos(coordinates[i].z/r);

            coordinates[i].x = r;
            coordinates[i].y = theta;
            coordinates[i].z = phi;
        }

        if(newType==Perspective){
            u = coordinates[i].x*F/coordinates[i].z;
            v = coordinates[i].y*F/coordinates[i].z;
            w = K/coordinates[i].z;
           
            coordinates[i].x = u;
            coordinates[i].y = v;
            coordinates[i].z = w;
        }
    }
    type = newType;
}

Triple CoordinateList::get(size_t index) {
    return coordinates[index];
}

void CoordinateList::set(size_t index, Triple value) {
    coordinates[index] = value;
}

size_t CoordinateList::getLength() {
    return length;
}

void CoordinateList::sortThatDoesntWorkYet(Triple origin) {
    size_t num_buckets = 2;
    // Pre-calculate all distances (so we dont have to do it every comparison)
    float distances[length];
    for(size_t i = 0; i < length; i++)
        distances[i] = dist2(origin, coordinates[i]);
    // Find min and max distances (for bucket partitioning)
    float max = 0;
    float min = std::numeric_limits<float>::max();
    for(size_t i = 0; i < length; i++){
        max = (max < distances[i]) ? distances[i] : max;
        min = (min > distances[i]) ? distances[i] : min;
    }
    // Figure out how big each bucket is going to be
    size_t bucket_sizes[num_buckets];
    for(size_t i = 0; i < num_buckets; i++) {
        bucket_sizes[i] = 0;
    }
    float d = (max - min)/num_buckets;
    for(size_t i = 0; i < length; i++){
        bucket_sizes[(size_t)floor((distances[i]-min)/d)]++;
    }
    // Figure out how big the largest bucket will be
    size_t max_size = 0;
    for(size_t i = 0; i < num_buckets; i++){
        max_size = (max_size < bucket_sizes[i]) ? bucket_sizes[i] : max_size;
    }
    // Set up the buckets as an NdArray
    size_t bounds[2] = {num_buckets, max_size};
    NdArray<Triple> buckets(2, bounds);
    // Put things in the buckets
    size_t indices [num_buckets];
    for(size_t i = 0; i < num_buckets; i++)
        indices[i] = 0;
    for(size_t i = 0; i < length; i++){
        size_t bucket_index = (floor((distances[i]-min)/d) < num_buckets - 1) ? floor((distances[i]-min)/d) : num_buckets - 1;
        size_t index[2] = {bucket_index, indices[bucket_index]};
        buckets.set(index, coordinates[i]);
        indices[bucket_index]++;
    }
    // Copy the things back into the original array
    size_t i = 0;
    for(size_t j = 0; j < num_buckets; j++)
        for(size_t k = 0; k < bucket_sizes[j]; k++){
            size_t index[2] = {i, j};
            coordinates[i++] = buckets.get(index);
        }
    // The array is now mostly in order. Do an insertion sort.
    sort(origin);
}

float CoordinateList::dist2(Triple a, Triple b){
    return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y);
}

void CoordinateList::sort(Triple origin) {
    for(size_t i = 1; i < length; i++){
        size_t j = i;
        while(dist2(coordinates[j], origin) < dist2(coordinates[j-1], origin) && j > 0){
            Triple temp = coordinates[j];
            coordinates[j] = coordinates[j-1];
            coordinates[j-1] = temp;
            j--;
        }
    }
}

void CoordinateList::log_distances(Triple origin){
    stringstream out;
    out << dist2(coordinates[0], origin);
    for(size_t i = 1; i < length; i++)
        out << " " << dist2(coordinates[i],origin);
    ROS_INFO("%s", out.str().c_str());
}