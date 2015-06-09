// LinkedArray.cpp
// Travis Vanderstad

// If you're looking for the comments, they're in LinkedArray.h

#pragma once

#include <algorithm>

#include "LinkedArray.h"

LinkedArray::LinkedArray(size_t length) {
    array = new Triple[length];
}

void LinkedArray::set(size_t index, Triple value) {
    array[index] = value;
}

Triple LinkedArray::get(size_t index) {
    return array[index];
}

void LinkedArray::sort() {
    size_t mid, right, n1, n2, prev, head;
    Triple* temp = new Triple[length];
    size_t* pointers = new size_t[length];
    for(size_t size = 1; size < length; size *= 2){
        for(size_t left = 0; left < length-1; left += 2*size){
            mid = left + size - 1;
            right = std::min(left + 2*size - 1, length-1);
            n1 = left;
            n2 = mid + 1;
            // merge
            if(dist2(array[n1], origin) < dist2(array[n2], origin)){
                head = n1;
                prev = n1++;
            }else{
                head = n2;
                prev = n2++;
            }
            while(n1 <= mid && n2 <= right){
                if(dist2(array[n1], origin) < dist2(array[n2], origin)){
                    pointers[prev] = n1;
                    prev = n1++;
                }else{
                    pointers[prev] = n2;
                    prev = n2++;
                }
            }
            while(n1 <= mid){
                pointers[prev] = n1;
                prev = n1++;
            }
            while(n2 <= right){
                pointers[prev] = n2;
                prev = n2++;
            }
            size_t c = head;
            for(size_t i = left; i <= right; i++){
                temp[i] = array[c];
                c = pointers[c];
            }
        }
    }
}

float LinkedArray::dist2(Triple a, Triple b){
    return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y);
}

void LinkedArray::resort(Triple t) {
    
}