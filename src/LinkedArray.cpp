// LinkedArray.cpp
// Travis Vanderstad

// If you're looking for the comments, they're in LinkedArray.h

#pragma once

#include "LinkedArray.h"

LinkedArray::LinkedArray(size_t length) {
    array = new Triple[length];
    pointers = new size_t[length];
    head = 0;
    this->length = length;
    for(size_t i = 0; i < length; i++)
        pointers[i] = i + 1;
}

size_t LinkedArray::getLength() {
    return length;
}

void LinkedArray::set(size_t index, Triple value) {
    size_t resultIndex = head;
    for(size_t i = 0; i < index; i++)
        resultIndex = pointers[resultIndex];
    array[resultIndex] = value;
}

void LinkedArray::setAbsolute(size_t index, Triple value) {
    array[index] = value;
}

void LinkedArray::reset() {
    state = head;
}

Triple LinkedArray::get(size_t index) {
    size_t resultIndex = head;
    for(size_t i = 0; i < index; i++)
        resultIndex = pointers[resultIndex];
    return array[resultIndex];
}

Triple LinkedArray::getAbsolute(size_t index) {
    return array[index];
}

Triple LinkedArray::next() {
    Triple result = array[state];
    state = pointers[state];
    return result;
}

void LinkedArray::sort(Triple t) {

}

void LinkedArray::resort(Triple t) {
    
}
