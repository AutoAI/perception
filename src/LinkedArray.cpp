#pragma once

#include "LinkedArray.h"

template<typename T>
LinkedArray<T>::LinkedArray(size_t length) {
    array = new T[length];
    pointers = new size_t[length];
    head = 0;
    this->length = length;
}

template<typename T>
void LinkedArray<T>::set(size_t index, T value) {
    size_t resultIndex = head;
    for(size_t i = 0; i < index; i++)
        resultIndex = pointers[resultIndex];
    array[resultIndex] = value;
}

template<typename T>
void LinkedArray<T>::setAbsolute(size_t index, T value) {
    array[index] = T;
}

template<typename T>
void LinkedArray<T>::reset() {
    state = 0;
}

template<typename T>
T LinkedArray<T>::get(size_t index) {
    size_t resultIndex = head;
    for(size_t i = 0; i < index; i++)
        resultIndex = pointers[resultIndex];
    return array[resultIndex];
}

template<typename T>
T LinkedArray<T>::getAbsolute(size_t index) {
    return array[index];
}

template<typename T>
T LinkedArray<T>::next() {
    T result = array[state];
    state = pointers[state];
    return result;
}

template<typename T>
void LinkedArray<T>::sort {
}

template<typename T>
void LinkedArray<T>::resort {
}