// NdArray.cpp
// @author: Travis Vanderstad, Steven Russo

// NdArray represents an n-dimensional array of whatever type you like

#pragma once

template <typename T>
class NdArray {
  public:
  T* array;
  unsigned long* sizes;
  unsigned long length;
  char numDimensions;

  NdArray(char numDimensions, unsigned long* dimensions);

  void set(unsigned long* dimensions, T value);
  void set(unsigned long index, T value);

  T get(unsigned long* dimensions);
  T get(unsigned long index);
  // TODO: make things that should be private private and make necessary setters + getters
};

