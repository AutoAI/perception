#pragma once

template <typename T>
class NdArray {
    public:
        T* array;
        unsigned long* sizes;
        unsigned long length;
        char numDimensions;

        NdArray(char numDimensions, char* dimensions);

        void set(char* dimensions, T value);

        T get(char* dimensions);
};

