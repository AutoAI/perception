#pragma once

template <typename T>
class NdArray {
    public:
        T* array;
        size_t* sizes;
        size_t length;
        char numDimensions;

        NdArray(char numDimensions, size_t* dimensions);

        void set(size_t* dimensions, T value);
        void set(size_t index, T value);

        T get(size_t* dimensions);
        T get(size_t index);
};

