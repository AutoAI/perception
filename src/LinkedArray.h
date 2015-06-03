#pragma once

template <typename T>
class LinkedArray {
    public:
        T* array;
        size_t* pointers;
        size_t head;
        size_t swap;
        size_t length;

        LinkedArray(size_t length);

        void set(size_t index, T value);
        void setAbsolute(size_t index, T value);

        T get(size_t index);
        T getAbsolute(size_t index);
};

