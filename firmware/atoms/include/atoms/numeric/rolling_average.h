#pragma once

// This file is part of 'Atoms' library - https://github.com/yaqwsx/atoms
// Author: Jan 'yaqwsx' Mr�zek

#include <array>
#include <initializer_list>
#include <algorithm>
#include <vector>

namespace atoms {

template <class T, size_t SIZE>
class RollingAverage {
public:
    RollingAverage() : values( SIZE, T( 0 ) ), sum(0), index(0)
    {
        // std::fill(values.begin(), values.end(), T(0));
    };

    void push(const T& t) {
        sum += t - values[index];
        values[index] = t;
        index++;
        if (index == SIZE)
            index = 0;
    }

    T get_average() {
        return sum / T(SIZE);
    }

    T get_sum() {
        return sum;
    }

    void clear(T t = 0) {
        std::fill(values.begin(), values.end(), t);
        sum = t * SIZE;
    }

private:
    std::vector< T > values;
    T sum;
    size_t index;
};

}
