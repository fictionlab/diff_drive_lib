#pragma once

#include <cstddef>

namespace diff_drive_lib {

template <class T, size_t SIZE>
class CircularBuffer {
  T values_[SIZE];
  size_t iter_ = 0;

 public:
  T push_back(T val) {
    T tmp = values_[iter_];
    values_[iter_++] = val;
    if (iter_ >= SIZE) iter_ = 0;
    return tmp;
  }
};

}  // namespace diff_drive_lib