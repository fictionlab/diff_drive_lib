#pragma once

#include <cstddef>
#include <atomic>

namespace diff_drive_lib {

template <class T>
class CircularBuffer {
  T* values_;
  size_t size_;
  size_t iter_;
  std::atomic_bool dynamicAlloc_;

 public:
  explicit CircularBuffer(size_t size)
      : values_(new T[size]()), size_(size), iter_(0), dynamicAlloc_(true) {}

  CircularBuffer(size_t size, T* buffer)
      : values_(buffer), size_(size), iter_(0), dynamicAlloc_(false) {}

  ~CircularBuffer() {
    if (dynamicAlloc_.load())
      delete values_;
  }

  T push_back(T val) {
    T tmp = values_[iter_];
    values_[iter_++] = val;
    if (iter_ >= size_) iter_ = 0;
    return tmp;
  }
};

}  // namespace diff_drive_lib