// Minimal cv::Mat: read-only view over a message buffer.
#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>
namespace cv {
class Mat {
public:
  int rows{0}, cols{0};
  const uint8_t * data{nullptr};
  size_t elem{1};
  size_t total_bytes{0};
  bool empty() const { return data == nullptr || rows == 0 || cols == 0; }
  // Like OpenCV in release mode: address = base + (r*cols+c)*elemSize, then
  // reinterpret. Out-of-buffer reads return 0 instead of crashing the test.
  template<class T> T at(int r, int c) const {
    T v{};
    const size_t off = (static_cast<size_t>(r) * cols + c) * elem;
    if (off + sizeof(T) <= total_bytes) std::memcpy(&v, data + off, sizeof(T));
    return v;
  }
};
}  // namespace cv
