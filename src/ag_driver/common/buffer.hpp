
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace asensing
{
namespace lidar
{
class Buffer
{
public:

  Buffer(size_t buf_size)
    : data_off_(0), data_size_(0)
  {
    buf_.resize(buf_size);
    buf_size_ = buf_size;
  }

  ~Buffer() = default;

  uint8_t* buf()
  {
    return buf_.data();
  }

  size_t bufSize() const
  {
    return buf_size_;
  }

  uint8_t* data()
  {
    return buf() + data_off_;
  }

  size_t dataSize() const
  {
    return data_size_;
  }

  void setData(size_t data_off, size_t data_size)
  {
    data_off_ = data_off;
    data_size_ = data_size;
  }

private:
  std::vector<uint8_t> buf_;
  size_t buf_size_;
  size_t data_off_;
  size_t data_size_;
};
}  // namespace lidar
}  // namespace asensing
