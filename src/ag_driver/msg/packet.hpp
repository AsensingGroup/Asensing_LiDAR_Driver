#pragma once

#include <cstdint>
#include <vector>
#include <string>

namespace asensing
{
namespace lidar
{

struct Packet
{
  double timestamp = 0.0;
  uint32_t seq = 0;
  uint8_t is_difop = 0;
  uint8_t is_frame_begin = 0;
  std::string frame_id = "";  // Packet message frame id

  Packet(const Packet& msg)
  {
    buf_.assign(msg.buf_.begin(), msg.buf_.end());
  }

  Packet(size_t size = 0)
  {
    buf_.resize(size);
  }

  std::vector<uint8_t> buf_;
};

}  // namespace lidar
}  // namespace asensing
