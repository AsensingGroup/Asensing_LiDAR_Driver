#pragma once

#include "ag_driver/driver/decoder/decoder.hpp"

#include "ag_driver/driver/decoder/A0/decoder_A0.hpp"
#include "ag_driver/driver/decoder/A2/decoder_A2.hpp"


namespace asensing
{
namespace lidar
{

template <typename T_PointCloud>
class DecoderFactory
{
public:

  static std::shared_ptr<Decoder<T_PointCloud>> createDecoder(
      LidarType type, const AGDecoderParam& param);
};

template <typename T_PointCloud>
inline std::shared_ptr<Decoder<T_PointCloud>> DecoderFactory<T_PointCloud>::createDecoder(
    LidarType type, const AGDecoderParam& param)
{
  std::shared_ptr<Decoder<T_PointCloud>> ret_ptr;

  switch (type)
  {
    case LidarType::A0:
      ret_ptr = std::make_shared<DecoderA0<T_PointCloud>>(param);
      break;
    case LidarType::A2:
      ret_ptr = std::make_shared<DecoderA2<T_PointCloud>>(param);
      break;  
    default:
      AG_ERROR << "Wrong LiDAR Type. Please check your LiDAR Version! " << AG_REND;
      ret_ptr = nullptr;
  }

  return ret_ptr;
}

}  // namespace lidar
}  // namespace asensing
