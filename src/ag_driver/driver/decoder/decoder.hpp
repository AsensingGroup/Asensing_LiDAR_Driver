#pragma once

#include "ag_driver/common/error_code.hpp"
#include "ag_driver/driver/driver_param.hpp"
#include "ag_driver/driver/decoder/member_checker.hpp"
#include "ag_driver/driver/decoder/trigon.hpp"
#include "ag_driver/driver/decoder/section.hpp"
#include "ag_driver/driver/decoder/basic_attr.hpp"
#include "ag_driver/common/sync_queue.hpp"
#include "ag_driver/common/json.hpp"

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES // for VC++, required to use const M_IP in <math.h>
#endif

#ifdef ENABLE_TRANSFORM
// Eigen lib
#include <Eigen/Dense>
#endif

#include <cmath>
#include <functional>
#include <memory>
#include <iomanip>

namespace asensing
{
namespace lidar
{

#pragma pack(push, 1)

#pragma pack(pop)

// Echo mode
enum AGEchoMode
{
  ECHO_SINGLE = 0,
  ECHO_DUAL
};

// decoder const param
struct AGDecoderConstParam
{
  // packet len
  uint16_t MSOP_LEN;
  uint16_t DIFOP_LEN;

  // packet identity
  uint8_t MSOP_ID_LEN;
  uint8_t DIFOP_ID_LEN;
  uint8_t MSOP_ID[8];
  uint8_t DIFOP_ID[8];

  // packet structure
  uint16_t LASER_NUM;
  uint16_t BLOCKS_PER_PKT;
  uint16_t CHANNELS_PER_BLOCK;

  // distance & temperature
  float DISTANCE_MIN;
  float DISTANCE_MAX;
  float DISTANCE_RES;
  float AZIMUTH_MIN;
  float AZIMUTH_MAX;
  float ELEVATION_MIN;
  float ELEVATION_MAX;
  uint8_t INTENSITY_MIN;
  uint8_t INTENSITY_MAX;
  uint8_t MODULE_MIN;
  uint8_t MODULE_MAX;
  float TEMPERATURE_RES;
};

#define INIT_ONLY_ONCE() \
  static bool init_flag = false; \
  if (init_flag) return param; \
  init_flag = true;

template <typename T_PointCloud>
class Decoder
{
public:

#ifdef ENABLE_TRANSFORM
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif

  virtual void decodeDifopPkt(const uint8_t* pkt, size_t size) = 0;
  virtual bool decodeMsopPkt(const uint8_t* pkt, size_t size) = 0;

  virtual ~Decoder() = default;

  void processDifopPkt(const uint8_t* pkt, size_t size);
  bool processMsopPkt(const uint8_t* pkt, size_t size);

  explicit Decoder(const AGDecoderConstParam& const_param, const AGDecoderParam& param);

  float getTemperature();
  bool getDeviceInfo(DeviceInfo& info);
  bool getDeviceStatus(DeviceStatus& status);
  bool getSelectionParam(SelectionParam& param);
  void setSelectionParam(SelectionParam& param);
  void setDefaultSelectionParam();
  double getPacketDuration();
  void enableWritePktTs(bool value);
  double prevPktTs();
  void transformPoint(float& x, float& y, float& z);

  void regCallback(
      const std::function<void(const Error&)>& cb_excep,
      const std::function<void(uint16_t, double)>& cb_split_frame);

  std::shared_ptr<T_PointCloud> point_cloud_; // accumulated point cloud currently

#ifndef UNIT_TEST
protected:
#endif

  double cloudTs(); //use which point's timestamp (first or last)

  AGDecoderConstParam const_param_; // const param
  AGDecoderParam param_; // user param
  std::function<void(uint16_t, double)> cb_split_frame_;
  std::function<void(const Error&)> cb_excep_;
  //bool write_pkt_ts_;

#ifdef ENABLE_TRANSFORM
  Eigen::Matrix4d trans_;
#endif

  Trigon trigon_;
#define SIN(angle) this->trigon_.sin(angle)
#define COS(angle) this->trigon_.cos(angle)

  double packet_duration_;//MSOP Packet's duration
  DistanceSection  distance_section_;  // invalid section of distance
  AngleSection     angle_section_;     // invalid section of azimuth and elevation
  IntensitySection intensity_section_; // invalid section of intensity
  ModuleSection    module_section_;    // invalid section of module

  AGEchoMode echo_mode_; // echo mode (defined by return mode)
  float temperature_; // lidar temperature
  DeviceInfo device_info_;
  DeviceStatus device_status_;

  bool angles_ready_; // is vert_angles/horiz_angles ready from csv file/difop packet?
  double prev_pkt_ts_; // timestamp of prevous packet
  double prev_point_ts_; // timestamp of previous point
  double first_point_ts_; // timestamp of first point

  bool IsCalibrated;
  bool CalibEnabled;

};

template <typename T_PointCloud>
inline void Decoder<T_PointCloud>::regCallback(
    const std::function<void(const Error&)>& cb_excep,
    const std::function<void(uint16_t, double)>& cb_split_frame)
{
  cb_excep_ = cb_excep;
  cb_split_frame_ = cb_split_frame;
}

template <typename T_PointCloud>
inline Decoder<T_PointCloud>::Decoder(const AGDecoderConstParam& const_param, const AGDecoderParam& param)
  : const_param_(const_param)
  , param_(param)
  //, write_pkt_ts_(false)
  , packet_duration_(0)
  , distance_section_(const_param.DISTANCE_MIN, const_param.DISTANCE_MAX, param.min_distance, param.max_distance)
  , angle_section_(const_param.AZIMUTH_MIN, const_param.AZIMUTH_MAX, const_param.ELEVATION_MIN, const_param.ELEVATION_MAX)
  , intensity_section_(const_param.INTENSITY_MIN, const_param.INTENSITY_MAX)
  , module_section_(const_param.MODULE_MIN, const_param.MODULE_MAX)
  , echo_mode_(ECHO_SINGLE)
  , temperature_(0.0)
  , angles_ready_(false)
  , prev_pkt_ts_(0.0)
  , prev_point_ts_(0.0)
  , first_point_ts_(0.0)
  
  , IsCalibrated(false)
  , CalibEnabled(true)
{
#ifdef ENABLE_TRANSFORM
  Eigen::AngleAxisd current_rotation_x(param_.transform_param.roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd current_rotation_y(param_.transform_param.elevation, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd current_rotation_z(param_.transform_param.azimuth, Eigen::Vector3d::UnitZ());
  Eigen::Translation3d current_translation(param_.transform_param.x, param_.transform_param.y,
                                           param_.transform_param.z);
  trans_ = (current_translation * current_rotation_z * current_rotation_y * current_rotation_x).matrix();  
#endif
}

template <typename T_PointCloud>
inline void Decoder<T_PointCloud>::enableWritePktTs(bool value)
{
  //write_pkt_ts_ = value;
}

template <typename T_PointCloud>
inline float Decoder<T_PointCloud>::getTemperature()
{
  return temperature_;
}

template <typename T_PointCloud>
inline bool Decoder<T_PointCloud>::getDeviceInfo(DeviceInfo& info)
{
  memcpy (&info, &device_info_, sizeof(DeviceInfo));
  return true;
}

template <typename T_PointCloud>
inline bool Decoder<T_PointCloud>::getDeviceStatus(DeviceStatus& status)
{
  memcpy (&status, &device_status_, sizeof(DeviceStatus));
  return true;
}

template <typename T_PointCloud>
inline bool Decoder<T_PointCloud>::getSelectionParam(SelectionParam& param)
{
  param.min_distance = param_.min_distance;
  param.max_distance = param_.max_distance;
  param.min_azimuth = param_.min_azimuth;
  param.max_azimuth = param_.max_azimuth;
  param.min_elevation = param_.min_elevation;
  param.max_elevation = param_.max_elevation;
  param.min_intensity = param_.min_intensity;
  param.max_intensity = param_.max_intensity;
  param.min_module = param_.min_module;
  param.max_module = param_.max_module;
  return true;
}

template <typename T_PointCloud>
inline void Decoder<T_PointCloud>::setSelectionParam(SelectionParam& param)
{
  distance_section_.updateDistanceSection(param.min_distance, param.max_distance);
  angle_section_.updateAngleSection(param.min_azimuth, param.max_azimuth, param.min_elevation, param.max_elevation);
  intensity_section_.updateIntensitySection(param.min_intensity, param.max_intensity);
  module_section_.updateModuleSection(param.min_module, param.max_module);
}

template <typename T_PointCloud>
inline void Decoder<T_PointCloud>::setDefaultSelectionParam()
{
  param_.min_distance = const_param_.DISTANCE_MIN;
  param_.max_distance = const_param_.DISTANCE_MAX;
  param_.min_azimuth = const_param_.AZIMUTH_MIN;
  param_.max_azimuth = const_param_.AZIMUTH_MAX;
  param_.min_elevation = const_param_.ELEVATION_MIN;
  param_.max_elevation = const_param_.ELEVATION_MAX;
  param_.min_intensity = const_param_.INTENSITY_MIN;
  param_.max_intensity = const_param_.INTENSITY_MAX;
  param_.min_module = const_param_.MODULE_MIN;
  param_.max_module = const_param_.MODULE_MAX;

  distance_section_.updateDistanceSection(const_param_.DISTANCE_MIN, const_param_.DISTANCE_MAX);
  angle_section_.updateAngleSection(const_param_.AZIMUTH_MIN, const_param_.AZIMUTH_MAX, const_param_.ELEVATION_MIN, const_param_.ELEVATION_MAX);
  intensity_section_.updateIntensitySection(const_param_.INTENSITY_MIN, const_param_.INTENSITY_MAX);
  module_section_.updateModuleSection(const_param_.MODULE_MIN, const_param_.MODULE_MAX);
}

template <typename T_PointCloud>
inline double Decoder<T_PointCloud>::getPacketDuration()
{
  return packet_duration_;
}

template <typename T_PointCloud>
inline double Decoder<T_PointCloud>::prevPktTs()
{
  return prev_pkt_ts_;
}

template <typename T_PointCloud>
inline double Decoder<T_PointCloud>::cloudTs()
{
  return (param_.ts_first_point ? first_point_ts_ : prev_point_ts_);
}

template <typename T_PointCloud>
inline void Decoder<T_PointCloud>::transformPoint(float& x, float& y, float& z)
{
#ifdef ENABLE_TRANSFORM
  Eigen::Vector4d target_ori(x, y, z, 1);
  Eigen::Vector4d target_rotate = trans_ * target_ori;
  x = target_rotate(0);
  y = target_rotate(1);
  z = target_rotate(2);
#endif
}

template <typename T_PointCloud>
inline void Decoder<T_PointCloud>::processDifopPkt(const uint8_t* pkt, size_t size)
{
  if (size != this->const_param_.DIFOP_LEN)
  {
     LIMIT_CALL(this->cb_excep_(Error(ERRCODE_WRONGDIFOPLEN)), 1);
     return;
  }

  if (memcmp(pkt, this->const_param_.DIFOP_ID, const_param_.DIFOP_ID_LEN) != 0)
  {
    LIMIT_CALL(this->cb_excep_(Error(ERRCODE_WRONGDIFOPID)), 1);
    return;
  }

  decodeDifopPkt(pkt, size);
}

template <typename T_PointCloud>
inline bool Decoder<T_PointCloud>::processMsopPkt(const uint8_t* pkt, size_t size)
{
  constexpr static int CLOUD_POINT_MAX = 1000000;

  if (this->point_cloud_ && (this->point_cloud_->points.size() > CLOUD_POINT_MAX))
  {
     LIMIT_CALL(this->cb_excep_(Error(ERRCODE_CLOUDOVERFLOW)), 1);
     this->point_cloud_->points.clear();
  }

  if (param_.wait_for_difop && !angles_ready_)
  {
     DELAY_LIMIT_CALL(cb_excep_(Error(ERRCODE_NODIFOPRECV)), 1);
     return false;
  }

  if (size != this->const_param_.MSOP_LEN)
  {
     LIMIT_CALL(this->cb_excep_(Error(ERRCODE_WRONGMSOPLEN)), 1);
     return false;
  }

  if (memcmp(pkt, this->const_param_.MSOP_ID, this->const_param_.MSOP_ID_LEN) != 0)
  {
    LIMIT_CALL(this->cb_excep_(Error(ERRCODE_WRONGMSOPID)), 1);
    return false;
  }

#ifdef ENABLE_CRC32_CHECK
  if (!isCrc32Correct(pkt, size))
  {
    LIMIT_CALL(this->cb_excep_(Error(ERRCODE_WRONGCRC32)), 1);
    return false;
  }
#endif

  return decodeMsopPkt(pkt, size);
}

}  // namespace lidar
}  // namespace asensing
