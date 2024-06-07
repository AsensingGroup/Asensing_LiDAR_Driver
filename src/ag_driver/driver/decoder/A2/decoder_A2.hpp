#pragma once

#include <fstream>
#include "A2_struct.hpp"

using json = nlohmann::json;

namespace asensing
{
namespace lidar
{
template <typename T_PointCloud>
class DecoderA2 : public Decoder<T_PointCloud>
{
public:
  constexpr static double FRAME_DURATION = 0.1;
  // when single callback
  constexpr static uint32_t SINGLE_PKT_NUM = 999;

  virtual void decodeDifopPkt(const uint8_t *pkt, size_t size);
  virtual bool decodeMsopPkt(const uint8_t *pkt, size_t size);

  virtual ~DecoderA2() = default;
  explicit DecoderA2(const AGDecoderParam &param);

  void A2LoadCalibration(const std::string &filename);

private:
  static AGDecoderConstParam &getConstParam();
  AGEchoMode getEchoMode(uint8_t mode);

  SplitStrategyBySeq split_strategy_;

  std::string A2CalibrationFileName = "A2-Correction.json";
  const float elevation_angle_start_ = 12.5;  // 垂直俯仰角范围
  const float elevation_angle_step_ = -0.26;  // 垂直俯仰角步长
      
  float elevation_offset_[A2_CHANNELS_PER_BLOCK] = {0};
  float azimuth_offset_[A2_CHANNELS_PER_BLOCK] = {0};
  float elevation_mirror_offset_[A2_SIDES_OF_MIRROR] = {0};
  float azimuth_mirror_offset_[A2_SIDES_OF_MIRROR] = {0};
  bool elevation_mirror_offset_enable_ = false;
  bool azimuth_mirror_offset_enable_ = false;

  void printCalibrationData();
};

template <typename T_PointCloud>
inline AGDecoderConstParam &DecoderA2<T_PointCloud>::getConstParam()
{
  static AGDecoderConstParam param =
  {
    812, // msop len
    108, // difop len

    2, // msop id len
    4, // difop id len

    {0x55, 0xA2}, // msop id
    {0xA5, 0xFF, 0x00, 0x5A}, // difop id

    96, // laser number (equivalent number of scan lines)
    1,  // blocks per packet
    A2_CHANNELS_PER_BLOCK, // channels per block

    0.0f,       // distance min
    200.0f,     // distance max
    0.01f,      // distance resolution
    -36000.0f,  // azimuth min
    36000.0f,   // azimuth max
    -9000.0f,   // elevation min
    36000.0f,   // elevation max
    0,          // intensity min
    255,        // intensity max
    0,          // module min
    0,          // module max
    80.0f   // initial value of temperature
  };

  return param;
}

template <typename T_PointCloud>
inline DecoderA2<T_PointCloud>::DecoderA2(const AGDecoderParam &param)
  : Decoder<T_PointCloud>(getConstParam(), param)
{
  this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;

  this->A2LoadCalibration(A2CalibrationFileName);
  this->printCalibrationData();
}

template <typename T_PointCloud>
inline AGEchoMode DecoderA2<T_PointCloud>::getEchoMode(uint8_t mode)
{
  switch (mode & 0x01)
  {
    case 0x00:
      return AGEchoMode::ECHO_SINGLE;
    case 0x01:
      return AGEchoMode::ECHO_DUAL;
    default:
      return AGEchoMode::ECHO_SINGLE;
  }
}

template <typename T_PointCloud>
inline void DecoderA2<T_PointCloud>::decodeDifopPkt(const uint8_t *packet, size_t size)
{
  const A2DifopPkt &pkt = *(A2DifopPkt *)packet;
  this->echo_mode_ = this->getEchoMode(pkt.difop_head.return_mode);

#ifdef ENABLE_DIFOP_PARSE
  // device info
  memcpy(this->device_info_.sn, pkt.difop_head.sn, 6);
  memcpy(this->device_info_.mac, pkt.difop_head.eth.mac_addr, 6);
  memcpy(this->device_info_.top_ver, pkt.difop_head.version.firmware_ver, 6);

  // device status
  this->device_status_.voltage = ntohs(pkt.func_safety.lidar_status);
#endif
}

template <typename T_PointCloud>
inline bool DecoderA2<T_PointCloud>::decodeMsopPkt(const uint8_t *packet, size_t size)
{
  const A2MsopPkt &pkt = *(A2MsopPkt *)packet;
  bool ret = false;

  // this->temperature_ = static_cast<float>((int)pkt.header.temperature - this->const_param_.TEMPERATURE_RES);
  uint8_t mytem = 0x30;
  this->temperature_ = mytem;

  // something wrong
  double pkt_ts = 0;
  if (this->param_.use_lidar_clock)
  {
    pkt_ts = parseTimeWithAG(&pkt.header.Timestamp);
  }
  else
  {
    /*uint64_t ts = getTimeHost();

    // roll back to first block to approach lidar ts as near as possible.
    pkt_ts = getTimeHost() * 1e-6 - this->getPacketDuration();

    if (this->write_pkt_ts_)
    {
      createTimeUTCWithUs (ts, (AGTimestampUTC*)&pkt.header.timestamp);
    }*/

    pkt_ts = getTimeHostWithAG();

    if (this->param_.write_pkt_ts)
    {
      createTimeWithAG((AGTimestampUTC *)&pkt.header.Timestamp);
    }
  }

  uint32_t frame_id = pkt.header.FrameSn;
  // Because the packet sent by radar does not convert the multiple bytes to the size end. Therefore, no "ntohs" processing is required
  uint16_t pkt_seq = pkt.header.PktSeq;

  if (split_strategy_.newPacket(frame_id, pkt_seq))
  {
    this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());
    this->first_point_ts_ = pkt_ts;
    ret = true;
  }

  // LidarType
  uint8_t LidarType = pkt.header.LidarType;
  if (LidarType != 2)
  {
    // do something
  }

  // uint8_t ProtocolVersion = pkt.header.Version;
  // do something

  // unused variable
  // uint8_t LidarInfo = pkt.header.LidarInfo;
  // uint8_t LiDARFlag1 = pkt.header.LiDARFlag1;
  // uint8_t LiDARFlag2 = pkt.header.LiDARFlag2;

  float elevation_mirror_offset = static_cast<float>(pkt.header.ElevationMirrorOffset);
  uint8_t mirror_side = frame_id % A2_SIDES_OF_MIRROR;

  for (uint16_t blk = 0; blk < this->const_param_.BLOCKS_PER_PKT; blk++)
  {
    const A2MsopBlock &block = pkt.blocks[blk];

    double point_time = pkt_ts; // + block.time_offset * 1e-6;

    for (uint16_t chan = 0; chan < this->const_param_.CHANNELS_PER_BLOCK; chan++)
    {
      for (uint16_t echo = 0; echo < 2; echo++)
      {
        const A2MsopChannel &channel = block.channel[chan][echo];

        float distance = static_cast<float>(channel.distance) * this->const_param_.DISTANCE_RES;
        float azimuth = static_cast<float>(block.Azimuth + azimuth_offset_[chan] * 100);
        float elevation = static_cast<float>(elevation_offset_[chan] * 100);

        // Internal reference calibration compensation
        if (azimuth_mirror_offset_enable_) {
          azimuth += static_cast<float>(azimuth_mirror_offset_[mirror_side] * 100);
        }

        if (elevation_mirror_offset_enable_) {
          elevation += static_cast<float>(elevation_mirror_offset_[mirror_side] * 100);
        }
        else {
          elevation += elevation_mirror_offset;
        }

        // Transform & Filling
        if (this->distance_section_.in(distance))
        {
          float x = 0;
          float y = 0;
          float z = 0;

          x = distance * COS(elevation) * COS(azimuth);
          y = distance * COS(elevation) * SIN(azimuth);
          z = distance * SIN(elevation);

          this->transformPoint(x, y, z);

          uint8_t ring = uint8_t(chan);

          typename T_PointCloud::PointT point;
          setX(point, x);
          setY(point, y);
          setZ(point, z);
          setIntensity(point, channel.intensity);
          setTimestamp(point, point_time);
          setRing(point, ring);
          setRange(point, channel.distance);
#ifdef ENABLE_POINT_EXTEND_FIELD
          setAzimuth(point, (uint16_t)azimuth);
          setElevation(point, (uint16_t)elevation);
#endif
#ifdef COMPILE_TOOLS
          setFrameSn(point, frame_id);
#endif

          this->point_cloud_->points.emplace_back(point);
        }
        else if (!this->param_.dense_points)
        {
          uint8_t ring = (uint8_t)chan;

          typename T_PointCloud::PointT point;
          setX(point, NAN);
          setY(point, NAN);
          setZ(point, NAN);
          setIntensity(point, 0);
          setTimestamp(point, point_time);
          setRing(point, ring);
          setRange(point, 0);
#ifdef ENABLE_POINT_EXTEND_FIELD
          setAzimuth(point, (uint16_t)azimuth);
          setElevation(point, (uint16_t)elevation);
#endif
#ifdef COMPILE_TOOLS
          setFrameSn(point, frame_id);
#endif
          this->point_cloud_->points.emplace_back(point);
        }
      }
    }

    this->prev_point_ts_ = point_time;
  }

  this->prev_pkt_ts_ = pkt_ts;
  return ret;
}

template <typename T_PointCloud>
inline void DecoderA2<T_PointCloud>::A2LoadCalibration(const std::string &filename)
{
  if (filename.empty())
  {
    this->IsCalibrated = false;
    return;
  }

  std::string keyword("A2-Correction");
  std::size_t found = filename.find(keyword);
  if (std::string::npos == found)
  {
    AG_WARNING << "Do not need to correct data: " << filename << AG_REND;
    this->CalibEnabled = false;
    this->IsCalibrated = false;
    return;
  }

  // Correct point cloud
  std::string config_path;

#ifdef DRIVER_PROJECT_PATH
  config_path = (std::string)DRIVER_PROJECT_PATH;
  config_path += "/src/ag_driver/config/";
  config_path += filename;
#else
#ifdef RUN_IN_ROS1_OR_ROS2
  config_path = (std::string)PROJECT_PATH;
  config_path += "/src/driver_sdk/src/ag_driver/config/";
  config_path += filename;
#else
  config_path = "../../../src/ag_driver/config/"; // for the MSVS project(demo and tool)
  config_path += filename;
#endif
#endif

  std::ifstream f(config_path.c_str());

  if (!f.is_open())
  {
    AG_WARNING << "Can not open: " << config_path << AG_REND;
    this->IsCalibrated = false;
    this->CalibEnabled = true;
    return;
  }

  json data = json::parse(f);
  // AG_DEBUG << data.dump() << AG_REND;

  if (data["elevation"].is_array() && data["elevation"].size() == this->const_param_.CHANNELS_PER_BLOCK)
  {
    for (int i = 0; i < this->const_param_.CHANNELS_PER_BLOCK; i++)
    {
      elevation_offset_[i] = data["elevation"][i];
    }
    this->IsCalibrated = true;
  }
  else 
  {
    for (int i = 0; i < this->const_param_.CHANNELS_PER_BLOCK; i++)
    {
      elevation_offset_[i] = elevation_angle_start_ + elevation_angle_step_ * i;
    }
    this->IsCalibrated = false;
  }

  if (data["elevation_mirror_offset"].is_array() && data["elevation_mirror_offset"].size() == A2_SIDES_OF_MIRROR)
  {
    for (int i = 0; i < A2_SIDES_OF_MIRROR; i++)
    {
      elevation_mirror_offset_[i] = data["elevation_mirror_offset"][i];
      this->IsCalibrated = true;
    }
  }
  else {
    this->IsCalibrated = false;
  }

  if (data["elevation_mirror_offset_enable"].is_boolean())
  {
    elevation_mirror_offset_enable_ = data["elevation_mirror_offset_enable"];
  }

  if (data["LD_azimuth_offset"].is_array() && data["LD_azimuth_offset"].size() == this->const_param_.CHANNELS_PER_BLOCK)
  {
    for (int i = 0; i < this->const_param_.CHANNELS_PER_BLOCK; i++)
    {
      azimuth_offset_[i] = data["LD_azimuth_offset"][i];
    }
    this->IsCalibrated = true;
  }
  else {
    this->IsCalibrated = false;
  }

  if (data["azimuth_mirror_offset"].is_array() && data["azimuth_mirror_offset"].size() == A2_SIDES_OF_MIRROR)
  {
    for (int i = 0; i < A2_SIDES_OF_MIRROR; i++)
    {
      azimuth_mirror_offset_[i] = data["azimuth_mirror_offset"][i];
      this->IsCalibrated = true;
    }
  }
  else {
    this->IsCalibrated = false;
  }

  if (data["azimuth_mirror_offset_enable"].is_boolean())
  {
    azimuth_mirror_offset_enable_ = data["azimuth_mirror_offset_enable"];
  }

  data.clear();
  f.close();
}

template <typename T_PointCloud>
inline void DecoderA2<T_PointCloud>::printCalibrationData()
{
  AG_INFO << "------------------------------------------------------" << AG_REND;
  AG_INFO << "        Calibration File: " << A2CalibrationFileName << AG_REND;

  if (this->IsCalibrated) {
    AG_INFOL << "(calibrated)" << AG_REND;
  }
  else {
    AG_INFOL << "(uncalibrated, use default values)" << AG_REND;
  }

  AG_INFOL << "elevation offset :" << AG_REND;
  for (int i = 0; i < A2_CHANNELS_PER_BLOCK; i++)
  {
    AG_INFOL << elevation_offset_[i] << " ";
    if ((i+1) % 8 == 0) AG_INFOL << AG_REND;
  }

  AG_INFOL << "azimuth offset :" << AG_REND;
  for (int i = 0; i < A2_CHANNELS_PER_BLOCK; i++)
  {
    AG_INFOL << azimuth_offset_[i] << " ";
    if ((i+1) % 8 == 0) AG_INFOL << AG_REND;
  }

  AG_INFOL << "Package length : " << sizeof(A2MsopPkt) << AG_REND;
  AG_INFO << "------------------------------------------------------" << AG_REND;
}

} // namespace lidar
} // namespace asensing
