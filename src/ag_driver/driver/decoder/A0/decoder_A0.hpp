#pragma once

#include <fstream>
#include "A0_struct.hpp"

#if ENABLE_OPENMP
#include <omp.h>
#endif

using json = nlohmann::json;

#define SPLIT_STRATEGY_BY_SEQ_NUM

namespace asensing
{
namespace lidar
{

template <typename T_PointCloud>
class DecoderA0 : public Decoder<T_PointCloud>
{
public:

  constexpr static double FRAME_DURATION = 0.1;
  //when single callback
  constexpr static uint32_t SINGLE_PKT_NUM = 1344;

  virtual void decodeDifopPkt(const uint8_t* pkt, size_t size);
  virtual bool decodeMsopPkt(const uint8_t* pkt, size_t size);

  virtual ~DecoderA0() = default;
  explicit DecoderA0(const AGDecoderParam& param);

  void A0LoadCalibration(const std::string& filename);

private:

  static AGDecoderConstParam& getConstParam();
  AGEchoMode getEchoMode(uint8_t mode);

#ifdef SPLIT_STRATEGY_BY_SEQ_NUM
  SplitStrategyBySeq split_strategy_;
#else
  SplitStrategyById split_strategy_;
#endif

  std::string A0CalibrationFileName = "A0-Correction.json";
  float m_angles[A0ANGLE_SIZE] = {-47.176, -23.548, 0, 23.548, 47.176, 17.8};

  void printCalibrationData();
};

template <typename T_PointCloud>
inline AGDecoderConstParam& DecoderA0<T_PointCloud>::getConstParam()
{
  static AGDecoderConstParam param = 
  {
    1172,   // msop len  
    108,    // difop len

    4,      // msop id len
    4,      // difop id len

    {0xAA, 0x55, 0xA5, 0x5A}, // msop id
    {0xA5, 0xFF, 0x00, 0x5A}, // difop id

    128,    // laser number (equivalent number of scan lines)
    12,     // blocks per packet
    10,     // channels per block

    0.0f,       // distance min
    200.0f,     // distance max
    0.01f,      // distance resolution
    -36000.0f,  // azimuth min
    72000.0f,   // azimuth max
    -9000.0f,   // elevation min
    36000.0f,   // elevation max
    0,          // intensity min
    255,        // intensity max
    0,          // module min
    4,          // module max
    80.0f   // initial value of temperature 
  };

  return param;
}

template <typename T_PointCloud>
inline DecoderA0<T_PointCloud>::DecoderA0(const AGDecoderParam& param)
  : Decoder<T_PointCloud>(getConstParam(), param)
{
  this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;

  this->A0LoadCalibration(A0CalibrationFileName);
  if((this->CalibEnabled == true) && (this->IsCalibrated == true)){
    this->angles_ready_ = true;
  }
  else{
    this->angles_ready_ = false;
  }
  this->printCalibrationData();
}

template <typename T_PointCloud>
inline AGEchoMode DecoderA0<T_PointCloud>::getEchoMode(uint8_t mode)
{
  switch (mode)
  {
    case 0x00:  // first return
    case 0x01:  // last return
    case 0x02:  // strongest return
      return AGEchoMode::ECHO_SINGLE;
    case 0x04: 
    case 0x05: 
    case 0x06: 
      return AGEchoMode::ECHO_DUAL;
    default:
      return AGEchoMode::ECHO_SINGLE;
  }
}

template <typename T_PointCloud>
inline void DecoderA0<T_PointCloud>::decodeDifopPkt(const uint8_t* packet, size_t size)
{
  const A0DifopPkt& pkt = *(A0DifopPkt*)packet;
  this->echo_mode_ = this->getEchoMode(pkt.difop_head.return_mode);

#ifdef ENABLE_DIFOP_PARSE
  // device info
  memcpy (this->device_info_.sn, pkt.difop_head.sn, 6);
  memcpy (this->device_info_.mac, pkt.difop_head.eth.mac_addr, 6);
  memcpy (this->device_info_.top_ver, pkt.difop_head.version.firmware_ver, 6);

  // device status
  this->device_status_.voltage = ntohs(pkt.func_safety.lidar_status);
#endif
}

template <typename T_PointCloud>
inline bool DecoderA0<T_PointCloud>::decodeMsopPkt(const uint8_t* packet, size_t size)
{
  const A0MsopPkt& pkt = *(A0MsopPkt*)packet;
  bool ret = false;

  //this->temperature_ = static_cast<float>((int)pkt.header.temperature - this->const_param_.TEMPERATURE_RES);
  uint8_t mytem = 0x30;
  this->temperature_ = mytem;

  //something wrong
  double pkt_ts = 0;
  if (this->param_.use_lidar_clock)
  {
    pkt_ts = parseTimeWithAG(&pkt.header.timestamp);
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
      createTimeWithAG ((AGTimestampUTC*)&pkt.header.timestamp);
    }
  }

  uint32_t frame_id = pkt.header.FrameSn;
  //Because the packet sent by radar does not convert the multiple bytes to the size end. Therefore, no "ntohs" processing is required
  uint16_t pkt_seq = pkt.header.pkt_seq;

#ifdef SPLIT_STRATEGY_BY_SEQ_NUM
  if (split_strategy_.newPacket(frame_id, pkt_seq))
  {
    this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());
    this->first_point_ts_ = pkt_ts;
    ret = true;
  }
#else
  SplitStrategyEnum split_flag = split_strategy_.newPacket(frame_id, pkt_seq);

  if (split_flag == SplitStrategyEnum::split)
  {
    this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());
    this->first_point_ts_ = pkt_ts;
    ret = true;
  }
  else if (split_flag == SplitStrategyEnum::discard)
  {
    return false;
  }
#endif

  uint8_t type = pkt.header.LidarInfo >> 6;

#if ENABLE_OPENMP
#pragma omp parallel for
#endif
  for (uint16_t blk = 0; blk < this->const_param_.BLOCKS_PER_PKT; blk++)
  {
    const A0MsopBlock& block = pkt.blocks[blk];

    double point_time = pkt_ts + block.time_offset * 1e-6;

#if ENABLE_OPENMP
#pragma omp parallel for
#endif
    /* The channel corresponds to a laser channel */
    for (uint16_t channel = 0; channel < this->const_param_.CHANNELS_PER_BLOCK; channel++)
    {
      const A0MsopChannel& unit = block.units[channel];

      float distance = static_cast<float>(unit.distance) * this->const_param_.DISTANCE_RES;
      // uint8_t confidence = unit.confidence;

      if (this->distance_section_.in(distance) && this->intensity_section_.in(unit.intensity) && 
          this->module_section_.in(channel/2) && this->angle_section_.in(unit.azimuth, unit.elevation))
      {
        float x = 0;
        float y = 0;
        float z = 0;

        if(type == 0x02 || type == 0x03) // 0x02 -> 0°, 0x03 -> 25°, obsoleted
        {
          //Incident vector solution
          float vector[A0VECTOR_SIZE] = {0};
          //float theta = DEGREE_TO_RADIAN(m_angles[channel / 2]);
          //float gamma0 = DEGREE_TO_RADIAN(m_angles[A0ANGLE_SIZE-1]);
          float theta = DEGREE_MULTIPLY_ACCURACY(m_angles[channel / 2]);
          float gamma0 = DEGREE_MULTIPLY_ACCURACY(m_angles[A0ANGLE_SIZE-1]);

          float sin_gamma0 = SIN(gamma0);
          float cos_gamma0 = COS(gamma0);
          float cos_theta = COS(theta);
          vector[0] = cos_theta - 2.0 * cos_theta * cos_gamma0 * cos_gamma0;
          vector[1] = SIN(theta);
          vector[2] =  2.0 * cos_theta * sin_gamma0 * cos_gamma0;

          //Normal vector solution
          float normal[A0VECTOR_SIZE] = {0};
       
          float angle = static_cast<float>(unit.azimuth) * ASENSING_AZIMUTH_UNIT;
          angle = (angle > 120) ? (angle - 360) : angle;

          if(type == 0x03)
          {
              if(channel / 2 == 0)
              {
                  angle += 50;
              }
              else if(channel / 2 == 1)
              {
                  angle += 25;
              }
              else if(channel / 2 == 3)
              {
                  angle -= 25;
              }
              else if(channel / 2 == 4)
              {
                  angle -= 50;
              }
          }
          //float gamma = DEGREE_TO_RADIAN(-angle);
          float gamma = DEGREE_MULTIPLY_ACCURACY(-angle);
          angle = static_cast<float>(unit.elevation) * ASENSING_ELEVATION_UNIT;
          angle = (angle > 120) ? (angle - 360) : angle;
        
          //float  beta = - 1 *DEGREE_TO_RADIAN(angle);
          float  beta = - 1 * DEGREE_MULTIPLY_ACCURACY(angle);
          float sin_gamma = SIN(gamma);
          float cos_gamma = COS(gamma);
          float sin_beta = SIN(beta);
          float cos_beta = COS(beta);
          normal[0] = cos_beta * cos_gamma * cos_gamma0 - sin_beta * sin_gamma0;
          normal[1] = sin_gamma * cos_gamma0;
          normal[2] = -cos_gamma0 * sin_beta * cos_gamma - cos_beta * sin_gamma0;
        
          //Final vector solution
          float out[A0VECTOR_SIZE] = {0};
          float k = vector[0] * normal[0] + vector[1] * normal[1] + vector[2] * normal[2];
          for(int i = 0; i < A0VECTOR_SIZE; i++)
          {
            out[i] = vector[i] - 2 * k * normal[i];
          }
          x = distance * out[0];
          y = distance * out[1];
          z = distance * out[2];
        }
        else{
          float elevation = unit.elevation;
          float azimuth = unit.azimuth;

          x = distance * COS (elevation) * COS (azimuth);
          y = distance * COS (elevation) * SIN (azimuth);
          z = distance * SIN (elevation);
        }
        
        this->transformPoint(x, y, z);

        //point num: 161280 ;   rows: 128   columns: 5* 252
        //pkt_seq:0-1343 ; channel:0-9 ; 
        uint8_t A0ring = (uint8_t)((((pkt.header.pkt_seq * 12) / 252) * 2) + (channel % 2));
        
        typename T_PointCloud::PointT point;
        setX(point, x);
        setY(point, y);
        setZ(point, z);
        setIntensity(point, unit.intensity);
        setTimestamp(point, point_time);
        setRing(point, A0ring);
        setRange(point, unit.distance);
        #ifdef ENABLE_POINT_EXTEND_FIELD
        setAzimuth(point, unit.azimuth);
        setElevation(point, unit.elevation);
        #endif
        #ifdef COMPILE_TOOLS
        setFrameSn(point, frame_id);
        #endif

        this->point_cloud_->points.emplace_back(point);
      }
      else if (!this->param_.dense_points)
      {
        uint8_t A0ring = (uint8_t)((((pkt.header.pkt_seq * 12) / 252) * 2) + (channel % 2));
        
        typename T_PointCloud::PointT point;
        setX(point, NAN);
        setY(point, NAN);
        setZ(point, NAN);
        setIntensity(point, 0);
        setTimestamp(point, point_time);
        setRing(point, A0ring);
        setRange(point, 0);
        #ifdef ENABLE_POINT_EXTEND_FIELD
        setAzimuth(point, unit.azimuth);
        setElevation(point, unit.elevation);
        #endif
        #ifdef COMPILE_TOOLS
        setFrameSn(point, frame_id);
        #endif

        this->point_cloud_->points.emplace_back(point);
      }
    }

    this->prev_point_ts_ = point_time;
  }

  this->prev_pkt_ts_ = pkt_ts;
  return ret;
}

template <typename T_PointCloud>
inline void DecoderA0<T_PointCloud>::A0LoadCalibration(const std::string& filename)
{
  if (filename.empty())
  {
    this->IsCalibrated = false;
    return;
  }

  std::string keyword("A0-Correction");
  std::size_t found = filename.find(keyword);
  if (std::string::npos == found)
  {
    AG_WARNING << "Do not need to correct data: " << filename << AG_REND;
    this->CalibEnabled = false;
    this->IsCalibrated = true;
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
  config_path = "../../../src/ag_driver/config/";  //for the MSVS project(demo and tool)
  config_path += filename;
#endif
#endif

  std::ifstream f(config_path.c_str());

  if (!f.is_open()) {
    AG_WARNING << "Can not open: " << config_path << AG_REND;
    this->IsCalibrated = false;
    this->CalibEnabled = true;
    return ;
  }

  json data = json::parse(f);
  // AG_DEBUG << data.dump() << AG_REND;

  for (int i = 0; i < A0ANGLE_SIZE; i++)
  {
    m_angles[i] = data["module_angles"][i];
  }

  data.clear();
  f.close();

  this->IsCalibrated = true;
  this->CalibEnabled = true;
}

template <typename T_PointCloud>
inline void DecoderA0<T_PointCloud>::printCalibrationData()
{
  AG_INFO << "------------------------------------------------------" << AG_REND;
  AG_INFO << "        Calibration File: " << A0CalibrationFileName << AG_REND;

  if (this->IsCalibrated) {
    AG_INFOL << "(calibrated)" << AG_REND;
  }
  else {
    AG_INFOL << "(uncalibrated, use default values)" << AG_REND;
  }

  for (int i = 0; i < A0ANGLE_SIZE; i++)
  {
    AG_INFOL << "Module angles " << i << " : " << m_angles[i] << AG_REND;
  }

  AG_INFO << "------------------------------------------------------" << AG_REND;
}

}  // namespace lidar
}  // namespace asensing
