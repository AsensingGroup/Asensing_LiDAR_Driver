#pragma once

#include "ag_driver/common/ag_log.hpp"
#include <string>
#include <map>

namespace asensing
{
namespace lidar
{
enum LidarType  // LiDAR type
{
  // mems
  AG_MEMS = 0x01,
  A0 = AG_MEMS,
  A2,
  A4
};

inline bool isMems (LidarType type)
{
  //return ((LidarType::AG_MEMS <= type));
  //only mems
  return true;
}

inline std::string lidarTypeToStr(const LidarType& type)
{
  std::string str = "";
  switch (type)
  {
    case LidarType::A0:
      str = "A0";
      break;
    case LidarType::A2:
      str = "A2";
      break;
    default:
      str = "ERROR";
      AG_ERROR << "AG_ERROR" << AG_REND;
  }
  return str;
}

inline LidarType strToLidarType(const std::string& type)
{
  if (type == "A0")
  {
    return lidar::LidarType::A0;
  }
  if (type == "A2")
  {
    return lidar::LidarType::A2;
  }
  else
  {
    AG_ERROR << "Wrong lidar type: " << type << AG_REND;
    AG_ERROR << "Please give correct type: A0, A2." << AG_REND;
    exit(-1);
  }
}

enum InputType
{
  ONLINE_LIDAR = 1,
  PCAP_FILE,
  RAW_PACKET
};

inline std::string inputTypeToStr(const InputType& type)
{
  std::string str = "";
  switch (type)
  {
    case InputType::ONLINE_LIDAR:
      str = "ONLINE_LIDAR";
      break;
    case InputType::PCAP_FILE:
      str = "PCAP_FILE";
      break;
    case InputType::RAW_PACKET:
      str = "RAW_PACKET";
      break;
    default:
      str = "ERROR";
      AG_ERROR << "AG_ERROR" << AG_REND;
  }
  return str;
}

struct AGTransformParam  // The Point transform parameter
{
  float x = 0.0f;      // unit, m
  float y = 0.0f;      // unit, m
  float z = 0.0f;      // unit, m
  float roll = 0.0f;   // unit, radian
  float elevation = 0.0f;  // unit, radian
  float azimuth = 0.0f;    // unit, radian

  void print() const
  {
    AG_INFO << "------------------------------------------------------" << AG_REND;
    AG_INFO << "             Asensing Transform Parameters " << AG_REND;
    AG_INFOL << "x: " << x << AG_REND;
    AG_INFOL << "y: " << y << AG_REND;
    AG_INFOL << "z: " << z << AG_REND;
    AG_INFOL << "roll: " << roll << AG_REND;
    AG_INFOL << "elevation: " << elevation << AG_REND;
    AG_INFOL << "azimuth: " << azimuth << AG_REND;
    AG_INFO << "------------------------------------------------------" << AG_REND;
  }
};

struct AGDecoderParam  // LiDAR decoder parameter
{
  bool config_from_file = false; // Internal use only for debugging
  std::string angle_path = "";   // Internal use only for debugging
  bool wait_for_difop = false;   // true: start sending point cloud until receive difop packet
  float min_distance = 0.0f;     // min/max distances of point cloud range. valid if min distance or max distance > 0
  float max_distance = 1000.0f; 
  float min_azimuth = -36000.0f;
  float max_azimuth = 36000.0f;
  float min_elevation = -36000.0f;
  float max_elevation = 36000.0f;
  uint8_t min_intensity = 0;
  uint8_t max_intensity = 255;
  uint8_t min_module = 0;
  uint8_t max_module = 255;
  bool use_lidar_clock = false;  // true: use LiDAR clock as timestamp; false: use system clock as timestamp
  bool write_pkt_ts = false;     // rewrite pkt timestamp or not; (prerequisite: use_lidar_clock must be false)
  bool dense_points = false;     // true: discard NAN points; false: reserve NAN points
  bool ts_first_point = true;    // true: time-stamp point cloud with the first point; false: with the last point;
  AGTransformParam transform_param; // Used to transform points

  void print() const
  {
    AG_INFO << "------------------------------------------------------" << AG_REND;
    AG_INFO << "             Asensing Decoder Parameters " << AG_REND;
    AG_INFOL << "wait_for_difop: " << wait_for_difop << AG_REND;
    AG_INFOL << "min_distance: " << min_distance << AG_REND;
    AG_INFOL << "max_distance: " << max_distance << AG_REND;
    AG_INFOL << "use_lidar_clock: " << use_lidar_clock << AG_REND;
    AG_INFOL << "write_pkt_ts: " << write_pkt_ts << AG_REND;
    AG_INFOL << "dense_points: " << dense_points << AG_REND;
    AG_INFOL << "config_from_file: " << config_from_file << AG_REND;
    AG_INFOL << "angle_path: " << angle_path << AG_REND;
    AG_INFO << "------------------------------------------------------" << AG_REND;
    transform_param.print();
  }

};

struct AGInputParam  // The LiDAR input parameter
{
  uint16_t msop_port = 51180;                   // Msop packet port number
  uint16_t difop_port = 9988;                  // Difop packet port number
  std::string host_address = "0.0.0.0";    // 192.168.101.101   // Address of host
  std::string group_address = "0.0.0.0";       // Address of multicast group
  std::string pcap_path = "";                  // Absolute path of pcap file
  bool pcap_repeat = true;                     // true: The pcap bag will repeat play
  float pcap_rate = 1.0f;                      // Rate to read the pcap file
  bool use_vlan = false;                       // Vlan on-off
  uint16_t user_layer_bytes = 0;    // Bytes of user layer. thers is no user layer if it is 0
  uint16_t tail_layer_bytes = 0;    // Bytes of tail layer. thers is no tail layer if it is 0

  void print() const
  {
    AG_INFO << "------------------------------------------------------" << AG_REND;
    AG_INFO << "             Asensing Input Parameters " << AG_REND;
    AG_INFOL << "msop_port: " << msop_port << AG_REND;
    AG_INFOL << "difop_port: " << difop_port << AG_REND;
    AG_INFOL << "host_address: " << host_address << AG_REND;
    AG_INFOL << "group_address: " << group_address << AG_REND;
    AG_INFOL << "pcap_path: " << pcap_path << AG_REND;
    AG_INFOL << "pcap_rate: " << pcap_rate << AG_REND;
    AG_INFOL << "pcap_repeat: " << pcap_repeat << AG_REND;
    AG_INFOL << "use_vlan: " << use_vlan << AG_REND;
    AG_INFOL << "user_layer_bytes: " << user_layer_bytes << AG_REND;
    AG_INFOL << "tail_layer_bytes: " << tail_layer_bytes << AG_REND;
    AG_INFO << "------------------------------------------------------" << AG_REND;
  }

};

struct AGDriverParam  // The LiDAR driver parameter
{
  LidarType lidar_type = LidarType::A0;  // Lidar type
  InputType input_type = InputType::ONLINE_LIDAR; // Input type
  std::string frame_id = "aglidar";  // The frame id of LiDAR mesage
  AGInputParam input_param;          // Input parameter
  AGDecoderParam decoder_param;      // Decoder parameter

  void print() const
  {
    AG_INFO << "------------------------------------------------------" << AG_REND;
    AG_INFO << "             Asensing Driver Parameters " << AG_REND;
    AG_INFOL << "input type: " << inputTypeToStr(input_type) << AG_REND;
    AG_INFOL << "lidar_type: " << lidarTypeToStr(lidar_type) << AG_REND;
    AG_INFOL << "frame_id: "   << frame_id << AG_REND;
    AG_INFOL << "------------------------------------------------------" << AG_REND;

    input_param.print();
    decoder_param.print();
  }

};

struct DeviceInfo
{
  uint8_t sn[6];
  uint8_t mac[6];
  uint8_t version[6];
};

struct DeviceStatus
{
  float voltage = 0.0f;
};

struct SelectionParam
{
  float min_distance = 0.0f;    // unit: mm
  float max_distance = 0.0f;    // unit: mm
  float min_azimuth = 0.0f;     // unit: degree * 100
  float max_azimuth = 0.0f;     // unit: degree * 100
  float min_elevation = 0.0f;   // unit: degree * 100
  float max_elevation = 0.0f;   // unit: degree * 100
  uint8_t min_intensity = 0;    // unit: 1 (range: 0-255)
  uint8_t max_intensity = 0;    // unit: 1 (range: 0-255)
  uint8_t min_module = 0;       // unit: 1 (range: 0-4, only for A0)
  uint8_t max_module = 0;       // unit: 1 (range: 0-4, only for A0)
};

}  // namespace lidar
}  // namespace asensing
