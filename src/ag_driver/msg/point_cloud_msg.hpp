#pragma once

#include <vector>
#include <string>

struct PointXYZI
{
  float x;
  float y;
  float z;
  uint8_t intensity;
};

struct PointXYZIRT
{
  float x;
  float y;
  float z;
  uint8_t intensity;
  uint8_t ring;
  uint16_t range;
#ifdef ENABLE_POINT_EXTEND_FIELD
  uint16_t azimuth;
  uint16_t elevation;
#endif
  double timestamp;
#ifdef COMPILE_TOOLS
  uint32_t FrameSn;
#endif
};

template <typename T_Point>
class PointCloudT
{
public:
  typedef T_Point PointT;
  typedef std::vector<PointT> VectorT;

  uint32_t height = 0;    // Height of point cloud
  uint32_t width = 0;     // Width of point cloud
  bool is_dense = false;  // If is_dense is true, the point cloud does not contain NAN points,
  double timestamp = 0.0;
  uint32_t seq = 0;           // Sequence number of message
  std::string frame_id = "";  // Point cloud frame id

  VectorT points;
};
