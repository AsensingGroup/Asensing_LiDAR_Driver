#define PCL_NO_PRECOMPILE
#pragma once
#ifdef PCL_USE_PCDIO
#include <pcl/io/pcd_io.h>
#else
#include <pcl/io/io.h>
#endif

#include <pcl/point_types.h>
//#include <pcl_conversion/pcl_conversion.h>

typedef pcl::PointXYZI PointXYZI;

// use std::uint32_t instead of pcl::uint32_t
struct PointXYZIRT
{
  PCL_ADD_POINT4D;
  std::uint8_t intensity;
  std::uint8_t ring;
  std::uint16_t range;
#ifdef ENABLE_POINT_EXTEND_FIELD
  std::uint16_t azimuth;
  std::uint16_t elevation;
#endif
  double timestamp;
#ifdef COMPILE_TOOLS
  std::uint32_t FrameSn;
#endif
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

#ifdef ENABLE_POINT_EXTEND_FIELD
#ifdef COMPILE_TOOLS
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT, 
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (std::uint8_t, intensity, intensity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, range, range)
    (std::uint16_t, azimuth, azimuth)
    (std::uint16_t, elevation, elevation)
    (double, timestamp, timestamp)
    (std::uint32_t, FrameSn, FrameSn))
#else
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT, 
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (std::uint8_t, intensity, intensity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, range, range)
    (std::uint16_t, azimuth, azimuth)
    (std::uint16_t, elevation, elevation)
    (double, timestamp, timestamp))
#endif // COMPILE_TOOLS
#else
#ifdef COMPILE_TOOLS
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT, 
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (std::uint8_t, intensity, intensity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, range, range)
    (double, timestamp, timestamp)
    (std::uint32_t, FrameSn, FrameSn))
#else
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT, 
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (std::uint8_t, intensity, intensity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, range, range)
    (double, timestamp, timestamp))
#endif // COMPILE_TOOLS
#endif // ENABLE_POINT_EXTEND_FIELD

template <typename T_Point>
class PointCloudT : public pcl::PointCloud<T_Point>
{
public:
  typedef T_Point PointT;
  typedef typename pcl::PointCloud<T_Point>::VectorType VectorT;

  double timestamp = 0.0;
  uint32_t seq = 0;           // Sequence number of message
  std::string frame_id = "";  // Point cloud frame id
};

