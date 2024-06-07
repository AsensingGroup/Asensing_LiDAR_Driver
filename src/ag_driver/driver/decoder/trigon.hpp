#pragma once

#include "ag_driver/common/process_common.hpp"

#include <cmath>

namespace asensing
{
namespace lidar
{

//#define DBG

class Trigon
{
public:

  constexpr static int32_t ANGLE_MIN = -9000;
  constexpr static int32_t ANGLE_MAX = 45000;

  Trigon()
  {
    int32_t range = ANGLE_MAX - ANGLE_MIN;
#ifdef DBG
    o_angles_ = (int32_t*)malloc(range * sizeof(int32_t));
#endif
    o_sins_ = (float*)malloc(range * sizeof(float));
    o_coss_ = (float*)malloc(range * sizeof(float));

    for (int32_t i = ANGLE_MIN, j = 0; i < ANGLE_MAX; i++, j++)
    {
      double rad = DEGREE_TO_RADIAN(static_cast<double>(i) * 0.01);

#ifdef DBG
      o_angles_[j] = i;
#endif
      o_sins_[j] = (float)std::sin(rad);
      o_coss_[j] = (float)std::cos(rad);
    }

#ifdef DBG
    angles_ = o_angles_ - ANGLE_MIN;
#endif
    sins_ = o_sins_ - ANGLE_MIN;
    coss_ = o_coss_ - ANGLE_MIN;
  }

  ~Trigon()
  {
    free(o_coss_);
    free(o_sins_);
#ifdef DBG
    free(o_angles_);
#endif
  }

  float sin(int32_t angle)
  {
    if (angle < ANGLE_MIN || angle >= ANGLE_MAX)
    {
      angle = 0;
    }

    return sins_[angle];
  }

  float cos(int32_t angle)
  {
    if (angle < ANGLE_MIN || angle >= ANGLE_MAX)
    {
      angle = 0;
    }

    return coss_[angle];
  }

  void print()
  {
    for (int32_t i = -10; i < 10; i++)
    {
      std::cout << 
#ifdef DBG 
        angles_[i] << "\t" << 
#endif
        sins_[i] << "\t" << coss_[i] << std::endl;
    }
  }

private:
#ifdef DBG 
  int32_t* o_angles_;
  int32_t* angles_;
#endif
  float* o_sins_;
  float* o_coss_;
  float* sins_;
  float* coss_;
};

}  // namespace lidar
}  // namespace asensing
