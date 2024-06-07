#pragma once

namespace asensing
{
namespace lidar
{


class DistanceSection
{
public:
  DistanceSection (float min, float max, float usr_min, float usr_max)
    : min_(min), max_(max)
  {
    if (usr_min <= 0.0) usr_min = 0.0;
    if (usr_max <= 0.0) usr_max = 0.0;

    if ((usr_min >= 0.0) && (usr_max > usr_min))
    {
      min_ = usr_min;
      max_ = usr_max;
    }
  }

  void updateDistanceSection(float usr_min, float usr_max)
  {
    if (usr_min < 0) usr_min = 0;
    if (usr_max < 0) usr_max = 0;

    if ((usr_min != 0) || (usr_max != 0))
    {
      min_ = usr_min;
      max_ = usr_max;
    }
  }

  bool in(float distance)
  {
    return ((min_ <= distance) && (distance <= max_));
  }

#ifndef UNIT_TEST
private:
#endif

  float min_;
  float max_;
};

class AngleSection
{
  public:
  AngleSection (float azi_min, float azi_max, float ele_min, float ele_max)
    : azi_min_(azi_min), azi_max_(azi_max), ele_min_(ele_min), ele_max_(ele_max)
  {
    if (azi_min_ > azi_max_)
    {
      azi_min_ = azi_max;
      azi_max_ = azi_min;
    }

    if (ele_min_ > ele_max_)
    {
      ele_min_ = ele_max;
      ele_max_ = ele_min;
    }
  }

  void updateAngleSection(float azi_min, float azi_max, float ele_min, float ele_max)
  {
    azi_min_ = azi_min;
    azi_max_ = azi_max;
    ele_min_ = ele_min;
    ele_max_ = ele_max;
  }

  bool in(float azimuth, float elevation)
  {
    return ((azi_min_ <= azimuth) && (azimuth <= azi_max_) && (ele_min_ <= elevation) && (elevation <= ele_max_));
  }
#ifndef UNIT_TEST
private:
#endif
  float azi_min_;
  float azi_max_;
  float ele_min_;
  float ele_max_;
};

class IntensitySection
{
public:
  IntensitySection (uint8_t intensity_min, uint8_t intensity_max)
    : min_(intensity_min), max_(intensity_max)
  {
  }

  void updateIntensitySection(uint8_t intensity_min, uint8_t intensity_max)
  {
    min_ = intensity_min;
    max_ = intensity_max;
  }

  bool in(float intensity)
  {
    return ((min_ <= intensity) && (intensity <= max_));
  }
#ifndef UNIT_TEST
private:
#endif
  uint8_t min_;
  uint8_t max_;
};

class ModuleSection
{
public:
  ModuleSection (uint8_t module_min, uint8_t module_max)
    : min_(module_min), max_(module_max)
  {
  }

  void updateModuleSection(uint8_t module_min, uint8_t module_max)
  {
    min_ = module_min;
    max_ = module_max;
  }

  bool in(float module)
  {
    return ((min_ <= module) && (module <= max_));
  }
#ifndef UNIT_TEST
private:
#endif
  uint8_t min_;
  uint8_t max_;
};

}  // namespace lidar
}  // namespace asensing
