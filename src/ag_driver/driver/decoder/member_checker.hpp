#pragma once

#define DEFINE_MEMBER_CHECKER(member)                                                                                  \
  template <typename T, typename V = bool>                                                                             \
  struct has_##member : std::false_type                                                                                \
  {                                                                                                                    \
  };                                                                                                                   \
  template <typename T>                                                                                                \
  struct has_##member<                                                                                                 \
      T, typename std::enable_if<!std::is_same<decltype(std::declval<T>().member), void>::value, bool>::type>          \
      : std::true_type                                                                                                 \
  {                                                                                                                    \
  };

DEFINE_MEMBER_CHECKER(x)
DEFINE_MEMBER_CHECKER(y)
DEFINE_MEMBER_CHECKER(z)
DEFINE_MEMBER_CHECKER(intensity)
DEFINE_MEMBER_CHECKER(ring)
DEFINE_MEMBER_CHECKER(range)
#ifdef ENABLE_POINT_EXTEND_FIELD
DEFINE_MEMBER_CHECKER(azimuth)
DEFINE_MEMBER_CHECKER(elevation)
#endif
DEFINE_MEMBER_CHECKER(timestamp)
#ifdef COMPILE_TOOLS
DEFINE_MEMBER_CHECKER(FrameSn)
#endif

#define AG_HAS_MEMBER(C, member) has_##member<C>::value

template <typename T_Point>
inline typename std::enable_if<!AG_HAS_MEMBER(T_Point, x)>::type setX(T_Point& point, const float& value)
{
}

template <typename T_Point>
inline typename std::enable_if<AG_HAS_MEMBER(T_Point, x)>::type setX(T_Point& point, const float& value)
{
  point.x = value;
}

template <typename T_Point>
inline typename std::enable_if<!AG_HAS_MEMBER(T_Point, y)>::type setY(T_Point& point, const float& value)
{
}

template <typename T_Point>
inline typename std::enable_if<AG_HAS_MEMBER(T_Point, y)>::type setY(T_Point& point, const float& value)
{
  point.y = value;
}

template <typename T_Point>
inline typename std::enable_if<!AG_HAS_MEMBER(T_Point, z)>::type setZ(T_Point& point, const float& value)
{
}

template <typename T_Point>
inline typename std::enable_if<AG_HAS_MEMBER(T_Point, z)>::type setZ(T_Point& point, const float& value)
{
  point.z = value;
}

template <typename T_Point>
inline typename std::enable_if<!AG_HAS_MEMBER(T_Point, intensity)>::type setIntensity(T_Point& point,
                                                                                      const uint8_t& value)
{
}

template <typename T_Point>
inline typename std::enable_if<AG_HAS_MEMBER(T_Point, intensity)>::type setIntensity(T_Point& point,
                                                                                     const uint8_t& value)
{
  point.intensity = value;
}

template <typename T_Point>
inline typename std::enable_if<!AG_HAS_MEMBER(T_Point, ring)>::type setRing(T_Point& point, const uint8_t& value)
{
}

template <typename T_Point>
inline typename std::enable_if<AG_HAS_MEMBER(T_Point, ring)>::type setRing(T_Point& point, const uint8_t& value)
{
  point.ring = value;
}

template <typename T_Point>
inline typename std::enable_if<!AG_HAS_MEMBER(T_Point, timestamp)>::type setTimestamp(T_Point& point,
                                                                                      const double& value)
{
}

template <typename T_Point>
inline typename std::enable_if<AG_HAS_MEMBER(T_Point, timestamp)>::type setTimestamp(T_Point& point,
                                                                                     const double& value)
{
  point.timestamp = value;
}

template <typename T_Point>
inline typename std::enable_if<!AG_HAS_MEMBER(T_Point, range)>::type setRange(T_Point& point, const uint16_t& value)
{
}

template <typename T_Point>
inline typename std::enable_if<AG_HAS_MEMBER(T_Point, range)>::type setRange(T_Point& point, const uint16_t& value)
{
  point.range = value;
}

#ifdef COMPILE_TOOLS
template <typename T_Point>
inline typename std::enable_if<!AG_HAS_MEMBER(T_Point, range)>::type setFrameSn(T_Point& point, const uint32_t& value)
{
}

template <typename T_Point>
inline typename std::enable_if<AG_HAS_MEMBER(T_Point, range)>::type setFrameSn(T_Point& point, const uint32_t& value)
{
  point.FrameSn = value;
}
#endif

#ifdef ENABLE_POINT_EXTEND_FIELD
template <typename T_Point>
inline typename std::enable_if<!AG_HAS_MEMBER(T_Point, azimuth)>::type setAzimuth(T_Point& point, const uint16_t& value)
{
}

template <typename T_Point>
inline typename std::enable_if<AG_HAS_MEMBER(T_Point, azimuth)>::type setAzimuth(T_Point& point, const uint16_t& value)
{
  point.azimuth = value;
}

template <typename T_Point>
inline typename std::enable_if<!AG_HAS_MEMBER(T_Point, elevation)>::type setElevation(T_Point& point, const uint16_t& value)
{
}

template <typename T_Point>
inline typename std::enable_if<AG_HAS_MEMBER(T_Point, elevation)>::type setElevation(T_Point& point, const uint16_t& value)
{
  point.elevation = value;
}
#endif