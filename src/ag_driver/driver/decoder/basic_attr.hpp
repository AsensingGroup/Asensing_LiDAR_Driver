#pragma once
#include "ag_driver/common/process_common.hpp"

#include <fstream>
#include <cmath>
#include <algorithm>
#include <functional>
#include <chrono>
#include <mutex>
#ifdef WIN32
#define NOMINMAX
#include <windows.h>
#else
#include <sys/time.h>
#endif

namespace asensing
{
namespace lidar
{

#pragma pack(push, 1)

typedef struct 
{
  uint8_t sec[6]; //year
                  //month
                  //day
                  //hour
                  //minute
                  //second
  uint8_t ss[4];  //identify the timestamp for point cloud
} AGTimestampUTC;

#pragma pack(pop)

#ifdef ENABLE_STAMP_WITH_LOCAL
inline long getTimezone(void)
{
  static long tzone = 0;
  static bool tzoneReady = false;

  if (!tzoneReady)
  {
#ifdef _MSC_VER
    _get_timezone(&tzone);
#else
    tzset();
    tzone = timezone;
#endif

    tzoneReady = true;
  }

  return tzone;
}
#endif
#ifdef WIN32
inline int gettimeofday(struct timeval *tp, void *tzp)
{
  time_t clock;
  struct tm tm;
  SYSTEMTIME wtm;
  GetLocalTime(&wtm);
  tm.tm_year   = wtm.wYear - 1900;
  tm.tm_mon   = wtm.wMonth - 1;
  tm.tm_mday   = wtm.wDay;
  tm.tm_hour   = wtm.wHour;
  tm.tm_min   = wtm.wMinute;
  tm.tm_sec   = wtm.wSecond;
  tm. tm_isdst  = -1;
  clock = mktime(&tm);
  tp->tv_sec = clock;
  tp->tv_usec = wtm.wMilliseconds * 1000;
  return (0);
}
#endif
inline double parseTimeWithAG(const AGTimestampUTC* tsUtc)
{
  struct tm t;
  t.tm_year = tsUtc->sec[0];
  t.tm_mon = tsUtc->sec[1] - 1;
  t.tm_mday = tsUtc->sec[2];
  t.tm_hour = tsUtc->sec[3];
  t.tm_min = tsUtc->sec[4];
  t.tm_sec = tsUtc->sec[5];
  t.tm_isdst = 0;

  // Time in second of the packets
  time_t unix_second = (timegm(&t));

  // Timestamp contains in the packet
  // roll back every second, probably in microsecond
  uint32_t timestampPacket = 0;
  for (int i = 0; i < 4; i++)
  {
    timestampPacket += tsUtc->ss[i] << (8 * i); 
  }

  // Timestamp in second of the packet
  double timestamp = unix_second + (timestampPacket / 1000000.0);

  return timestamp;
}

inline void createTimeWithAG(AGTimestampUTC* tsUtc)
{
  struct timeval tv;
  gettimeofday(&tv,NULL);

  time_t unix_second = (tv.tv_sec);

  // Timestamp contains in the packet
  // roll back every second, probably in microsecond
  uint32_t timestampPacket = tv.tv_usec;

  // Timestamp in second of the packet
#ifdef ENABLE_STAMP_WITH_LOCAL
  unix_second += getTimezone();
#endif
  
  struct tm *t = gmtime(&unix_second);

  tsUtc->sec[0] = t->tm_year;
  tsUtc->sec[1] = t->tm_mon + 1;
  tsUtc->sec[2] = t->tm_mday;
  tsUtc->sec[3] = t->tm_hour;
  tsUtc->sec[4] = t->tm_min;
  tsUtc->sec[5] = t->tm_sec;
  t->tm_isdst = 0;

  for (int i = 0; i < 4; i++)
  {  
    tsUtc->ss[i] = timestampPacket & 0xFF; 
    timestampPacket >>= 8;
  }
  return ;

}

inline double getTimeHostWithAG(void)
{
  struct timeval tv;
  gettimeofday(&tv, NULL);

  time_t unix_second = (tv.tv_sec);

  // Timestamp contains in the packet
  // roll back every second, probably in microsecond
  uint32_t timestampPacket = tv.tv_usec;

  // Timestamp in second of the packet
  double timestamp = unix_second + (timestampPacket / 1000000.0);
  
  return timestamp;
}

inline uint64_t parseTimeUTCWithUs(const AGTimestampUTC* tsUtc)
{
  // sec
  uint64_t sec = 0;
  for (int i = 0; i < 6; i++)
  {
    sec <<= 8;
    sec += tsUtc->sec[i];
  }

  // us
  uint64_t us = 0;
  for (int i = 0; i < 4; i++)
  {
    us <<= 8;
    us += tsUtc->ss[i]; 
  }

#ifdef ENABLE_STAMP_WITH_LOCAL
  sec -= getTimezone();
#endif

  uint64_t pkt_time = (sec * 1000000 + us) ;

  return (pkt_time);
}

inline void createTimeUTCWithUs(uint64_t us, AGTimestampUTC* tsUtc)
{
  uint64_t sec  = us / 1000000;
  uint64_t usec = us % 1000000;

#ifdef ENABLE_STAMP_WITH_LOCAL
  sec += getTimezone();
#endif

  for (int i = 5; i >= 0; i--)
  {
    tsUtc->sec[i] = sec & 0xFF;
    sec >>= 8;
  }

  for (int i = 3; i >= 0; i--)
  {
    tsUtc->ss[i] = usec & 0xFF;
    usec >>= 8;
  }
}


inline uint64_t getTimeHost(void)
{
  std::chrono::system_clock::time_point t = std::chrono::system_clock::now();

  std::chrono::system_clock::duration t_s = t.time_since_epoch();

  //t_us:  clock period(1l/1000000l s)    time interval(t_s)
  //to Calculate the number of clock cycles
  std::chrono::duration<uint64_t, std::ratio<1l, 1000000l>> t_us = 
    std::chrono::duration_cast<std::chrono::duration<uint64_t, std::ratio<1l, 1000000l>>>(t_s);
  
  return t_us.count();
}


inline uint32_t calcCrc32(const uint8_t *data, uint32_t len, 
    uint32_t startValue, bool isFirstCall)
{
  static const uint32_t crc32table[] =
  {
    0x00000000U, 0x77073096U, 0xee0e612cU, 0x990951baU, 0x076dc419U,
    0x706af48fU, 0xe963a535U, 0x9e6495a3U, 0x0edb8832U, 0x79dcb8a4U,
    0xe0d5e91eU, 0x97d2d988U, 0x09b64c2bU, 0x7eb17cbdU, 0xe7b82d07U,
    0x90bf1d91U, 0x1db71064U, 0x6ab020f2U, 0xf3b97148U, 0x84be41deU,
    0x1adad47dU, 0x6ddde4ebU, 0xf4d4b551U, 0x83d385c7U, 0x136c9856U,
    0x646ba8c0U, 0xfd62f97aU, 0x8a65c9ecU, 0x14015c4fU, 0x63066cd9U,
    0xfa0f3d63U, 0x8d080df5U, 0x3b6e20c8U, 0x4c69105eU, 0xd56041e4U,
    0xa2677172U, 0x3c03e4d1U, 0x4b04d447U, 0xd20d85fdU, 0xa50ab56bU,
    0x35b5a8faU, 0x42b2986cU, 0xdbbbc9d6U, 0xacbcf940U, 0x32d86ce3U,
    0x45df5c75U, 0xdcd60dcfU, 0xabd13d59U, 0x26d930acU, 0x51de003aU,
    0xc8d75180U, 0xbfd06116U, 0x21b4f4b5U, 0x56b3c423U, 0xcfba9599U,
    0xb8bda50fU, 0x2802b89eU, 0x5f058808U, 0xc60cd9b2U, 0xb10be924U,
    0x2f6f7c87U, 0x58684c11U, 0xc1611dabU, 0xb6662d3dU, 0x76dc4190U,
    0x01db7106U, 0x98d220bcU, 0xefd5102aU, 0x71b18589U, 0x06b6b51fU,
    0x9fbfe4a5U, 0xe8b8d433U, 0x7807c9a2U, 0x0f00f934U, 0x9609a88eU,
    0xe10e9818U, 0x7f6a0dbbU, 0x086d3d2dU, 0x91646c97U, 0xe6635c01U,
    0x6b6b51f4U, 0x1c6c6162U, 0x856530d8U, 0xf262004eU, 0x6c0695edU,
    0x1b01a57bU, 0x8208f4c1U, 0xf50fc457U, 0x65b0d9c6U, 0x12b7e950U,
    0x8bbeb8eaU, 0xfcb9887cU, 0x62dd1ddfU, 0x15da2d49U, 0x8cd37cf3U,
    0xfbd44c65U, 0x4db26158U, 0x3ab551ceU, 0xa3bc0074U, 0xd4bb30e2U,
    0x4adfa541U, 0x3dd895d7U, 0xa4d1c46dU, 0xd3d6f4fbU, 0x4369e96aU,
    0x346ed9fcU, 0xad678846U, 0xda60b8d0U, 0x44042d73U, 0x33031de5U,
    0xaa0a4c5fU, 0xdd0d7cc9U, 0x5005713cU, 0x270241aaU, 0xbe0b1010U,
    0xc90c2086U, 0x5768b525U, 0x206f85b3U, 0xb966d409U, 0xce61e49fU,
    0x5edef90eU, 0x29d9c998U, 0xb0d09822U, 0xc7d7a8b4U, 0x59b33d17U,
    0x2eb40d81U, 0xb7bd5c3bU, 0xc0ba6cadU, 0xedb88320U, 0x9abfb3b6U,
    0x03b6e20cU, 0x74b1d29aU, 0xead54739U, 0x9dd277afU, 0x04db2615U,
    0x73dc1683U, 0xe3630b12U, 0x94643b84U, 0x0d6d6a3eU, 0x7a6a5aa8U,
    0xe40ecf0bU, 0x9309ff9dU, 0x0a00ae27U, 0x7d079eb1U, 0xf00f9344U,
    0x8708a3d2U, 0x1e01f268U, 0x6906c2feU, 0xf762575dU, 0x806567cbU,
    0x196c3671U, 0x6e6b06e7U, 0xfed41b76U, 0x89d32be0U, 0x10da7a5aU,
    0x67dd4accU, 0xf9b9df6fU, 0x8ebeeff9U, 0x17b7be43U, 0x60b08ed5U,
    0xd6d6a3e8U, 0xa1d1937eU, 0x38d8c2c4U, 0x4fdff252U, 0xd1bb67f1U,
    0xa6bc5767U, 0x3fb506ddU, 0x48b2364bU, 0xd80d2bdaU, 0xaf0a1b4cU,
    0x36034af6U, 0x41047a60U, 0xdf60efc3U, 0xa867df55U, 0x316e8eefU,
    0x4669be79U, 0xcb61b38cU, 0xbc66831aU, 0x256fd2a0U, 0x5268e236U,
    0xcc0c7795U, 0xbb0b4703U, 0x220216b9U, 0x5505262fU, 0xc5ba3bbeU,
    0xb2bd0b28U, 0x2bb45a92U, 0x5cb36a04U, 0xc2d7ffa7U, 0xb5d0cf31U,
    0x2cd99e8bU, 0x5bdeae1dU, 0x9b64c2b0U, 0xec63f226U, 0x756aa39cU,
    0x026d930aU, 0x9c0906a9U, 0xeb0e363fU, 0x72076785U, 0x05005713U,
    0x95bf4a82U, 0xe2b87a14U, 0x7bb12baeU, 0x0cb61b38U, 0x92d28e9bU,
    0xe5d5be0dU, 0x7cdcefb7U, 0x0bdbdf21U, 0x86d3d2d4U, 0xf1d4e242U,
    0x68ddb3f8U, 0x1fda836eU, 0x81be16cdU, 0xf6b9265bU, 0x6fb077e1U,
    0x18b74777U, 0x88085ae6U, 0xff0f6a70U, 0x66063bcaU, 0x11010b5cU,
    0x8f659effU, 0xf862ae69U, 0x616bffd3U, 0x166ccf45U, 0xa00ae278U,
    0xd70dd2eeU, 0x4e048354U, 0x3903b3c2U, 0xa7672661U, 0xd06016f7U,
    0x4969474dU, 0x3e6e77dbU, 0xaed16a4aU, 0xd9d65adcU, 0x40df0b66U,
    0x37d83bf0U, 0xa9bcae53U, 0xdebb9ec5U, 0x47b2cf7fU, 0x30b5ffe9U,
    0xbdbdf21cU, 0xcabac28aU, 0x53b39330U, 0x24b4a3a6U, 0xbad03605U,
    0xcdd70693U, 0x54de5729U, 0x23d967bfU, 0xb3667a2eU, 0xc4614ab8U,
    0x5d681b02U, 0x2a6f2b94U, 0xb40bbe37U, 0xc30c8ea1U, 0x5a05df1bU,
    0x2d02ef8dU
  };

  if (isFirstCall)
  {
    startValue = 0xFFFFFFFFU;
  }
  else
  {
    /* undo the XOR on the start value */
    startValue ^= 0xFFFFFFFFU;

    /* The reflection of the initial value is not necessary here as we used
     * the "reflected" algorithm and reflected table values. */
  }

  /* Process all data byte-wise */
  while (len != 0U)
  {

    /* Process one byte of data */
    startValue = crc32table[((uint8_t)startValue) ^ *data] ^ (startValue >> 8U);
    /* Advance the pointer and decrease remaining bytes to calculate over
     * until all bytes in the buffer have been used as input */
    ++data;
    --len;
  } /* while (u32Length != 0U) */

  /* The reflection of the remainder is not necessary here as we used the
   * "reflected" algorithm and reflected table values. */
  startValue ^= 0xFFFFFFFFU; /* XOR crc value */

  return startValue;
}

inline bool isCrc32Correct(const uint8_t* pkt, size_t size)
{
  //
  // packet format
  //
  // | packet header + packet data | crc32    |  rolling_counter |
  // | n bytes                     | 4 bytes  |  2 bytes         |
  //
  uint32_t expected;
  expected = calcCrc32 (pkt, size - 6, 0/* ignored */, true);
  expected = calcCrc32 (pkt + size - 2, 2, expected, false);

  uint32_t actual = *(uint32_t*)(pkt + size - 6);
  actual = htonl(actual);

  return (expected == actual);
}

}  // namespace lidar
}  // namespace asensing
