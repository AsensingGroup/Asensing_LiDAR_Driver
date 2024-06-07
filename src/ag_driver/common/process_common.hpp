
#pragma once

//
// define ntohs()
// 
#ifdef _WIN32
#include <ws2tcpip.h>
#else //__linux__
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/syscall.h>
#define gettid() syscall(SYS_gettid)
#endif

inline int16_t AG_SWAP_INT16(int16_t value)
{
  uint8_t* v = (uint8_t*)&value;

  uint8_t temp;
  temp = v[0];
  v[0] = v[1];
  v[1] = temp;

  return value;
}


//
//#define M_PI 3.14159265f
// 
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES // for VC++, required to use const M_IP in <math.h>
#endif

#include <math.h>

#define DEGREE_TO_RADIAN(deg)  ((deg) * M_PI / 180)
#define RADIAN_TO_DEGREE(rad)  ((rad) * 180 / M_PI)

// because the accuracy of point cloud is 0.01,the value processed has already been multiplied by 100.
// when another angle needs to use this ,it needs to be multiplied by 100
#define DEGREE_MULTIPLY_ACCURACY(deg)  ((deg) * 100)  

#ifdef ENABLE_THREAD_PRIORITY
inline bool AG_SetThreadPriority(int policy, int priority)
{
  struct sched_param sp;
  pthread_t tid = gettid();

  memset((void *)&sp, 0, sizeof(sp));
  sp.__sched_priority = priority;

  AG_DEBUG << "pthread " << tid << " set sched policy " << policy << ", priority " << priority << AG_REND;

  if (pthread_setschedparam(pthread_self(), policy, &sp) != 0)
  {
    AG_DEBUG << "pthread " << tid << " set sched policy failed!" << AG_REND;
  }

  return true;
}
#endif
