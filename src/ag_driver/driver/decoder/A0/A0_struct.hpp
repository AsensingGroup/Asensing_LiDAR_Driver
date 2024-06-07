#pragma once

#include "ag_driver/driver/decoder/decoder.hpp"
#include "ag_driver/driver/decoder/split_strategy.hpp"

namespace asensing
{
namespace lidar
{

#define ASENSING_DISTANCE_UNIT       (0.01f)
#define ASENSING_AZIMUTH_UNIT        (0.01f)
#define ASENSING_ELEVATION_UNIT      (0.01f)

#define CHANNELNUM_A0     10
#define BLOCKNUM_A0       12

enum {
  A0VECTOR_SIZE = 3,
  A0ANGLE_SIZE = 6
};

#pragma pack(push, 1)

/**********************
----Msop
***********************/
typedef struct
{
  uint8_t  id[4];           // package identification
  uint32_t FrameSn;         // Frame sequnce num
  uint16_t pkt_seq;         // package sequnce num
  uint16_t PkgLen;          // package paload len
  uint8_t  LidarType;       // identify lidar type
                            // 0x0(NA); 0x01(A0); 0x02(A2);
  uint8_t  LidarInfo;       // scan_mode:3 bits;   0b000(MEMS-only); 0b001(Polygon-only);
                            // scan_module:3 bits; 06000(M-MEMS); 0b001(A-MEMS); 0b010(X-MEMS)
                            // angle_mode:2 bits;  0b00(Default); 0b01(Matrix_ON) 

  uint8_t  VersionMajor;    // identify the using protocol major version
  uint8_t  VersionMinor;    // identify the using protocol minor version

  AGTimestampUTC timestamp;

  uint8_t  MeasureMode;     // identify city or high way road
  uint8_t  LaserNum;        // the laser scan at the same time //the real lasenum
  uint8_t  BlockNum;        // the block num in one MSOP pkg //the real blocnum
  uint8_t  EchoCount;     
  uint8_t  TimeSyncMode;    // self clock signal、sync clock signal
  uint8_t  TimeSyncStat;    // state of clock signal
  uint8_t  MemsTemp;        // the temperature of lidar
  uint8_t  SlotNum;         // muti lidar identified by the slot num

  uint32_t pointsNumInFrame;
  uint16_t Reserved;
} A0MsopHeader;

typedef struct
{
  uint16_t distance;
  uint16_t azimuth;
  uint16_t elevation;
  uint8_t  intensity;
  uint8_t  confidence;
  uint8_t  reserved;
} A0MsopChannel;

typedef struct
{
  uint8_t channelNum;
  uint8_t return_seq;
  uint16_t time_offset;
  A0MsopChannel units[CHANNELNUM_A0];
} A0MsopBlock;

typedef struct
{
  uint8_t Reserved1;
  uint8_t Reserved2;
  uint8_t Reserved3;
  uint8_t Reserved4;
} A0MsopTail;

typedef struct
{
  A0MsopHeader header;
  A0MsopBlock blocks[BLOCKNUM_A0];
  A0MsopTail tail;
} A0MsopPkt;


/**********************
----Difop
***********************/
typedef struct
{
  uint8_t ip_source[4];
  uint8_t ip_destination[4];
  uint8_t mac_addr[6];
  uint8_t msop_port[2];
  uint8_t difop_port[2];
} A0DifopEther;

typedef struct
{
  uint8_t horizontal_fov_start[2];
  uint8_t horizontal_fov_end[2];
  uint8_t vertical_fov_start[2];
  uint8_t vertical_fov_end[2];
} A0DifopFov;

typedef struct
{
  uint8_t sync_mode;
  uint8_t sync_sts;
  AGTimestampUTC timestamp;
} AGTimeInfo;

typedef struct
{
  uint8_t id[4];
  uint16_t frame_rate;
  uint16_t temperature;

  A0DifopEther eth;
  A0DifopFov fov;

  uint8_t firmware_ver[6];
  uint8_t sn[6];
  uint8_t return_mode;

  AGTimeInfo time_info;
  uint8_t reserved;

} A0DifopHead;

typedef struct
{
  uint8_t ver;
  //uint8_t running_status;
  uint16_t lidar_status;
  uint8_t err_code;
  uint8_t fault_num;
  uint8_t fault_id;
  uint8_t reserved[6];
  uint8_t check[4];

} A0DifopFuncSafety;

typedef struct
{
  uint8_t Signature[32];

} A0DifopNetSafety;

typedef struct
{
  A0DifopHead difop_head;
  A0DifopFuncSafety func_safety;
  A0DifopNetSafety net_safety;
} A0DifopPkt;

#pragma pack(pop)

/* For filter algorithm */
#pragma pack(push, 1)
typedef struct
{
    uint16_t distance;
    uint16_t azimuth;
    uint16_t elevation;
    uint8_t reflectivity;
    uint8_t confidence;
    uint8_t rsv; // delete later
} PointT;

typedef struct
{
    PointT *points;
    int pointsNum;
} lidarPointCloud;

#pragma pack(pop)

}  // namespace lidar
}  // namespace asensing
