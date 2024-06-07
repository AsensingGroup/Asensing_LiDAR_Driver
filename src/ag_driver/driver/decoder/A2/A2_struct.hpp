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

#define A2_CHANNELS_PER_BLOCK        (96)
#define A2_BLOCKS_PER_PACKET         (1)
#define A2_SIDES_OF_MIRROR           (4)

#pragma pack(push, 1)

/**********************
----Msop
***********************/
typedef struct
{
  uint8_t  Sob[2];            // package identification (0x55 0xA2)
  uint8_t  LidarType;         // 0x0(NA); 0x01(A0); 0x02(A2);
  uint8_t  LidarInfo;         // scan_mode:3 bits;   
                              //        0b000(MEMS-only); 0b001(Polygon-only);
                              // scan_module:3 bits; 
                              //        06000(M-MEMS); 0b001(A-MEMS); 0b010(X-MEMS)
                              // angle_mode:2 bits;  
                              //        0b00(Default); 0b01(Matrix_ON)
  uint8_t  Version;           // major version: 4 bits
                              // minor version: 4 bits
  uint8_t  BlockNum;          // the block num in one MSOP pkg //the real blocnum
  uint16_t ChannelNum;        // the channel num in one block, here is 96
  uint8_t  LiDARFlag1;        // WaveMode:4 bits 
                              //       Echo pattern:2bit;
                              //       The packet carries the callback:2bit;
                              // TimeSyncSource:2 bits; 
                              //       0b00(Inside); 0b01(gps); 0b10(PTP);0b11(reserved)
                              // TimeSyncState :2 bits; 
                              //       0b00(FreeRun); 0b01(Tracking); 0b10(Locked);0b11(Frozen)
  uint8_t  LiDARFlag2;
                              // AzimuthFlag :1 bit; 
                              //       0b00(No Azimuth/Elevation info in ChannelData. Azimuth is in the DataBockHeader); 
                              //       0b01(ChannelData has Azimuth/Elevation)
                              // SignatureFlag:1 bit; 
                              //       0b00(non); 0b01(has)
                              // FuSAFlag:1 bit; 
                              //       0b00(non); 0b01(has)
                              // IMUFlag:1 bit; 
                              //       0b00(non); 0b01(has)                    
                              // ConfidenceFlag:1 bit; 
                              //       0b00(non); 0b01(has)
                              // Reserved:3 bit
  uint32_t PointNumInFrame;
  uint16_t PktLen;            // package payload length
  uint16_t FrameSn;           // Frame sequnce num
  uint16_t PktSeq;            // package sequnce num
  
  AGTimestampUTC Timestamp;
  uint16_t ElevationMirrorOffset;

} A2MsopHeader; //32

typedef struct
{
  uint16_t distance;
  uint8_t  intensity; // reflectivity
  uint8_t  confidence;
} A2MsopChannel; //4

typedef struct
{
  uint16_t TimeOffset;
  uint16_t Azimuth;  // when AzimuthFlag is 0
  // uint16_t Elevation;
  A2MsopChannel channel[A2_CHANNELS_PER_BLOCK][2];
} A2MsopBlock; //4+4*96*2=772

typedef struct
{
  uint8_t  timestamp[4];
  uint16_t temperature;
  uint16_t AccelerationUnit;
  uint16_t AngularVelocityUnit;

  uint16_t X_axis_accceleration;
  uint16_t Y_axis_accceleration;
  uint16_t Z_axis_accceleration;

  uint16_t X_axis_angular_velocity;
  uint16_t Y_axis_angular_velocity;
  uint16_t Z_axis_angular_velocity;

  uint16_t Reserved;
  uint32_t CRC;
} A2MsopIMUData; //28

typedef struct
{
  uint8_t FuSAVersion;
  uint8_t LiDARState;            // init, normal, degrade...

  uint8_t LiDARTemperature;      // unit:1  -127~128 signed char
  uint8_t LaserTemperatureMax;   // unit:1  -127~128 signed char
  uint8_t DetectorTemperatureMax;// unit:1  -127~128 signed char
  uint8_t ScannerTemperature;    // unit:1  -127~128 signed char
  uint8_t WindowTemperature;     // unit:1  -127~128 signed char

  uint8_t Humidity;              // 0~100
  uint8_t TotalFaultNum;         // N=128
  uint8_t CurrentFaultNum;       // <N
  uint32_t AlarmWarningState;    // [(N+31)/32]*4 Bytes
                                 // bit0: AlarmCode on
  uint32_t CRC;
} A2MsopFuSAData; // 18

typedef struct
{
  uint64_t Reserved1;
  uint64_t Reserved2;
  uint64_t Reserved3;
  uint64_t Reserved4;
} A2MsopSignature; // 32

typedef struct
{
  A2MsopIMUData IMUData;
  A2MsopFuSAData FuSAData;
  A2MsopSignature Signature;
} A2MsopTail; // 28+18+32=78

typedef struct
{
  uint32_t CRC;
  uint32_t Reserved;
} A2MsopTail2; // 8

typedef struct
{
  A2MsopHeader header;
  A2MsopBlock  blocks[A2_BLOCKS_PER_PACKET];
  A2MsopTail2  tail;
} A2MsopPkt; // 32+772+8=812
#pragma pack(pop)

#pragma pack(push, 1)
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
} A2DifopEther;

typedef struct
{
  uint8_t horizontal_fov_start[2];
  uint8_t horizontal_fov_end[2];
  uint8_t vertical_fov_start[2];
  uint8_t vertical_fov_end[2];
} A2DifopFov;

typedef struct
{
  uint8_t sync_mode;
  uint8_t sync_sts;
  AGTimestampUTC timestamp;
} A2TimeInfo;

typedef struct
{
  uint8_t id[4];
  uint16_t frame_rate;
  uint16_t temperature;

  A2DifopEther eth;
  A2DifopFov fov;

  uint8_t firmware_ver[6];
  uint8_t sn[6];
  uint8_t return_mode;

  A2TimeInfo time_info;
  uint8_t reserved;

} A2DifopHead;

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

} A2DifopFuncSafety;

typedef struct
{
  uint8_t Signature[32];

} A2DifopNetSafety;

typedef struct
{
  A2DifopHead difop_head;
  A2DifopFuncSafety func_safety;
  A2DifopNetSafety net_safety;
} A2DifopPkt;

#pragma pack(pop)

}  // namespace lidar
}  // namespace asensing
