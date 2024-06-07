#pragma once

#include "ag_driver/driver/driver_param.hpp"
#include "ag_driver/msg/packet.hpp"
#include "ag_driver/common/error_code.hpp"
#include "ag_driver/common/version.hpp"
#include "ag_driver/common/sync_queue.hpp"
#include "ag_driver/common/buffer.hpp"
#include "ag_driver/driver/input/input_factory.hpp"
#include "ag_driver/driver/decoder/decoder_factory.hpp"
#include <sstream>

namespace asensing
{
namespace lidar
{

inline std::string getDriverVersion()
{
  std::stringstream stream;
  stream << AGLIDAR_VERSION_MAJOR << "." << AGLIDAR_VERSION_MINOR << "." << AGLIDAR_VERSION_PATCH;
  return stream.str();
}

template <typename T_PointCloud>
class LidarDriverImpl
{
public:

  LidarDriverImpl();
  ~LidarDriverImpl();

  void regPointCloudCallback(
      const std::function<std::shared_ptr<T_PointCloud>(void)>& cb_get_cloud,
      const std::function<void(std::shared_ptr<T_PointCloud>)>& cb_put_cloud);
  void regPacketCallback(const std::function<void(const Packet&)>& cb_put_pkt);
  void regExceptionCallback(const std::function<void(const Error&)>& cb_excep);
 
  bool init(const AGDriverParam& param);
  bool start();
  void stop();

  void decodePacket(const Packet& pkt);
  bool getTemperature(float& temp);
  bool getDeviceInfo(DeviceInfo& info);
  bool getDeviceStatus(DeviceStatus& status);
  bool getSelectionParam(SelectionParam& param);
  bool setSelectionParam(SelectionParam& param);
  bool setDefaultSelectionParam();
  
  void implChangeViewerStopFlag(bool value);
  double implGetFrameRate();
  void implChangeInitFlag(bool value);

  bool imu_motion_compensation_flag =false;

private:

  void runPacketCallBack(uint8_t* data, size_t data_size, double timestamp, uint8_t is_difop, uint8_t is_frame_begin);
  void runExceptionCallback(const Error& error);

  std::shared_ptr<Buffer> packetGet(size_t size);
  void packetPut(std::shared_ptr<Buffer> pkt, bool stuffed);

  void processPacket();

  std::shared_ptr<T_PointCloud> getPointCloud();
  void splitFrame(uint16_t height, double ts);
  void setPointCloudHeader(std::shared_ptr<T_PointCloud> msg, uint16_t height, double chan_ts);

  AGDriverParam driver_param_;
  std::function<std::shared_ptr<T_PointCloud>(void)> cb_get_cloud_;
  std::function<void(std::shared_ptr<T_PointCloud>)> cb_put_cloud_;
  std::function<void(const Packet&)> cb_put_pkt_;
  std::function<void(const Error&)> cb_excep_;
  std::function<void(const uint8_t*, size_t)> cb_feed_pkt_;

  std::shared_ptr<Input> input_ptr_;
  std::shared_ptr<Decoder<T_PointCloud>> decoder_ptr_;
  SyncQueue<std::shared_ptr<Buffer>> free_pkt_queue_;
  SyncQueue<std::shared_ptr<Buffer>> pkt_queue_;
  std::thread handle_thread_;
  uint32_t pkt_seq_;
  uint32_t point_cloud_seq_;
  bool to_exit_handle_;
  bool init_flag_;
  bool start_flag_;
};

template <typename T_PointCloud>
inline LidarDriverImpl<T_PointCloud>::LidarDriverImpl()
  : pkt_seq_(0), point_cloud_seq_(0), init_flag_(false), start_flag_(false)
{
}

template <typename T_PointCloud>
inline LidarDriverImpl<T_PointCloud>::~LidarDriverImpl()
{
  stop();
}

template <typename T_PointCloud>
std::shared_ptr<T_PointCloud> LidarDriverImpl<T_PointCloud>::getPointCloud()
{
  while (1)
  {
    std::shared_ptr<T_PointCloud> cloud = cb_get_cloud_();
    if (cloud)
    {
      cloud->points.resize(0);
      return cloud;
    }

    LIMIT_CALL(runExceptionCallback(Error(ERRCODE_POINTCLOUDNULL)), 1);
  }
}

template <typename T_PointCloud>
void LidarDriverImpl<T_PointCloud>::regPointCloudCallback( 
    const std::function<std::shared_ptr<T_PointCloud>(void)>& cb_get_cloud,
    const std::function<void(std::shared_ptr<T_PointCloud>)>& cb_put_cloud) 
{
  cb_get_cloud_ = cb_get_cloud;
  cb_put_cloud_ = cb_put_cloud;
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::regPacketCallback(
    const std::function<void(const Packet&)>& cb_put_pkt)
{
  cb_put_pkt_ = cb_put_pkt;
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::regExceptionCallback(
    const std::function<void(const Error&)>& cb_excep)
{
  cb_excep_ = cb_excep;
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::implChangeViewerStopFlag(bool value)
{
  input_ptr_->ChangeViewerStopFlag(value);
}

template <typename T_PointCloud>
inline double LidarDriverImpl<T_PointCloud>::implGetFrameRate()
{
  return input_ptr_->GetFrameRate();
}

template <typename T_PointCloud>
inline bool LidarDriverImpl<T_PointCloud>::init(const AGDriverParam& param)
{
  if (init_flag_)
  {
    return true;
  }

  //
  // decoder
  //
  decoder_ptr_ = DecoderFactory<T_PointCloud>::createDecoder(param.lidar_type, param.decoder_param);

  if (decoder_ptr_.get() == nullptr)
  {
    return false;
  }
  
#if 0
  // rewrite pkt timestamp or not ?
  decoder_ptr_->enableWritePktTs((cb_put_pkt_ == nullptr) ? false : true);  //wrong: Initialize before register, so it will always be false
#endif

  // point cloud related
  decoder_ptr_->point_cloud_ = getPointCloud();
  decoder_ptr_->regCallback( 
      std::bind(&LidarDriverImpl<T_PointCloud>::runExceptionCallback, this, std::placeholders::_1),
      std::bind(&LidarDriverImpl<T_PointCloud>::splitFrame, this, std::placeholders::_1, std::placeholders::_2));

  double packet_duration = decoder_ptr_->getPacketDuration();

  //
  // input
  //
  input_ptr_ = InputFactory::createInput(param.input_type, param.input_param, packet_duration, cb_feed_pkt_);

  if (input_ptr_.get() == nullptr)
  {
    goto failInputInit;
  }

  input_ptr_->regCallback(
      std::bind(&LidarDriverImpl<T_PointCloud>::runExceptionCallback, this, std::placeholders::_1), 
      std::bind(&LidarDriverImpl<T_PointCloud>::packetGet, this, std::placeholders::_1), 
      std::bind(&LidarDriverImpl<T_PointCloud>::packetPut, this, std::placeholders::_1, std::placeholders::_2));

  input_ptr_->ChangeViewerStopFlag(false);

  if (!input_ptr_->init())
  {
    goto failInputInit;
  }

  driver_param_ = param;
  init_flag_ = true;
  return true;

failInputInit:
  input_ptr_.reset();
  decoder_ptr_.reset();
  return false;
}

template <typename T_PointCloud>
inline bool LidarDriverImpl<T_PointCloud>::start()
{
  if (start_flag_)
  {
    return true;
  }

  if (!init_flag_)
  {
    return false;
  }

  to_exit_handle_ = false;

  handle_thread_ = std::thread(std::bind(&LidarDriverImpl<T_PointCloud>::processPacket, this));

  input_ptr_->start();

  start_flag_ = true;
  return true;
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::implChangeInitFlag(bool value)
{
  init_flag_ = value;
  input_ptr_->ChangeInitFlag(value);
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::stop()
{
  if (!start_flag_)
  {
    return;
  }

  input_ptr_->stop();

  to_exit_handle_ = true;
  handle_thread_.join();

  // clear all points before next session
  if (decoder_ptr_->point_cloud_)
  {
    decoder_ptr_->point_cloud_->points.clear();
  }

  start_flag_ = false;
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::decodePacket(const Packet& pkt)
{
  cb_feed_pkt_(pkt.buf_.data(), pkt.buf_.size());
}

template <typename T_PointCloud>
inline bool LidarDriverImpl<T_PointCloud>::getTemperature(float& temp)
{
  if (decoder_ptr_ == nullptr)
  {
    return false;
  }

  temp = decoder_ptr_->getTemperature();
  return true;
}

template <typename T_PointCloud>
inline bool LidarDriverImpl<T_PointCloud>::getDeviceInfo(DeviceInfo& info)
{
  if (decoder_ptr_ == nullptr)
  {
    return false;
  }

  return decoder_ptr_->getDeviceInfo(info);
}

template <typename T_PointCloud>
inline bool LidarDriverImpl<T_PointCloud>::getDeviceStatus(DeviceStatus& status)
{
  if (decoder_ptr_ == nullptr)
  {
    return false;
  }

  return decoder_ptr_->getDeviceStatus(status);
}

template <typename T_PointCloud>
inline bool LidarDriverImpl<T_PointCloud>::getSelectionParam(SelectionParam& param)
{
  if (decoder_ptr_ == nullptr)
  {
    return false;
  }

  return decoder_ptr_->getSelectionParam(param);
}

template <typename T_PointCloud>
inline bool LidarDriverImpl<T_PointCloud>::setSelectionParam(SelectionParam& param)
{
  // update decoder parameter, but not necessary
  driver_param_.decoder_param.min_distance = param.min_distance;
  driver_param_.decoder_param.max_distance = param.max_distance;
  driver_param_.decoder_param.min_azimuth = param.min_azimuth;
  driver_param_.decoder_param.max_azimuth = param.max_azimuth;
  driver_param_.decoder_param.min_elevation = param.min_elevation;
  driver_param_.decoder_param.max_elevation = param.max_elevation;
  driver_param_.decoder_param.min_intensity = param.min_intensity;
  driver_param_.decoder_param.max_intensity = param.max_intensity;
  driver_param_.decoder_param.min_module = param.min_module;
  driver_param_.decoder_param.max_module = param.max_module;

  decoder_ptr_->setSelectionParam(param);

  return true;
}

template <typename T_PointCloud>
inline bool LidarDriverImpl<T_PointCloud>::setDefaultSelectionParam()
{
  decoder_ptr_->setDefaultSelectionParam();
  return true;
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::runPacketCallBack(uint8_t* data, size_t data_size,
    double timestamp, uint8_t is_difop, uint8_t is_frame_begin)
{
  if (cb_put_pkt_)
  {
    Packet pkt;
    pkt.timestamp = timestamp;
    pkt.is_difop = is_difop;
    pkt.is_frame_begin = is_frame_begin;
    pkt.seq = pkt_seq_++;
    pkt.frame_id = driver_param_.frame_id;

    pkt.buf_.resize(data_size);
    memcpy (pkt.buf_.data(), data, data_size);
    cb_put_pkt_(pkt);
  }
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::runExceptionCallback(const Error& error)
{
  if (cb_excep_)
  {
    cb_excep_(error);
  }
}

template <typename T_PointCloud>
inline std::shared_ptr<Buffer> LidarDriverImpl<T_PointCloud>::packetGet(size_t size)
{
  std::shared_ptr<Buffer> pkt = free_pkt_queue_.pop();
  if (pkt.get() != NULL)
  {
    return pkt;
  }

  return std::make_shared<Buffer>(size);
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::packetPut(std::shared_ptr<Buffer> pkt, bool stuffed)
{
  constexpr static int PACKET_POOL_MAX = 1024;

  if (!stuffed)
  {
    free_pkt_queue_.push(pkt);
    return;
  }

  size_t sz = pkt_queue_.push(pkt);
  if (sz > PACKET_POOL_MAX)
  {
    LIMIT_CALL(runExceptionCallback(Error(ERRCODE_PKTBUFOVERFLOW)), 1);
    pkt_queue_.clear();
  }
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::processPacket()
{
  static const uint8_t msop_id[] =  {0xAA, 0x55};
  static const uint8_t msop_id2[] = {0x55, 0xA2};
  static const uint8_t difop_id[] = {0xA5, 0xFF};

#ifndef WIN32
  pthread_setname_np(pthread_self(), "process_packet");
#endif

#ifdef ENABLE_THREAD_PRIORITY
  AG_SetThreadPriority(SCHED_RR, HIGH_THREAD_PRIORITY);
#endif

  while (!to_exit_handle_)
  {
    std::shared_ptr<Buffer> pkt = pkt_queue_.popWait(500000);
    if (pkt.get() == NULL)
    {
      continue;
    }

    uint8_t* id = pkt->data();
    if (memcmp(id, msop_id, sizeof(msop_id)) == 0 || memcmp(id, msop_id2, sizeof(msop_id2)) == 0)
    {
      bool pkt_to_split = decoder_ptr_->processMsopPkt(pkt->data(), pkt->dataSize());
      runPacketCallBack(pkt->data(), pkt->dataSize(), decoder_ptr_->prevPktTs(), false, pkt_to_split); // msop packet
    }
    else if(memcmp(id, difop_id, sizeof(difop_id)) == 0)
    {
      decoder_ptr_->processDifopPkt(pkt->data(), pkt->dataSize());
      runPacketCallBack(pkt->data(), pkt->dataSize(), 0, true, false); // difop packet
    }

    free_pkt_queue_.push(pkt);
  }
}

template <typename T_PointCloud>
void LidarDriverImpl<T_PointCloud>::splitFrame(uint16_t height, double ts)
{
  std::shared_ptr<T_PointCloud> cloud = decoder_ptr_->point_cloud_;
  if (cloud->points.size() > 0)
  {
    setPointCloudHeader(cloud, height, ts);
    //Put "cloud" in  "point_cloud_queue_"
    cb_put_cloud_(cloud);
    //getPointCloud() clear "cloud" and return it to "decoder_ptr_->point_cloud" for reuse
    decoder_ptr_->point_cloud_ = getPointCloud();
  }
  else
  {
    runExceptionCallback(Error(ERRCODE_ZEROPOINTS));
  }
}

template <typename T_PointCloud>
void LidarDriverImpl<T_PointCloud>::setPointCloudHeader(std::shared_ptr<T_PointCloud> msg, 
    uint16_t height, double ts)
{
  msg->seq = point_cloud_seq_++;
  msg->timestamp = ts;
  msg->is_dense = driver_param_.decoder_param.dense_points;

  if (msg->is_dense)
  {
    msg->height = 1;
    msg->width = (uint32_t)msg->points.size();
  }
  else
  {
    msg->height = height;
    msg->width = (uint32_t)msg->points.size() / msg->height;
  }

  msg->frame_id = driver_param_.frame_id;
}

}  // namespace lidar
}  // namespace asensing
