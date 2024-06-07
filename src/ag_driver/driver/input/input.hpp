#pragma once

#include "ag_driver/driver/driver_param.hpp"
#include "ag_driver/common/buffer.hpp"

#include <functional>
#include <thread>
#include <cstring>

#define VLAN_HDR_LEN  4
#define ETH_HDR_LEN   42
#define ETH_LEN       (ETH_HDR_LEN + VLAN_HDR_LEN + 1500)
#define IP_LEN        65536 
#define UDP_HDR_LEN   8

namespace asensing
{
namespace lidar
{
class Input
{
public:
  Input(const AGInputParam& input_param);

  inline void regCallback(
      const std::function<void(const Error&)>& cb_excep,
      const std::function<std::shared_ptr<Buffer>(size_t)>& cb_get_pkt,
      const std::function<void(std::shared_ptr<Buffer>, bool)>& cb_put_pkt);

  virtual bool init() = 0;
  virtual bool start() = 0;
  virtual void stop();
  virtual ~Input()
  {
  }

  void ChangeViewerStopFlag(bool value);
  double GetFrameRate();
  void ChangeInitFlag(bool value);

protected:
  inline void pushPacket(std::shared_ptr<Buffer> pkt, bool stuffed = true);

  AGInputParam input_param_;
  std::function<std::shared_ptr<Buffer>(size_t size)> cb_get_pkt_;
  std::function<void(std::shared_ptr<Buffer>, bool)> cb_put_pkt_;
  std::function<void(const Error&)> cb_excep_;
  std::thread recv_thread_;
  bool to_exit_recv_;
  bool init_flag_;
  bool start_flag_;

  bool viewer_stop_flag_;
  double frame_rate;
};

inline Input::Input(const AGInputParam& input_param)
  : input_param_(input_param), to_exit_recv_(false), 
  init_flag_(false), start_flag_(false)
{
}

inline void Input::regCallback(
    const std::function<void(const Error&)>& cb_excep,
    const std::function<std::shared_ptr<Buffer>(size_t)>& cb_get_pkt, 
    const std::function<void(std::shared_ptr<Buffer>, bool)>& cb_put_pkt)
{
  cb_excep_   = cb_excep;
  cb_get_pkt_ = cb_get_pkt;
  cb_put_pkt_ = cb_put_pkt;
}


inline void Input::ChangeViewerStopFlag(bool value)
{
  viewer_stop_flag_ = value;
}

inline double Input::GetFrameRate()
{
  return frame_rate;
}

inline void Input::ChangeInitFlag(bool value)
{
  init_flag_  = value;
}

inline void Input::stop()
{
  if (start_flag_)
  {
    to_exit_recv_ = true;
    recv_thread_.join();

    start_flag_ = false;
  }
}

inline void Input::pushPacket(std::shared_ptr<Buffer> pkt, bool stuffed)
{
  cb_put_pkt_(pkt, stuffed);
}

}  // namespace lidar
}  // namespace asensing
