#pragma once

#include "ag_driver/driver/input/input.hpp"
#include "ag_driver/driver/input/raw/input_raw.hpp"

#include "ag_driver/driver/input/socket/input_sock.hpp"

#ifndef DISABLE_PCAP_PARSE
#include "ag_driver/driver/input/pcap/input_pcap.hpp"

#endif

namespace asensing
{
namespace lidar
{

class InputFactory
{
public:
  static std::shared_ptr<Input> createInput(InputType type, const AGInputParam& param,
      double sec_to_delay, std::function<void(const uint8_t*, size_t)>& cb_feed_pkt);
};

inline std::shared_ptr<Input> InputFactory::createInput(InputType type, const AGInputParam& param,
    double sec_to_delay, std::function<void(const uint8_t*, size_t)>& cb_feed_pkt)
{
  std::shared_ptr<Input> input;

  switch(type)
  {
    case InputType::ONLINE_LIDAR:
      {
        input = std::make_shared<InputSock>(param);
      }
      break;

#ifndef DISABLE_PCAP_PARSE
    case InputType::PCAP_FILE:
      {
        input = std::make_shared<InputPcap>(param, sec_to_delay);
      }
      break;
#endif

    case InputType::RAW_PACKET:
      {
        std::shared_ptr<InputRaw> inputRaw;

        inputRaw = std::make_shared<InputRaw>(param);

        cb_feed_pkt = std::bind(&InputRaw::feedPacket, inputRaw, 
            std::placeholders::_1, std::placeholders::_2);

        input = inputRaw;
      }
      break;

    default:

      AG_ERROR << "Wrong Input Type " << type << "." << AG_REND;

      if (type == InputType::PCAP_FILE) 
      {
        AG_ERROR << "To use InputType::PCAP_FILE, please do not specify the make option DISABLE_PCAP_PARSE." << AG_REND;
      }

      input = nullptr;
  }

  return input;
}

}  // namespace lidar
}  // namespace asensing
