#pragma once

#include "ag_driver/driver/input/input.hpp"

#include <winsock2.h>
#include <ws2tcpip.h>

#pragma warning(disable : 4244)

namespace asensing
{
namespace lidar
{
class InputSock : public Input
{
public:
  InputSock(const AGInputParam& input_param)
    : Input(input_param), pkt_buf_len_(ETH_LEN),
      sock_offset_(0), sock_tail_(0)
  {
    sock_offset_ += input_param.user_layer_bytes;
    sock_tail_   += input_param.tail_layer_bytes;
  }

  virtual bool init();
  virtual bool start();
  virtual ~InputSock();

private:
  inline void recvPacket();
  inline int createSocket(uint16_t port, const std::string& hostIp, const std::string& grpIp);

protected:
  size_t pkt_buf_len_;
  int fds_[2];
  size_t sock_offset_;
  size_t sock_tail_;
};

inline bool InputSock::init()
{
  if (init_flag_)
  {
    return true;
  }

  int msop_fd = -1, difop_fd = -1;

  WORD version = MAKEWORD(2, 2);
  WSADATA wsaData;
  int ret = WSAStartup(version, &wsaData);
  if(ret < 0)
    goto failWsa;

  msop_fd = createSocket(input_param_.msop_port, input_param_.host_address, input_param_.group_address);
  if (msop_fd < 0)
    goto failMsop;

  if ((input_param_.difop_port != 0) && (input_param_.difop_port != input_param_.msop_port))
  {
    difop_fd = createSocket(input_param_.difop_port, input_param_.host_address, input_param_.group_address);
    if (difop_fd < 0)
      goto failDifop;
  }

  fds_[0] = msop_fd;
  fds_[1] = difop_fd;

  init_flag_ = true;
  return true;

failDifop:
  closesocket(msop_fd);
failMsop:
failWsa:
  return false;
}

inline bool InputSock::start()
{
  if (start_flag_)
  {
    return true;
  }

  if (!init_flag_)
  {
    cb_excep_(Error(ERRCODE_STARTBEFOREINIT));
    return false;
  }

  to_exit_recv_ = false;
  recv_thread_ = std::thread(std::bind(&InputSock::recvPacket, this));

  start_flag_ = true;
  return true;
}

inline InputSock::~InputSock()
{
  stop();

  closesocket(fds_[0]);
  if (fds_[1] >= 0)
    closesocket(fds_[1]);
}

inline int InputSock::createSocket(uint16_t port, const std::string& hostIp, const std::string& grpIp)
{
  int fd;
  int ret;
  int reuse = 1;

  fd = socket(PF_INET, SOCK_DGRAM, 0);
  if (fd < 0)
  {
    perror("socket: ");
    goto failSocket;
  }

  ret = setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse));
  if (ret < 0)
  {
    perror("setsockopt(SO_REUSEADDR): ");
    goto failOption;
  }

  struct sockaddr_in host_addr;
  memset(&host_addr, 0, sizeof(host_addr));
  host_addr.sin_family = AF_INET;
  host_addr.sin_port = htons(port);
  host_addr.sin_addr.s_addr = INADDR_ANY;
  if (hostIp != "0.0.0.0" && grpIp == "0.0.0.0")
  {
    inet_pton(AF_INET, hostIp.c_str(), &(host_addr.sin_addr));
  }

  ret = bind(fd, (struct sockaddr*)&host_addr, sizeof(host_addr));
  if (ret < 0)
  {
    perror("bind: ");
    goto failBind;
  }

  if (grpIp != "0.0.0.0")
  {
#if 0
    struct ip_mreqn ipm;
    memset(&ipm, 0, sizeof(ipm));
    inet_pton(AF_INET, grpIp.c_str(), &(ipm.imr_multiaddr));
    inet_pton(AF_INET, hostIp.c_str(), &(ipm.imr_address));
#else
    struct ip_mreq ipm;
    inet_pton(AF_INET, grpIp.c_str(), &(ipm.imr_multiaddr));
    inet_pton(AF_INET, hostIp.c_str(), &(ipm.imr_interface));
#endif
    ret = setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (const char*)&ipm, sizeof(ipm));
    if (ret < 0)
    {
      perror("setsockopt(IP_ADD_MEMBERSHIP): ");
      goto failGroup;
    }
  }

#ifdef ENABLE_DOUBLE_RCVBUF
  {
    uint32_t opt_val;
    socklen_t opt_len = sizeof(uint32_t);
    getsockopt(fd, SOL_SOCKET, SO_RCVBUF, (char*)&opt_val, &opt_len);
    opt_val *= 4;
    setsockopt(fd, SOL_SOCKET, SO_RCVBUF, (char*)&opt_val, opt_len);
  }
#endif

  {
    u_long mode = 1;
    ret = ioctlsocket(fd, FIONBIO, &mode);
    if (ret < 0)
    {
      perror("ioctlsocket: ");
      goto failNonBlock;
    }
  }

  return fd;

failNonBlock:
failGroup:
failBind:
failOption:
  closesocket(fd);
failSocket:
  return -1;
}

inline void InputSock::recvPacket()
{
  int max_fd = ((fds_[0] > fds_[1]) ? fds_[0] : fds_[1]);

  while (!to_exit_recv_)
  {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fds_[0], &rfds);
    if (fds_[1] >= 0)
      FD_SET(fds_[1], &rfds);

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    int retval = select(max_fd + 1, &rfds, NULL, NULL, &tv);
    if (retval == 0)
    {
      cb_excep_(Error(ERRCODE_MSOPTIMEOUT));
      continue;
    }
    else if (retval < 0)
    {
      if (errno == EINTR)
        continue;

      perror("select: ");
      break;
    }

    for (int i = 0; i < 2; i++)
    {
      if ((fds_[i] >= 0) && FD_ISSET(fds_[i], &rfds))
      {
        std::shared_ptr<Buffer> pkt = cb_get_pkt_(pkt_buf_len_);
        int ret = recvfrom(fds_[i], (char*)pkt->buf(), (int)pkt->bufSize(), 0, NULL, NULL);
        if (ret < 0)
        {
          perror("recvfrom: ");
          break;
        }
        else if (ret > 0)
        {
          pkt->setData(sock_offset_, ret - sock_offset_ - sock_tail_);
          pushPacket(pkt);
        }
      }
    }
  }
}

}  // namespace lidar
}  // namespace asensing
