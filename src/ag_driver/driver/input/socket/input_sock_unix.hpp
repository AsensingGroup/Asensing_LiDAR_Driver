#pragma once

#include "ag_driver/driver/input/input.hpp"
#include "ag_driver/common/process_common.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/epoll.h>

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
  int epfd_;
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
  int epfd = epoll_create(1);
  if (epfd < 0)
    goto failEpfd;

  //
  // msop
  //
  {
    msop_fd = createSocket(input_param_.msop_port, input_param_.host_address, input_param_.group_address);
    if (msop_fd < 0)
      goto failMsop;

    struct epoll_event ev;
    ev.data.fd = msop_fd;
    ev.events = EPOLLIN; // level-triggered
    epoll_ctl (epfd, EPOLL_CTL_ADD, msop_fd, &ev);
  }

  //
  // difop
  //
  if ((input_param_.difop_port != 0) && (input_param_.difop_port != input_param_.msop_port))
  {
    difop_fd = createSocket(input_param_.difop_port, input_param_.host_address, input_param_.group_address);
    if (difop_fd < 0)
      goto failDifop;

    struct epoll_event ev;
    ev.data.fd = difop_fd;
    ev.events = EPOLLIN; // level-triggered
    epoll_ctl (epfd, EPOLL_CTL_ADD, difop_fd, &ev);
  }

  epfd_ = epfd;
  fds_[0] = msop_fd;
  fds_[1] = difop_fd;

  init_flag_ = true;
  return true;

failDifop:
  close(msop_fd);
failMsop:
  close(epfd);
failEpfd:
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

  close(fds_[0]);
  if (fds_[1] >= 0)
    close(fds_[1]);

  close(epfd_);
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

  ret = setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
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
    ret = setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &ipm, sizeof(ipm));
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
    int flags = fcntl(fd, F_GETFL, 0);
    ret = fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    if (ret < 0)
    {
      perror("fcntl: ");
      goto failNonBlock;
    }
  }

  return fd;

failNonBlock:
failGroup:
failBind:
failOption:
  close(fd);
failSocket:
  return -1;
}

inline void InputSock::recvPacket()
{
  pthread_setname_np(pthread_self(), "recv_packet");

#ifdef ENABLE_THREAD_PRIORITY
  AG_SetThreadPriority(SCHED_RR, HIGH_THREAD_PRIORITY);
#endif

  while (!to_exit_recv_)
  {
    struct epoll_event events[8];
    int retval = epoll_wait (epfd_, events, 8, 1000);
    if (retval == 0)
    {
      cb_excep_(Error(ERRCODE_MSOPTIMEOUT));
      continue;
    }
    else if (retval < 0)
    {
      if (errno == EINTR)
        continue;

       perror("epoll_wait: ");
      break;
    }

    for(int i = 0; i < retval; i++)
    {
      if (events[i].events & EPOLLIN)
      {
        std::shared_ptr<Buffer> pkt = cb_get_pkt_(pkt_buf_len_);
        ssize_t ret = recvfrom(events[i].data.fd, pkt->buf(), pkt->bufSize(), 0, NULL, NULL);
        if (ret < 0)
        {
          perror("recvfrom: ");
          goto failExit;
        }
        else if (ret > 0)
        {
          pkt->setData(sock_offset_, ret - sock_offset_ - sock_tail_);
          pushPacket(pkt);
        }
      }
    }
  }

failExit:
  return;
}

}  // namespace lidar
}  // namespace asensing
