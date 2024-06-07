
#pragma once

#include <mutex>
#include <condition_variable>
#include <thread>
#include <queue>

namespace asensing
{
namespace lidar
{
template <typename T>
class SyncQueue
{
public:
  inline size_t push(const T& value)
  {
#ifndef ENABLE_WAIT_IF_QUEUE_EMPTY
     bool empty = false;
#endif
     size_t size = 0;

    {
      std::lock_guard<std::mutex> lg(mtx_);
#ifndef ENABLE_WAIT_IF_QUEUE_EMPTY
      empty = queue_.empty();
#endif
      queue_.push(value);
      size = queue_.size();
    }

#ifndef ENABLE_WAIT_IF_QUEUE_EMPTY
    if (empty)
      cv_.notify_one();
#endif

    return size;
  }

  inline size_t size()
  {
    size_t size = 0;
    std::lock_guard<std::mutex> lg(mtx_);
    size = queue_.size();
    return size;
  }

  inline T pop()
  {
    T value;

    std::lock_guard<std::mutex> lg(mtx_);
    if (!queue_.empty())
    {
      value = queue_.front();
      queue_.pop();
    }

    return value;
  }

  inline T popWait(unsigned int usec = 1000000)
  {
    //
    // Low latency, or low CPU usage, that is the question. 
    //                                            - Hamlet

#ifdef ENABLE_WAIT_IF_QUEUE_EMPTY
    T value;

    {
      std::lock_guard<std::mutex> lg(mtx_);
      if (!queue_.empty())
      {
        value = queue_.front();
        queue_.pop();
        return value;
      }
    }

    std::this_thread::sleep_for(std::chrono::microseconds(1000));
    return value;
#else

    T value;

    std::unique_lock<std::mutex> ul(mtx_);
    cv_.wait_for(ul, std::chrono::microseconds(usec), [this] { return (!queue_.empty()); });

    if (!queue_.empty())
    {
      value = queue_.front();
      queue_.pop();
    }

    return value;
#endif
  }

  inline T back()
  {
    T value;

    std::lock_guard<std::mutex> lg(mtx_);
    if (!queue_.empty())
    {
      value = queue_.back();
    }
    return value;
  }

  inline void clear()
  {
    std::queue<T> empty;
    std::lock_guard<std::mutex> lg(mtx_);
    swap(empty, queue_);
  }

private:
  std::queue<T> queue_;
  std::mutex mtx_;
#ifndef ENABLE_WAIT_IF_QUEUE_EMPTY
  std::condition_variable cv_;
#endif
};
}  // namespace lidar
}  // namespace asensing
