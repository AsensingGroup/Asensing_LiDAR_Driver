#pragma once

namespace asensing
{
namespace lidar
{
typedef enum {
  normal = 0,
  split = 1,
  discard = 2
} SplitStrategyEnum;

class SplitStrategyById
{
public:

  SplitStrategyById()
    : prev_frame_(0), prev_seq_(0)
  {
    
  }

  SplitStrategyEnum newPacket(uint32_t frame, uint16_t seq)
  {
    if( !init_ )
    {
      init_ = true;
      prev_frame_ = frame;
      prev_seq_ = seq;
      return SplitStrategyEnum::normal;      
    }

    if( frame == prev_frame_ )
    {
      prev_seq_ = seq;
      return SplitStrategyEnum::normal;
    }
    else if( frame < prev_frame_ )
    { 
      if( frame == 0 )
      {
        prev_frame_ = frame;
        prev_seq_ = seq;
        return SplitStrategyEnum::split;
      }
      else
      {
        return SplitStrategyEnum::discard;
      }
    }
    else /*if( (frame > prev_frame_ )*/
    {
      if( (prev_frame_ > 0) || (prev_frame_==0 && seq < prev_seq_))
      {
        prev_frame_ = frame;
        prev_seq_ = seq;
        return SplitStrategyEnum::split;
      }
      else
      {
        return SplitStrategyEnum::discard;
      }
    }

    return SplitStrategyEnum::discard;
  }

#ifndef UNIT_TEST
private:
#endif
  bool init_ = false;
  uint32_t prev_frame_;
  uint16_t prev_seq_;
};

class SplitStrategyBySeq
{
public:

  SplitStrategyBySeq()
    : prev_frame_(0), prev_seq_(0)
  {
    setSafeRange();
  }

  bool newPacket(uint32_t frame, uint16_t seq)
  {
    bool split = false;

    if (seq < safe_seq_min_) 
    {
      prev_seq_ = seq;
      split = true;
    }
    else if (seq < prev_seq_) //min < seq < prev (The bag was switched in the range. no matter)
    {
      // do nothing.
    }
    else if (seq <= safe_seq_max_) //prev < seq < max (normal)
    {
      prev_seq_ = seq;
      prev_frame_ = frame;
    }
    else //max_ < seq (Serious packet loss)
    {
      if (prev_seq_ == 0) //initial
      {
        prev_seq_ = seq;
        prev_frame_ = frame;
      }

      //do nothing When it's lost, it's lost 
      //You can do packet loss statistics here
    }

    setSafeRange();
    return split;
  }

#ifndef UNIT_TEST
private:
#endif

  constexpr static uint16_t RANGE = 10;

  void setSafeRange()
  {
    safe_seq_min_ = (prev_seq_ > RANGE) ? (prev_seq_ - RANGE) : 0;
    safe_seq_max_ = prev_seq_ + RANGE;
  }

  uint32_t prev_frame_;
  uint16_t prev_seq_;
  uint16_t safe_seq_min_;
  uint16_t safe_seq_max_;
};

}  // namespace lidar
}  // namespace asensing
