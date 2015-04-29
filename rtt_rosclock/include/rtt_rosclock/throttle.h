#ifndef __RTT_ROSCLOCK_THROTTLE_H__
#define __RTT_ROSCLOCK_THROTTLE_H__

#include <rtt_rosclock/rtt_rosclock.h>

namespace rtt_rosclock {
  class WallThrottle {
  public:
    WallThrottle(ros::Duration period) :
      last_check_(0),
      period_(period)
    { }

    bool ready()
    {
      ros::Time now = rtt_rosclock::rtt_wall_now();
      if(now - last_check_ < period_) {
        return false;
      }
      last_check_ = now;
      return true;
    }

  private:
    ros::Time last_check_;
    ros::Duration period_;
  };
}

#endif // ifndef __RTT_ROSCLOCK_THROTTLE_H__
