#ifndef __RTT_ROSCLOCK_PROF_H__
#define __RTT_ROSCLOCK_PROF_H__

#include <queue>
#include <rtt_rosclock/rtt_rosclock.h>

namespace rtt_rosclock {
  class WallProf {
  public:
    typedef std::pair<ros::Time, ros::Time> TicToc;

    WallProf(double memory) :
      memory_(memory)
    {

    }

    void tic()
    {
      last_tic = rtt_rosclock::rtt_wall_now();
    }

    void toc()
    {
      ros::Time toc = rtt_rosclock::rtt_wall_now();
      tictocs_.push_back(std::make_pair(last_tic, toc));
      while(tictocs_.size() > 1 && (toc - tictocs_.front().first).toSec() > memory_) {
        tictocs_.pop_front();
      }
    }

    ros::Duration last()
    {
      TicToc &last_tictoc = tictocs_.back();
      return last_tictoc.second - last_tictoc.first;
    }

    void analyze()
    {
      if(tictocs_.size() < 1) {
        return;
      }

      TicToc &last_tictoc = tictocs_.back();

      double sum = 0.0;
      double count = tictocs_.size();

      max_ = 0;
      min_ = last_tictoc.second.toSec();

      for(std::list<TicToc>::const_iterator it=tictocs_.begin(); it!=tictocs_.end(); ++it) {
        const double tictoc = (it->second - it->first).toSec();
        sum += tictoc;
        max_ = std::max(max_, tictoc);
        min_ = std::min(min_, tictoc);
      }

      mean_ = sum/count;

      double err_sum = 0.0;

      for(std::list<TicToc>::const_iterator it=tictocs_.begin(); it!=tictocs_.end(); ++it) {
        const double tictoc = (it->second - it->first).toSec();
        err_sum += std::pow(mean_ - tictoc, 2.0);
      }

      stddev_ = sqrt(err_sum/count);
    }

    double mean() { return mean_; }
    double min() { return min_; }
    double max() { return max_; }
    double stddev() { return stddev_; }
    size_t n() { return tictocs_.size(); }

  private:
    ros::Time last_tic;
    double memory_;
    std::list<TicToc> tictocs_;

    double mean_;
    double min_;
    double max_;
    double stddev_;
  };
}

#endif // ifndef __RTT_ROSCLOCK_PROF_H__
