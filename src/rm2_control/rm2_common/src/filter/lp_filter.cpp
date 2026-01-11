#include "rm2_common/filters/lp_filter.h"
#include <cmath>
LowPassFilter::LowPassFilter(double cutoff_freq)
{
  cutoff_frequency_ = cutoff_freq;
}

void LowPassFilter::input(double in, double time)
{
  // My filter reference was Julius O. Smith III, Intro. to Digital Filters
  // With Audio Applications.
  // See https://ccrma.stanford.edu/~jos/filters/Example_Second_Order_Butterworth_Lowpass.html
  in_[2] = in_[1];
  in_[1] = in_[0];
  in_[0] = in;

  if (prev_time_ != 0)  // Not first time through the program
  {
    delta_t_ = time - prev_time_;
    prev_time_ = time;
    if (delta_t_ == 0)
    {
      // ROS_ERROR("delta_t is 0, skipping this loop. Possible overloaded cpu at time: %f", time.toSec());
      return;
    }
  }
  else
  {
    prev_time_ = time;
    return;
  }

  if (cutoff_frequency_ != -1 && cutoff_frequency_ > 0)
  {
    // Check if tan(_) is really small, could cause c = NaN
    tan_filt_ = tan((cutoff_frequency_ * 6.2832) * delta_t_ / 2.);
    // Avoid tan(0) ==> NaN
    if ((tan_filt_ <= 0.) && (tan_filt_ > -0.01))
      tan_filt_ = -0.01;
    if ((tan_filt_ >= 0.) && (tan_filt_ < 0.01))
      tan_filt_ = 0.01;
    c_ = 1 / tan_filt_;
  }

  out_[2] = out_[1];
  out_[1] = out_[0];
  out_[0] = (1 / (1 + c_ * c_ + M_SQRT2 * c_)) *
            (in_[2] + 2 * in_[1] + in_[0] - (c_ * c_ - M_SQRT2 * c_ + 1) * out_[2] - (-2 * c_ * c_ + 2) * out_[1]);
}

double LowPassFilter::output()
{
  return out_[0];
}

void LowPassFilter::reset()
{
  for (int i = 0; i < 3; ++i)
  {
    in_[i] = 0;
    out_[i] = 0;
  }
}