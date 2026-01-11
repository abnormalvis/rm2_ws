//
// Created by ch on 2025/10/5.
//

#pragma once

#include <cmath>

template <typename T>
class OneEuroFilter
{
public:
  OneEuroFilter(double _freq, T _mincutoff, T _beta, T _dcutoff)
    : freq(_freq), mincutoff(_mincutoff), beta(_beta), dcutoff(_dcutoff)
  {
    firsttime = true;
    x_prev = 0;
    hatxprev = 0;
    dhatxprev = 0;
    filtered_val = 0;
  };

  ~OneEuroFilter() = default;

  void input(T input_value)
  {
    T dx = 0;
    if (!firsttime)
      dx = (input_value - x_prev) * freq;
    if (firsttime)
      dhatxprev = dx;
    T edx = alpha(dcutoff, freq) * dx + (1 - alpha(dcutoff, freq)) * dhatxprev;
    dhatxprev = edx;
    T cutoff = mincutoff + beta * std::abs(static_cast<double>(edx));

    if (firsttime)
      hatxprev = input_value;
    filtered_val = alpha(cutoff, freq) * input_value + (1 - alpha(cutoff, freq)) * hatxprev;
    hatxprev = filtered_val;
    firsttime = false;
  };

  T alpha(T cutoff, double freq)
  {
    T tau = 1.0 / (2 * M_PI * cutoff);
    T te = 1.0 / freq;
    return 1.0 / (1.0 + tau / te);
  }

  T output()
  {
    return filtered_val;
  };

  void clear()
  {
    firsttime = true;
    x_prev = 0;
    hatxprev = 0;
    dhatxprev = 0;
  };

private:
  double freq;
  bool firsttime;
  T mincutoff, beta, dcutoff;
  T x_prev, dhatxprev, hatxprev;
  T filtered_val;
};
