#pragma once

class LowPassFilter {
public:
  explicit LowPassFilter(double cutoff_freq);
  void input(double in, double time);
  double output();
  void reset();

private:
  double in_[3]{};
  double out_[3]{};

  // Cutoff frequency for the derivative calculation in Hz.
  // Negative -> Has not been set by the user yet, so use a default.
  double cutoff_frequency_ = -1;
  // Used in filter calculations. Default 1.0 corresponds to a cutoff frequency
  // at 1/4 of the sample rate.
  double c_ = 1.;
  // Used to check for tan(0)==>NaN in the filter calculation
  double tan_filt_ = 1.;

  double prev_time_ = 0., delta_t_ = 0.;
};