// utils.hpp

#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

namespace lib {
namespace utils {

// Clamp value to [min_val, max_val]
template <typename T>
inline T clamp(T v, T min_val, T max_val) {
  if (v < min_val) return min_val;
  if (v > max_val) return max_val;
  return v;
}

// Linearly remap a value from one range to another. If input range has zero
// length, returns out_min.
inline double map_range(double v, double in_min, double in_max, double out_min, double out_max) {
  if (in_max == in_min) return out_min;
  double t = (v - in_min) / (in_max - in_min);
  return out_min + t * (out_max - out_min);
}

// Sign of a number: -1, 0 or 1
template <typename T>
inline int sign(T v) {
  if (v > T(0)) return 1;
  if (v < T(0)) return -1;
  return 0;
}

// Deadband: zeroes small values
inline double deadband(double v, double db) {
  return (std::abs(v) <= std::abs(db)) ? 0.0 : v;
}

// Angle helpers (degrees)
inline double deg_to_rad(double deg) { return deg * M_PI / 180.0; }
inline double rad_to_deg(double rad) { return rad * 180.0 / M_PI; }

// Wrap angle in degrees to (-180, 180]
inline double wrap_angle_deg(double deg) {
  double a = std::fmod(deg, 360.0);
  if (a <= -180.0) a += 360.0;
  if (a > 180.0) a -= 360.0;
  return a;
}

// Wrap radians to (-pi, pi]
inline double wrap_angle_rad(double rad) {
  double a = std::fmod(rad, 2.0 * M_PI);
  if (a <= -M_PI) a += 2.0 * M_PI;
  if (a > M_PI) a -= 2.0 * M_PI;
  return a;
}

// Simple first-order low-pass filter (discrete-time). Use update(value, dt)
// where dt is seconds. The cutoff is specified in Hz.
class LowPassFilter {
public:
  LowPassFilter(double cutoff_hz = 1.0, double initial = 0.0)
      : cutoff_hz_(cutoff_hz), state_(initial) {}

  void reset(double v = 0.0) { state_ = v; }

  double update(double input, double dt) {
    if (dt <= 0.0 || cutoff_hz_ <= 0.0) {
      state_ = input;
      return state_;
    }
    const double rc = 1.0 / (2.0 * M_PI * cutoff_hz_);
    const double alpha = dt / (rc + dt);
    state_ += alpha * (input - state_);
    return state_;
  }

  double get() const { return state_; }

  void set_cutoff(double cutoff_hz) { cutoff_hz_ = cutoff_hz; }

private:
  double cutoff_hz_;
  double state_;
};

// Moving average filter (windowed). Update returns the running average.
class MovingAverage {
public:
  explicit MovingAverage(std::size_t window = 4) : window_(window), sum_(0.0) {
    if (window_ == 0) window_ = 1;
  }

  void reset() {
    buf_.clear();
    sum_ = 0.0;
  }

  double update(double v) {
    buf_.push_back(v);
    sum_ += v;
    if (buf_.size() > window_) {
      sum_ -= buf_.front();
      buf_.pop_front();
    }
    return sum_ / static_cast<double>(buf_.size());
  }

  std::size_t size() const { return buf_.size(); }

private:
  std::size_t window_;
  std::deque<double> buf_;
  double sum_;
};

// Slew rate limiter: limits how fast a value can change (units per second).
class SlewRateLimiter {
public:
  SlewRateLimiter(double rate_per_second = 1000.0, double initial = 0.0)
      : rate_(std::abs(rate_per_second)), state_(initial) {}

  void reset(double v = 0.0) { state_ = v; }

  // target: desired value; dt: seconds since last call
  double update(double target, double dt) {
    if (dt <= 0.0) return state_;
    double max_delta = rate_ * dt;
    double delta = target - state_;
    if (delta > max_delta) delta = max_delta;
    if (delta < -max_delta) delta = -max_delta;
    state_ += delta;
    return state_;
  }

  double get() const { return state_; }

  void set_rate(double r) { rate_ = std::abs(r); }

private:
  double rate_;
  double state_;
};

} // namespace utils
} // namespace lib
