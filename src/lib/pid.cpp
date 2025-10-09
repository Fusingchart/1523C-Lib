// pid.cpp

#include "../../include/lib/pid.hpp"

#include <limits>

PID::PID(double kp, double ki, double kd, double integral_limit, double output_min, double output_max)
    : kp_(kp), ki_(ki), kd_(kd), target_(0.0), prev_error_(0.0), integral_(0.0), last_output_(0.0),
      integral_limit_(std::abs(integral_limit)), out_min_(output_min), out_max_(output_max), error_(0.0) {}

void PID::setGains(double kp, double ki, double kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PID::setIntegralLimit(double limit) {
  integral_limit_ = std::abs(limit);
}

void PID::setOutputLimits(double out_min, double out_max) {
  out_min_ = out_min;
  out_max_ = out_max;
}

void PID::setTarget(double target) {
  target_ = target;
}

void PID::reset() {
  prev_error_ = 0.0;
  integral_ = 0.0;
  last_output_ = 0.0;
  error_ = 0.0;
}

double PID::update(double measurement, double dt) {
  if (dt <= 0.0) {
    // avoid division by zero or negative time steps
    return last_output_;
  }

  // error: target - measurement
  error_ = target_ - measurement;

  // Proportional
  double p = kp_ * error_;

  // Integral with anti-windup via clamping
  integral_ += error_ * dt;
  if (integral_ > integral_limit_) integral_ = integral_limit_;
  if (integral_ < -integral_limit_) integral_ = -integral_limit_;
  double i = ki_ * integral_;

  // Derivative (on error)
  double derivative = (error_ - prev_error_) / dt;
  double d = kd_ * derivative;

  double output = p + i + d;

  // Clamp output
  if (output > out_max_) output = out_max_;
  if (output < out_min_) output = out_min_;

  // Save state
  prev_error_ = error_;
  last_output_ = output;

  return output;
}

bool PID::atSetpoint(double tolerance) const {
  return std::abs(error_) <= std::abs(tolerance);
}
