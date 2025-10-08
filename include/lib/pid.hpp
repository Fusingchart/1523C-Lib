// pid.hpp

#pragma once

#include <algorithm>
#include <cmath>

class PID {
public:
  // Constructor: initialize gains and limits
  PID(double kp = 0.0, double ki = 0.0, double kd = 0.0,
      double integral_limit = 1e6, double output_min = -1e6, double output_max = 1e6);

  // Setters
  void setGains(double kp, double ki, double kd);
  void setIntegralLimit(double limit);
  void setOutputLimits(double out_min, double out_max);
  void setTarget(double target);

  // Reset internal state (integral, derivative history)
  void reset();

  // Main update function. Pass the current measurement and dt (seconds).
  // Returns the controller output (clamped to output limits).
  double update(double measurement, double dt);

  // Convenience: returns true if within tolerance of target
  bool atSetpoint(double tolerance) const;

  // Accessors
  double getError() const { return error_; }
  double getIntegral() const { return integral_; }
  double getLastOutput() const { return last_output_; }

private:
  double kp_;
  double ki_;
  double kd_;

  double target_;
  double prev_error_;
  double integral_;
  double last_output_;

  double integral_limit_;
  double out_min_;
  double out_max_;

  // temporary last computed error (for accessor)
  double error_;
};
