#pragma once

#include <memory>
#include "pros/motor_group.hpp"
#include "pros/imu.hpp"
#include "pid.hpp"
#include "pidf.hpp"
#include "utils.hpp"

namespace lib {

class Chassis {
public:
  struct Params {
    // port lists for motor groups (use negative for reversed)
    std::initializer_list<std::int8_t> left_ports{};
    std::initializer_list<std::int8_t> right_ports{};
    std::uint8_t imu_port = 1;

    // default PID/PIDF gains
    PID drive_pid = PID(0.8, 0.0, 0.0);
    PIDF turn_pidf = PIDF(3.0, 0.0, 0.0, 0.0);

    // Odometry parameters (units are user-defined, e.g., mm)
    double wheel_circumference = 1.0; // distance per motor rotation unit (set appropriately)
    double track_width = 1.0;         // distance between left and right wheels
  };

  // Construct with explicit motor port lists and optional IMU port
  Chassis(const Params& p);


  // Open-loop control: -127..127
  void setVoltage(int32_t v);
  void stop();

  // Simple getters
  double getHeading() const;
  double getLeftPosition() const;
  double getRightPosition() const;

  // Odometry API: update should be called periodically to integrate encoder deltas
  void updateOdometry();
  void resetOdometry(double x = 0.0, double y = 0.0, double heading_deg = 0.0);

  double getX() const;
  double getY() const;
  // Returns the internally stored heading (degrees) used by odometry
  double getHeadingInternal() const;

  // Expose controllers for tuning
  PID& drivePid() { return drive_pid_; }
  PIDF& turnPidf() { return turn_pidf_; }

private:
  pros::MotorGroup left_motors_;
  pros::MotorGroup right_motors_;
  pros::Imu imu_;

  PID drive_pid_;
  PIDF turn_pidf_;

  // Odometry state
  double x_{0.0};
  double y_{0.0};
  double heading_{0.0}; // degrees

  // odometry parameters
  double wheel_circumference_{1.0};
  double track_width_{1.0};

  // last encoder readings
  double last_left_pos_{0.0};
  double last_right_pos_{0.0};
};

} // namespace lib
