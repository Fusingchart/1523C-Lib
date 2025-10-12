#include "lib/chassis.hpp"
#include "pros/rtos.hpp"

using namespace lib;

Chassis::Chassis(const Params& p)
    : left_motors_(std::vector<std::int8_t>(p.left_ports)), right_motors_(std::vector<std::int8_t>(p.right_ports)), imu_(p.imu_port), drive_pid_(p.drive_pid), turn_pidf_(p.turn_pidf), wheel_circumference_(p.wheel_circumference), track_width_(p.track_width) {
  // ensure IMU is reset (calibrate) non-blocking
  imu_.reset(false);

  // initialize odometry
  last_left_pos_ = left_motors_.get_position();
  last_right_pos_ = right_motors_.get_position();
  heading_ = imu_.get_heading();
}

void Chassis::setVoltage(int32_t v) {
  left_motors_.move(v);
  right_motors_.move(v);
}

void Chassis::stop() {
  left_motors_.brake();
  right_motors_.brake();
}
double Chassis::getHeading() const { return imu_.get_heading(); }

double Chassis::getLeftPosition() const { return left_motors_.get_position(); }

double Chassis::getRightPosition() const { return right_motors_.get_position(); }

// Update odometry: should be called periodically. Uses motor encoder positions and IMU heading.
void Chassis::updateOdometry() {
  double lpos = left_motors_.get_position();
  double rpos = right_motors_.get_position();

  // delta in encoder units (assumed to be rotations or consistent unit chosen by user)
  double dl = lpos - last_left_pos_;
  double dr = rpos - last_right_pos_;

  last_left_pos_ = lpos;
  last_right_pos_ = rpos;

  // Convert encoder delta to distance using wheel_circumference_
  double dsl = dl * wheel_circumference_;
  double dsr = dr * wheel_circumference_;

  // forward distance and change in heading (radians)
  double ds = 0.5 * (dsl + dsr);
  double dtheta = (dsr - dsl) / track_width_; // in same distance units / track width

  // current heading in radians
  double heading_rad = lib::utils::deg_to_rad(heading_);

  if (std::abs(dtheta) < 1e-6) {
    // approximate straight
    x_ += ds * std::cos(heading_rad);
    y_ += ds * std::sin(heading_rad);
  } else {
    // ICC based update
    double r = ds / dtheta;
    double cx = x_ - r * std::sin(heading_rad);
    double cy = y_ + r * std::cos(heading_rad);

    heading_rad += dtheta;
    x_ = cx + r * std::sin(heading_rad);
    y_ = cy - r * std::cos(heading_rad);
  }

  // update heading from IMU for better absolute heading (use IMU instead of integrating)
  heading_ = imu_.get_heading();
}

void Chassis::resetOdometry(double x, double y, double heading_deg) {
  x_ = x;
  y_ = y;
  heading_ = heading_deg;
  last_left_pos_ = left_motors_.get_position();
  last_right_pos_ = right_motors_.get_position();
}

double Chassis::getX() const { return x_; }

double Chassis::getY() const { return y_; }

double Chassis::getHeadingInternal() const { return heading_; }
