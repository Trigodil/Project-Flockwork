#pragma once

namespace swarm_control
{

// Single-axis PID controller.
// Integral term is clamped (anti-windup). Derivative acts on the
// measurement, not the error, to avoid a spike on setpoint changes.
class PidController
{
public:
  PidController(double kp, double ki, double kd, double integral_limit)
  : kp_(kp), ki_(ki), kd_(kd), integral_limit_(integral_limit)
  {
  }

  // Call once per control cycle. dt is seconds since the last call.
  double update(double setpoint, double measurement, double dt)
  {
    const double error = setpoint - measurement;

    integral_ += error * dt;
    if (integral_ > integral_limit_) {
      integral_ = integral_limit_;
    } else if (integral_ < -integral_limit_) {
      integral_ = -integral_limit_;
    }

    double derivative = 0.0;
    if (dt > 0.0) {
      derivative = (measurement - prev_measurement_) / dt;
    }
    prev_measurement_ = measurement;

    // Minus sign: derivative is on measurement, so a rising measurement damps the output, 
    // hopefully it does not crash, that would be very unfortunate.
    return kp_ * error + ki_ * integral_ - kd_ * derivative;
  }

  void reset()
  {
    integral_ = 0.0;
    prev_measurement_ = 0.0;
  }

private:
  double kp_;
  double ki_;
  double kd_;
  double integral_limit_;

  double integral_ = 0.0;
  double prev_measurement_ = 0.0;
};

}  // namespace swarm_control
