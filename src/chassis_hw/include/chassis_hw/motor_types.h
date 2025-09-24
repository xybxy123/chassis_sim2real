// motor_types.h
#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

namespace chassis_hw
{
struct MotorData
{
  double angle;
  double velocity;
  double effort;
  double temp;
  double last_angle;
  double total_angle;
};

struct MotorCommand
{
  double effort;
};
} // namespace chassis_hw

#endif