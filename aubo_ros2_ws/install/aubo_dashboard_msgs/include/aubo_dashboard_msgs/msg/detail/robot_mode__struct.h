// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from aubo_dashboard_msgs:msg/RobotMode.idl
// generated code does not contain a copyright notice

#ifndef AUBO_DASHBOARD_MSGS__MSG__DETAIL__ROBOT_MODE__STRUCT_H_
#define AUBO_DASHBOARD_MSGS__MSG__DETAIL__ROBOT_MODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'NO_CONTROLLER'.
enum
{
  aubo_dashboard_msgs__msg__RobotMode__NO_CONTROLLER = -1
};

/// Constant 'DISCONNECTED'.
enum
{
  aubo_dashboard_msgs__msg__RobotMode__DISCONNECTED = 0
};

/// Constant 'CONFIRM_SAFETY'.
enum
{
  aubo_dashboard_msgs__msg__RobotMode__CONFIRM_SAFETY = 1
};

/// Constant 'BOOTING'.
enum
{
  aubo_dashboard_msgs__msg__RobotMode__BOOTING = 2
};

/// Constant 'POWER_OFF'.
enum
{
  aubo_dashboard_msgs__msg__RobotMode__POWER_OFF = 3
};

/// Constant 'POWER_ON'.
enum
{
  aubo_dashboard_msgs__msg__RobotMode__POWER_ON = 4
};

/// Constant 'IDLE'.
enum
{
  aubo_dashboard_msgs__msg__RobotMode__IDLE = 5
};

/// Constant 'BACKDRIVE'.
enum
{
  aubo_dashboard_msgs__msg__RobotMode__BACKDRIVE = 6
};

/// Constant 'RUNNING'.
enum
{
  aubo_dashboard_msgs__msg__RobotMode__RUNNING = 7
};

/// Constant 'UPDATING_FIRMWARE'.
enum
{
  aubo_dashboard_msgs__msg__RobotMode__UPDATING_FIRMWARE = 8
};

// Struct defined in msg/RobotMode in the package aubo_dashboard_msgs.
typedef struct aubo_dashboard_msgs__msg__RobotMode
{
  int8_t mode;
} aubo_dashboard_msgs__msg__RobotMode;

// Struct for a sequence of aubo_dashboard_msgs__msg__RobotMode.
typedef struct aubo_dashboard_msgs__msg__RobotMode__Sequence
{
  aubo_dashboard_msgs__msg__RobotMode * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aubo_dashboard_msgs__msg__RobotMode__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUBO_DASHBOARD_MSGS__MSG__DETAIL__ROBOT_MODE__STRUCT_H_
