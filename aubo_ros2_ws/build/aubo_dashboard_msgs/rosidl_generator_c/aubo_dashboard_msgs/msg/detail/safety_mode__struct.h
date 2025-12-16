// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from aubo_dashboard_msgs:msg/SafetyMode.idl
// generated code does not contain a copyright notice

#ifndef AUBO_DASHBOARD_MSGS__MSG__DETAIL__SAFETY_MODE__STRUCT_H_
#define AUBO_DASHBOARD_MSGS__MSG__DETAIL__SAFETY_MODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'NORMAL'.
enum
{
  aubo_dashboard_msgs__msg__SafetyMode__NORMAL = 1
};

/// Constant 'REDUCED'.
enum
{
  aubo_dashboard_msgs__msg__SafetyMode__REDUCED = 2
};

/// Constant 'PROTECTIVE_STOP'.
enum
{
  aubo_dashboard_msgs__msg__SafetyMode__PROTECTIVE_STOP = 3
};

/// Constant 'RECOVERY'.
enum
{
  aubo_dashboard_msgs__msg__SafetyMode__RECOVERY = 4
};

/// Constant 'SAFEGUARD_STOP'.
enum
{
  aubo_dashboard_msgs__msg__SafetyMode__SAFEGUARD_STOP = 5
};

/// Constant 'SYSTEM_EMERGENCY_STOP'.
enum
{
  aubo_dashboard_msgs__msg__SafetyMode__SYSTEM_EMERGENCY_STOP = 6
};

/// Constant 'ROBOT_EMERGENCY_STOP'.
enum
{
  aubo_dashboard_msgs__msg__SafetyMode__ROBOT_EMERGENCY_STOP = 7
};

/// Constant 'VIOLATION'.
enum
{
  aubo_dashboard_msgs__msg__SafetyMode__VIOLATION = 8
};

/// Constant 'FAULT'.
enum
{
  aubo_dashboard_msgs__msg__SafetyMode__FAULT = 9
};

/// Constant 'VALIDATE_JOINT_ID'.
enum
{
  aubo_dashboard_msgs__msg__SafetyMode__VALIDATE_JOINT_ID = 10
};

/// Constant 'UNDEFINED_SAFETY_MODE'.
enum
{
  aubo_dashboard_msgs__msg__SafetyMode__UNDEFINED_SAFETY_MODE = 11
};

/// Constant 'AUTOMATIC_MODE_SAFEGUARD_STOP'.
enum
{
  aubo_dashboard_msgs__msg__SafetyMode__AUTOMATIC_MODE_SAFEGUARD_STOP = 12
};

/// Constant 'SYSTEM_THREE_POSITION_ENABLING_STOP'.
enum
{
  aubo_dashboard_msgs__msg__SafetyMode__SYSTEM_THREE_POSITION_ENABLING_STOP = 13
};

// Struct defined in msg/SafetyMode in the package aubo_dashboard_msgs.
typedef struct aubo_dashboard_msgs__msg__SafetyMode
{
  uint8_t mode;
} aubo_dashboard_msgs__msg__SafetyMode;

// Struct for a sequence of aubo_dashboard_msgs__msg__SafetyMode.
typedef struct aubo_dashboard_msgs__msg__SafetyMode__Sequence
{
  aubo_dashboard_msgs__msg__SafetyMode * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aubo_dashboard_msgs__msg__SafetyMode__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUBO_DASHBOARD_MSGS__MSG__DETAIL__SAFETY_MODE__STRUCT_H_
