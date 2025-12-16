// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from aubo_dashboard_msgs:srv/GetRobotMode.idl
// generated code does not contain a copyright notice

#ifndef AUBO_DASHBOARD_MSGS__SRV__DETAIL__GET_ROBOT_MODE__STRUCT_H_
#define AUBO_DASHBOARD_MSGS__SRV__DETAIL__GET_ROBOT_MODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/GetRobotMode in the package aubo_dashboard_msgs.
typedef struct aubo_dashboard_msgs__srv__GetRobotMode_Request
{
  uint8_t structure_needs_at_least_one_member;
} aubo_dashboard_msgs__srv__GetRobotMode_Request;

// Struct for a sequence of aubo_dashboard_msgs__srv__GetRobotMode_Request.
typedef struct aubo_dashboard_msgs__srv__GetRobotMode_Request__Sequence
{
  aubo_dashboard_msgs__srv__GetRobotMode_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aubo_dashboard_msgs__srv__GetRobotMode_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'robot_mode'
#include "aubo_dashboard_msgs/msg/detail/robot_mode__struct.h"
// Member 'answer'
#include "rosidl_runtime_c/string.h"

// Struct defined in srv/GetRobotMode in the package aubo_dashboard_msgs.
typedef struct aubo_dashboard_msgs__srv__GetRobotMode_Response
{
  aubo_dashboard_msgs__msg__RobotMode robot_mode;
  rosidl_runtime_c__String answer;
  bool success;
} aubo_dashboard_msgs__srv__GetRobotMode_Response;

// Struct for a sequence of aubo_dashboard_msgs__srv__GetRobotMode_Response.
typedef struct aubo_dashboard_msgs__srv__GetRobotMode_Response__Sequence
{
  aubo_dashboard_msgs__srv__GetRobotMode_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aubo_dashboard_msgs__srv__GetRobotMode_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUBO_DASHBOARD_MSGS__SRV__DETAIL__GET_ROBOT_MODE__STRUCT_H_
