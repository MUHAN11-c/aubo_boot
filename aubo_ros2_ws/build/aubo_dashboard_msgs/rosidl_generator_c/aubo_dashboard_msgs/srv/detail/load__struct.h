// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from aubo_dashboard_msgs:srv/Load.idl
// generated code does not contain a copyright notice

#ifndef AUBO_DASHBOARD_MSGS__SRV__DETAIL__LOAD__STRUCT_H_
#define AUBO_DASHBOARD_MSGS__SRV__DETAIL__LOAD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'filename'
#include "rosidl_runtime_c/string.h"

// Struct defined in srv/Load in the package aubo_dashboard_msgs.
typedef struct aubo_dashboard_msgs__srv__Load_Request
{
  rosidl_runtime_c__String filename;
} aubo_dashboard_msgs__srv__Load_Request;

// Struct for a sequence of aubo_dashboard_msgs__srv__Load_Request.
typedef struct aubo_dashboard_msgs__srv__Load_Request__Sequence
{
  aubo_dashboard_msgs__srv__Load_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aubo_dashboard_msgs__srv__Load_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'answer'
// already included above
// #include "rosidl_runtime_c/string.h"

// Struct defined in srv/Load in the package aubo_dashboard_msgs.
typedef struct aubo_dashboard_msgs__srv__Load_Response
{
  rosidl_runtime_c__String answer;
  bool success;
} aubo_dashboard_msgs__srv__Load_Response;

// Struct for a sequence of aubo_dashboard_msgs__srv__Load_Response.
typedef struct aubo_dashboard_msgs__srv__Load_Response__Sequence
{
  aubo_dashboard_msgs__srv__Load_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aubo_dashboard_msgs__srv__Load_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUBO_DASHBOARD_MSGS__SRV__DETAIL__LOAD__STRUCT_H_
