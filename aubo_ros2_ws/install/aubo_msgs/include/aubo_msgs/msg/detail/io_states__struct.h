// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from aubo_msgs:msg/IOStates.idl
// generated code does not contain a copyright notice

#ifndef AUBO_MSGS__MSG__DETAIL__IO_STATES__STRUCT_H_
#define AUBO_MSGS__MSG__DETAIL__IO_STATES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'digital_in_states'
// Member 'digital_out_states'
// Member 'flag_states'
#include "aubo_msgs/msg/detail/digital__struct.h"
// Member 'analog_in_states'
// Member 'analog_out_states'
#include "aubo_msgs/msg/detail/analog__struct.h"

// Struct defined in msg/IOStates in the package aubo_msgs.
typedef struct aubo_msgs__msg__IOStates
{
  aubo_msgs__msg__Digital__Sequence digital_in_states;
  aubo_msgs__msg__Digital__Sequence digital_out_states;
  aubo_msgs__msg__Digital__Sequence flag_states;
  aubo_msgs__msg__Analog__Sequence analog_in_states;
  aubo_msgs__msg__Analog__Sequence analog_out_states;
} aubo_msgs__msg__IOStates;

// Struct for a sequence of aubo_msgs__msg__IOStates.
typedef struct aubo_msgs__msg__IOStates__Sequence
{
  aubo_msgs__msg__IOStates * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aubo_msgs__msg__IOStates__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUBO_MSGS__MSG__DETAIL__IO_STATES__STRUCT_H_
