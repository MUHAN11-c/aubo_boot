// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from aubo_dashboard_msgs:action/SetMode.idl
// generated code does not contain a copyright notice

#ifndef AUBO_DASHBOARD_MSGS__ACTION__DETAIL__SET_MODE__STRUCT_H_
#define AUBO_DASHBOARD_MSGS__ACTION__DETAIL__SET_MODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in action/SetMode in the package aubo_dashboard_msgs.
typedef struct aubo_dashboard_msgs__action__SetMode_Goal
{
  int8_t target_robot_mode;
  bool stop_program;
  bool play_program;
} aubo_dashboard_msgs__action__SetMode_Goal;

// Struct for a sequence of aubo_dashboard_msgs__action__SetMode_Goal.
typedef struct aubo_dashboard_msgs__action__SetMode_Goal__Sequence
{
  aubo_dashboard_msgs__action__SetMode_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aubo_dashboard_msgs__action__SetMode_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

// Struct defined in action/SetMode in the package aubo_dashboard_msgs.
typedef struct aubo_dashboard_msgs__action__SetMode_Result
{
  bool success;
  rosidl_runtime_c__String message;
} aubo_dashboard_msgs__action__SetMode_Result;

// Struct for a sequence of aubo_dashboard_msgs__action__SetMode_Result.
typedef struct aubo_dashboard_msgs__action__SetMode_Result__Sequence
{
  aubo_dashboard_msgs__action__SetMode_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aubo_dashboard_msgs__action__SetMode_Result__Sequence;


// Constants defined in the message

// Struct defined in action/SetMode in the package aubo_dashboard_msgs.
typedef struct aubo_dashboard_msgs__action__SetMode_Feedback
{
  int8_t current_robot_mode;
  int8_t current_safety_mode;
} aubo_dashboard_msgs__action__SetMode_Feedback;

// Struct for a sequence of aubo_dashboard_msgs__action__SetMode_Feedback.
typedef struct aubo_dashboard_msgs__action__SetMode_Feedback__Sequence
{
  aubo_dashboard_msgs__action__SetMode_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aubo_dashboard_msgs__action__SetMode_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "aubo_dashboard_msgs/action/detail/set_mode__struct.h"

// Struct defined in action/SetMode in the package aubo_dashboard_msgs.
typedef struct aubo_dashboard_msgs__action__SetMode_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  aubo_dashboard_msgs__action__SetMode_Goal goal;
} aubo_dashboard_msgs__action__SetMode_SendGoal_Request;

// Struct for a sequence of aubo_dashboard_msgs__action__SetMode_SendGoal_Request.
typedef struct aubo_dashboard_msgs__action__SetMode_SendGoal_Request__Sequence
{
  aubo_dashboard_msgs__action__SetMode_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aubo_dashboard_msgs__action__SetMode_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

// Struct defined in action/SetMode in the package aubo_dashboard_msgs.
typedef struct aubo_dashboard_msgs__action__SetMode_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} aubo_dashboard_msgs__action__SetMode_SendGoal_Response;

// Struct for a sequence of aubo_dashboard_msgs__action__SetMode_SendGoal_Response.
typedef struct aubo_dashboard_msgs__action__SetMode_SendGoal_Response__Sequence
{
  aubo_dashboard_msgs__action__SetMode_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aubo_dashboard_msgs__action__SetMode_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

// Struct defined in action/SetMode in the package aubo_dashboard_msgs.
typedef struct aubo_dashboard_msgs__action__SetMode_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} aubo_dashboard_msgs__action__SetMode_GetResult_Request;

// Struct for a sequence of aubo_dashboard_msgs__action__SetMode_GetResult_Request.
typedef struct aubo_dashboard_msgs__action__SetMode_GetResult_Request__Sequence
{
  aubo_dashboard_msgs__action__SetMode_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aubo_dashboard_msgs__action__SetMode_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "aubo_dashboard_msgs/action/detail/set_mode__struct.h"

// Struct defined in action/SetMode in the package aubo_dashboard_msgs.
typedef struct aubo_dashboard_msgs__action__SetMode_GetResult_Response
{
  int8_t status;
  aubo_dashboard_msgs__action__SetMode_Result result;
} aubo_dashboard_msgs__action__SetMode_GetResult_Response;

// Struct for a sequence of aubo_dashboard_msgs__action__SetMode_GetResult_Response.
typedef struct aubo_dashboard_msgs__action__SetMode_GetResult_Response__Sequence
{
  aubo_dashboard_msgs__action__SetMode_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aubo_dashboard_msgs__action__SetMode_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "aubo_dashboard_msgs/action/detail/set_mode__struct.h"

// Struct defined in action/SetMode in the package aubo_dashboard_msgs.
typedef struct aubo_dashboard_msgs__action__SetMode_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  aubo_dashboard_msgs__action__SetMode_Feedback feedback;
} aubo_dashboard_msgs__action__SetMode_FeedbackMessage;

// Struct for a sequence of aubo_dashboard_msgs__action__SetMode_FeedbackMessage.
typedef struct aubo_dashboard_msgs__action__SetMode_FeedbackMessage__Sequence
{
  aubo_dashboard_msgs__action__SetMode_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aubo_dashboard_msgs__action__SetMode_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUBO_DASHBOARD_MSGS__ACTION__DETAIL__SET_MODE__STRUCT_H_
