// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from aubo_dashboard_msgs:srv/IsInRemoteControl.idl
// generated code does not contain a copyright notice
#include "aubo_dashboard_msgs/srv/detail/is_in_remote_control__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "aubo_dashboard_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "aubo_dashboard_msgs/srv/detail/is_in_remote_control__struct.h"
#include "aubo_dashboard_msgs/srv/detail/is_in_remote_control__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _IsInRemoteControl_Request__ros_msg_type = aubo_dashboard_msgs__srv__IsInRemoteControl_Request;

static bool _IsInRemoteControl_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _IsInRemoteControl_Request__ros_msg_type * ros_message = static_cast<const _IsInRemoteControl_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: structure_needs_at_least_one_member
  {
    cdr << ros_message->structure_needs_at_least_one_member;
  }

  return true;
}

static bool _IsInRemoteControl_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _IsInRemoteControl_Request__ros_msg_type * ros_message = static_cast<_IsInRemoteControl_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: structure_needs_at_least_one_member
  {
    cdr >> ros_message->structure_needs_at_least_one_member;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_aubo_dashboard_msgs
size_t get_serialized_size_aubo_dashboard_msgs__srv__IsInRemoteControl_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _IsInRemoteControl_Request__ros_msg_type * ros_message = static_cast<const _IsInRemoteControl_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name structure_needs_at_least_one_member
  {
    size_t item_size = sizeof(ros_message->structure_needs_at_least_one_member);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _IsInRemoteControl_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_aubo_dashboard_msgs__srv__IsInRemoteControl_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_aubo_dashboard_msgs
size_t max_serialized_size_aubo_dashboard_msgs__srv__IsInRemoteControl_Request(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: structure_needs_at_least_one_member
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _IsInRemoteControl_Request__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_aubo_dashboard_msgs__srv__IsInRemoteControl_Request(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_IsInRemoteControl_Request = {
  "aubo_dashboard_msgs::srv",
  "IsInRemoteControl_Request",
  _IsInRemoteControl_Request__cdr_serialize,
  _IsInRemoteControl_Request__cdr_deserialize,
  _IsInRemoteControl_Request__get_serialized_size,
  _IsInRemoteControl_Request__max_serialized_size
};

static rosidl_message_type_support_t _IsInRemoteControl_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_IsInRemoteControl_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, aubo_dashboard_msgs, srv, IsInRemoteControl_Request)() {
  return &_IsInRemoteControl_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "aubo_dashboard_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "aubo_dashboard_msgs/srv/detail/is_in_remote_control__struct.h"
// already included above
// #include "aubo_dashboard_msgs/srv/detail/is_in_remote_control__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // answer
#include "rosidl_runtime_c/string_functions.h"  // answer

// forward declare type support functions


using _IsInRemoteControl_Response__ros_msg_type = aubo_dashboard_msgs__srv__IsInRemoteControl_Response;

static bool _IsInRemoteControl_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _IsInRemoteControl_Response__ros_msg_type * ros_message = static_cast<const _IsInRemoteControl_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: answer
  {
    const rosidl_runtime_c__String * str = &ros_message->answer;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: in_remote_control
  {
    cdr << (ros_message->in_remote_control ? true : false);
  }

  // Field name: success
  {
    cdr << (ros_message->success ? true : false);
  }

  return true;
}

static bool _IsInRemoteControl_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _IsInRemoteControl_Response__ros_msg_type * ros_message = static_cast<_IsInRemoteControl_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: answer
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->answer.data) {
      rosidl_runtime_c__String__init(&ros_message->answer);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->answer,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'answer'\n");
      return false;
    }
  }

  // Field name: in_remote_control
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->in_remote_control = tmp ? true : false;
  }

  // Field name: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->success = tmp ? true : false;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_aubo_dashboard_msgs
size_t get_serialized_size_aubo_dashboard_msgs__srv__IsInRemoteControl_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _IsInRemoteControl_Response__ros_msg_type * ros_message = static_cast<const _IsInRemoteControl_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name answer
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->answer.size + 1);
  // field.name in_remote_control
  {
    size_t item_size = sizeof(ros_message->in_remote_control);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name success
  {
    size_t item_size = sizeof(ros_message->success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _IsInRemoteControl_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_aubo_dashboard_msgs__srv__IsInRemoteControl_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_aubo_dashboard_msgs
size_t max_serialized_size_aubo_dashboard_msgs__srv__IsInRemoteControl_Response(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: answer
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: in_remote_control
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: success
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _IsInRemoteControl_Response__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_aubo_dashboard_msgs__srv__IsInRemoteControl_Response(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_IsInRemoteControl_Response = {
  "aubo_dashboard_msgs::srv",
  "IsInRemoteControl_Response",
  _IsInRemoteControl_Response__cdr_serialize,
  _IsInRemoteControl_Response__cdr_deserialize,
  _IsInRemoteControl_Response__get_serialized_size,
  _IsInRemoteControl_Response__max_serialized_size
};

static rosidl_message_type_support_t _IsInRemoteControl_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_IsInRemoteControl_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, aubo_dashboard_msgs, srv, IsInRemoteControl_Response)() {
  return &_IsInRemoteControl_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "aubo_dashboard_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "aubo_dashboard_msgs/srv/is_in_remote_control.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t IsInRemoteControl__callbacks = {
  "aubo_dashboard_msgs::srv",
  "IsInRemoteControl",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, aubo_dashboard_msgs, srv, IsInRemoteControl_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, aubo_dashboard_msgs, srv, IsInRemoteControl_Response)(),
};

static rosidl_service_type_support_t IsInRemoteControl__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &IsInRemoteControl__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, aubo_dashboard_msgs, srv, IsInRemoteControl)() {
  return &IsInRemoteControl__handle;
}

#if defined(__cplusplus)
}
#endif
