// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from aubo_dashboard_msgs:msg/SafetyMode.idl
// generated code does not contain a copyright notice
#include "aubo_dashboard_msgs/msg/detail/safety_mode__rosidl_typesupport_fastrtps_cpp.hpp"
#include "aubo_dashboard_msgs/msg/detail/safety_mode__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace aubo_dashboard_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_aubo_dashboard_msgs
cdr_serialize(
  const aubo_dashboard_msgs::msg::SafetyMode & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: mode
  cdr << ros_message.mode;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_aubo_dashboard_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  aubo_dashboard_msgs::msg::SafetyMode & ros_message)
{
  // Member: mode
  cdr >> ros_message.mode;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_aubo_dashboard_msgs
get_serialized_size(
  const aubo_dashboard_msgs::msg::SafetyMode & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: mode
  {
    size_t item_size = sizeof(ros_message.mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_aubo_dashboard_msgs
max_serialized_size_SafetyMode(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: mode
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static bool _SafetyMode__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const aubo_dashboard_msgs::msg::SafetyMode *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _SafetyMode__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<aubo_dashboard_msgs::msg::SafetyMode *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _SafetyMode__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const aubo_dashboard_msgs::msg::SafetyMode *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _SafetyMode__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_SafetyMode(full_bounded, 0);
}

static message_type_support_callbacks_t _SafetyMode__callbacks = {
  "aubo_dashboard_msgs::msg",
  "SafetyMode",
  _SafetyMode__cdr_serialize,
  _SafetyMode__cdr_deserialize,
  _SafetyMode__get_serialized_size,
  _SafetyMode__max_serialized_size
};

static rosidl_message_type_support_t _SafetyMode__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SafetyMode__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace aubo_dashboard_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_aubo_dashboard_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<aubo_dashboard_msgs::msg::SafetyMode>()
{
  return &aubo_dashboard_msgs::msg::typesupport_fastrtps_cpp::_SafetyMode__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, aubo_dashboard_msgs, msg, SafetyMode)() {
  return &aubo_dashboard_msgs::msg::typesupport_fastrtps_cpp::_SafetyMode__handle;
}

#ifdef __cplusplus
}
#endif
