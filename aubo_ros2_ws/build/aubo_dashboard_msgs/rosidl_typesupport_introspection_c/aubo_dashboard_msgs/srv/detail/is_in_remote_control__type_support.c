// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from aubo_dashboard_msgs:srv/IsInRemoteControl.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "aubo_dashboard_msgs/srv/detail/is_in_remote_control__rosidl_typesupport_introspection_c.h"
#include "aubo_dashboard_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "aubo_dashboard_msgs/srv/detail/is_in_remote_control__functions.h"
#include "aubo_dashboard_msgs/srv/detail/is_in_remote_control__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void IsInRemoteControl_Request__rosidl_typesupport_introspection_c__IsInRemoteControl_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  aubo_dashboard_msgs__srv__IsInRemoteControl_Request__init(message_memory);
}

void IsInRemoteControl_Request__rosidl_typesupport_introspection_c__IsInRemoteControl_Request_fini_function(void * message_memory)
{
  aubo_dashboard_msgs__srv__IsInRemoteControl_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember IsInRemoteControl_Request__rosidl_typesupport_introspection_c__IsInRemoteControl_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aubo_dashboard_msgs__srv__IsInRemoteControl_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers IsInRemoteControl_Request__rosidl_typesupport_introspection_c__IsInRemoteControl_Request_message_members = {
  "aubo_dashboard_msgs__srv",  // message namespace
  "IsInRemoteControl_Request",  // message name
  1,  // number of fields
  sizeof(aubo_dashboard_msgs__srv__IsInRemoteControl_Request),
  IsInRemoteControl_Request__rosidl_typesupport_introspection_c__IsInRemoteControl_Request_message_member_array,  // message members
  IsInRemoteControl_Request__rosidl_typesupport_introspection_c__IsInRemoteControl_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  IsInRemoteControl_Request__rosidl_typesupport_introspection_c__IsInRemoteControl_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t IsInRemoteControl_Request__rosidl_typesupport_introspection_c__IsInRemoteControl_Request_message_type_support_handle = {
  0,
  &IsInRemoteControl_Request__rosidl_typesupport_introspection_c__IsInRemoteControl_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_aubo_dashboard_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aubo_dashboard_msgs, srv, IsInRemoteControl_Request)() {
  if (!IsInRemoteControl_Request__rosidl_typesupport_introspection_c__IsInRemoteControl_Request_message_type_support_handle.typesupport_identifier) {
    IsInRemoteControl_Request__rosidl_typesupport_introspection_c__IsInRemoteControl_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &IsInRemoteControl_Request__rosidl_typesupport_introspection_c__IsInRemoteControl_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "aubo_dashboard_msgs/srv/detail/is_in_remote_control__rosidl_typesupport_introspection_c.h"
// already included above
// #include "aubo_dashboard_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "aubo_dashboard_msgs/srv/detail/is_in_remote_control__functions.h"
// already included above
// #include "aubo_dashboard_msgs/srv/detail/is_in_remote_control__struct.h"


// Include directives for member types
// Member `answer`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void IsInRemoteControl_Response__rosidl_typesupport_introspection_c__IsInRemoteControl_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  aubo_dashboard_msgs__srv__IsInRemoteControl_Response__init(message_memory);
}

void IsInRemoteControl_Response__rosidl_typesupport_introspection_c__IsInRemoteControl_Response_fini_function(void * message_memory)
{
  aubo_dashboard_msgs__srv__IsInRemoteControl_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember IsInRemoteControl_Response__rosidl_typesupport_introspection_c__IsInRemoteControl_Response_message_member_array[3] = {
  {
    "answer",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aubo_dashboard_msgs__srv__IsInRemoteControl_Response, answer),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "in_remote_control",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aubo_dashboard_msgs__srv__IsInRemoteControl_Response, in_remote_control),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aubo_dashboard_msgs__srv__IsInRemoteControl_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers IsInRemoteControl_Response__rosidl_typesupport_introspection_c__IsInRemoteControl_Response_message_members = {
  "aubo_dashboard_msgs__srv",  // message namespace
  "IsInRemoteControl_Response",  // message name
  3,  // number of fields
  sizeof(aubo_dashboard_msgs__srv__IsInRemoteControl_Response),
  IsInRemoteControl_Response__rosidl_typesupport_introspection_c__IsInRemoteControl_Response_message_member_array,  // message members
  IsInRemoteControl_Response__rosidl_typesupport_introspection_c__IsInRemoteControl_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  IsInRemoteControl_Response__rosidl_typesupport_introspection_c__IsInRemoteControl_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t IsInRemoteControl_Response__rosidl_typesupport_introspection_c__IsInRemoteControl_Response_message_type_support_handle = {
  0,
  &IsInRemoteControl_Response__rosidl_typesupport_introspection_c__IsInRemoteControl_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_aubo_dashboard_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aubo_dashboard_msgs, srv, IsInRemoteControl_Response)() {
  if (!IsInRemoteControl_Response__rosidl_typesupport_introspection_c__IsInRemoteControl_Response_message_type_support_handle.typesupport_identifier) {
    IsInRemoteControl_Response__rosidl_typesupport_introspection_c__IsInRemoteControl_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &IsInRemoteControl_Response__rosidl_typesupport_introspection_c__IsInRemoteControl_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "aubo_dashboard_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "aubo_dashboard_msgs/srv/detail/is_in_remote_control__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers aubo_dashboard_msgs__srv__detail__is_in_remote_control__rosidl_typesupport_introspection_c__IsInRemoteControl_service_members = {
  "aubo_dashboard_msgs__srv",  // service namespace
  "IsInRemoteControl",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // aubo_dashboard_msgs__srv__detail__is_in_remote_control__rosidl_typesupport_introspection_c__IsInRemoteControl_Request_message_type_support_handle,
  NULL  // response message
  // aubo_dashboard_msgs__srv__detail__is_in_remote_control__rosidl_typesupport_introspection_c__IsInRemoteControl_Response_message_type_support_handle
};

static rosidl_service_type_support_t aubo_dashboard_msgs__srv__detail__is_in_remote_control__rosidl_typesupport_introspection_c__IsInRemoteControl_service_type_support_handle = {
  0,
  &aubo_dashboard_msgs__srv__detail__is_in_remote_control__rosidl_typesupport_introspection_c__IsInRemoteControl_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aubo_dashboard_msgs, srv, IsInRemoteControl_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aubo_dashboard_msgs, srv, IsInRemoteControl_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_aubo_dashboard_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aubo_dashboard_msgs, srv, IsInRemoteControl)() {
  if (!aubo_dashboard_msgs__srv__detail__is_in_remote_control__rosidl_typesupport_introspection_c__IsInRemoteControl_service_type_support_handle.typesupport_identifier) {
    aubo_dashboard_msgs__srv__detail__is_in_remote_control__rosidl_typesupport_introspection_c__IsInRemoteControl_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)aubo_dashboard_msgs__srv__detail__is_in_remote_control__rosidl_typesupport_introspection_c__IsInRemoteControl_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aubo_dashboard_msgs, srv, IsInRemoteControl_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aubo_dashboard_msgs, srv, IsInRemoteControl_Response)()->data;
  }

  return &aubo_dashboard_msgs__srv__detail__is_in_remote_control__rosidl_typesupport_introspection_c__IsInRemoteControl_service_type_support_handle;
}
