// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from aubo_dashboard_msgs:srv/IsInRemoteControl.idl
// generated code does not contain a copyright notice

#ifndef AUBO_DASHBOARD_MSGS__SRV__DETAIL__IS_IN_REMOTE_CONTROL__FUNCTIONS_H_
#define AUBO_DASHBOARD_MSGS__SRV__DETAIL__IS_IN_REMOTE_CONTROL__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "aubo_dashboard_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "aubo_dashboard_msgs/srv/detail/is_in_remote_control__struct.h"

/// Initialize srv/IsInRemoteControl message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * aubo_dashboard_msgs__srv__IsInRemoteControl_Request
 * )) before or use
 * aubo_dashboard_msgs__srv__IsInRemoteControl_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_aubo_dashboard_msgs
bool
aubo_dashboard_msgs__srv__IsInRemoteControl_Request__init(aubo_dashboard_msgs__srv__IsInRemoteControl_Request * msg);

/// Finalize srv/IsInRemoteControl message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_aubo_dashboard_msgs
void
aubo_dashboard_msgs__srv__IsInRemoteControl_Request__fini(aubo_dashboard_msgs__srv__IsInRemoteControl_Request * msg);

/// Create srv/IsInRemoteControl message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * aubo_dashboard_msgs__srv__IsInRemoteControl_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_aubo_dashboard_msgs
aubo_dashboard_msgs__srv__IsInRemoteControl_Request *
aubo_dashboard_msgs__srv__IsInRemoteControl_Request__create();

/// Destroy srv/IsInRemoteControl message.
/**
 * It calls
 * aubo_dashboard_msgs__srv__IsInRemoteControl_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_aubo_dashboard_msgs
void
aubo_dashboard_msgs__srv__IsInRemoteControl_Request__destroy(aubo_dashboard_msgs__srv__IsInRemoteControl_Request * msg);

/// Check for srv/IsInRemoteControl message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_aubo_dashboard_msgs
bool
aubo_dashboard_msgs__srv__IsInRemoteControl_Request__are_equal(const aubo_dashboard_msgs__srv__IsInRemoteControl_Request * lhs, const aubo_dashboard_msgs__srv__IsInRemoteControl_Request * rhs);

/// Copy a srv/IsInRemoteControl message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_aubo_dashboard_msgs
bool
aubo_dashboard_msgs__srv__IsInRemoteControl_Request__copy(
  const aubo_dashboard_msgs__srv__IsInRemoteControl_Request * input,
  aubo_dashboard_msgs__srv__IsInRemoteControl_Request * output);

/// Initialize array of srv/IsInRemoteControl messages.
/**
 * It allocates the memory for the number of elements and calls
 * aubo_dashboard_msgs__srv__IsInRemoteControl_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_aubo_dashboard_msgs
bool
aubo_dashboard_msgs__srv__IsInRemoteControl_Request__Sequence__init(aubo_dashboard_msgs__srv__IsInRemoteControl_Request__Sequence * array, size_t size);

/// Finalize array of srv/IsInRemoteControl messages.
/**
 * It calls
 * aubo_dashboard_msgs__srv__IsInRemoteControl_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_aubo_dashboard_msgs
void
aubo_dashboard_msgs__srv__IsInRemoteControl_Request__Sequence__fini(aubo_dashboard_msgs__srv__IsInRemoteControl_Request__Sequence * array);

/// Create array of srv/IsInRemoteControl messages.
/**
 * It allocates the memory for the array and calls
 * aubo_dashboard_msgs__srv__IsInRemoteControl_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_aubo_dashboard_msgs
aubo_dashboard_msgs__srv__IsInRemoteControl_Request__Sequence *
aubo_dashboard_msgs__srv__IsInRemoteControl_Request__Sequence__create(size_t size);

/// Destroy array of srv/IsInRemoteControl messages.
/**
 * It calls
 * aubo_dashboard_msgs__srv__IsInRemoteControl_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_aubo_dashboard_msgs
void
aubo_dashboard_msgs__srv__IsInRemoteControl_Request__Sequence__destroy(aubo_dashboard_msgs__srv__IsInRemoteControl_Request__Sequence * array);

/// Check for srv/IsInRemoteControl message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_aubo_dashboard_msgs
bool
aubo_dashboard_msgs__srv__IsInRemoteControl_Request__Sequence__are_equal(const aubo_dashboard_msgs__srv__IsInRemoteControl_Request__Sequence * lhs, const aubo_dashboard_msgs__srv__IsInRemoteControl_Request__Sequence * rhs);

/// Copy an array of srv/IsInRemoteControl messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_aubo_dashboard_msgs
bool
aubo_dashboard_msgs__srv__IsInRemoteControl_Request__Sequence__copy(
  const aubo_dashboard_msgs__srv__IsInRemoteControl_Request__Sequence * input,
  aubo_dashboard_msgs__srv__IsInRemoteControl_Request__Sequence * output);

/// Initialize srv/IsInRemoteControl message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * aubo_dashboard_msgs__srv__IsInRemoteControl_Response
 * )) before or use
 * aubo_dashboard_msgs__srv__IsInRemoteControl_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_aubo_dashboard_msgs
bool
aubo_dashboard_msgs__srv__IsInRemoteControl_Response__init(aubo_dashboard_msgs__srv__IsInRemoteControl_Response * msg);

/// Finalize srv/IsInRemoteControl message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_aubo_dashboard_msgs
void
aubo_dashboard_msgs__srv__IsInRemoteControl_Response__fini(aubo_dashboard_msgs__srv__IsInRemoteControl_Response * msg);

/// Create srv/IsInRemoteControl message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * aubo_dashboard_msgs__srv__IsInRemoteControl_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_aubo_dashboard_msgs
aubo_dashboard_msgs__srv__IsInRemoteControl_Response *
aubo_dashboard_msgs__srv__IsInRemoteControl_Response__create();

/// Destroy srv/IsInRemoteControl message.
/**
 * It calls
 * aubo_dashboard_msgs__srv__IsInRemoteControl_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_aubo_dashboard_msgs
void
aubo_dashboard_msgs__srv__IsInRemoteControl_Response__destroy(aubo_dashboard_msgs__srv__IsInRemoteControl_Response * msg);

/// Check for srv/IsInRemoteControl message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_aubo_dashboard_msgs
bool
aubo_dashboard_msgs__srv__IsInRemoteControl_Response__are_equal(const aubo_dashboard_msgs__srv__IsInRemoteControl_Response * lhs, const aubo_dashboard_msgs__srv__IsInRemoteControl_Response * rhs);

/// Copy a srv/IsInRemoteControl message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_aubo_dashboard_msgs
bool
aubo_dashboard_msgs__srv__IsInRemoteControl_Response__copy(
  const aubo_dashboard_msgs__srv__IsInRemoteControl_Response * input,
  aubo_dashboard_msgs__srv__IsInRemoteControl_Response * output);

/// Initialize array of srv/IsInRemoteControl messages.
/**
 * It allocates the memory for the number of elements and calls
 * aubo_dashboard_msgs__srv__IsInRemoteControl_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_aubo_dashboard_msgs
bool
aubo_dashboard_msgs__srv__IsInRemoteControl_Response__Sequence__init(aubo_dashboard_msgs__srv__IsInRemoteControl_Response__Sequence * array, size_t size);

/// Finalize array of srv/IsInRemoteControl messages.
/**
 * It calls
 * aubo_dashboard_msgs__srv__IsInRemoteControl_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_aubo_dashboard_msgs
void
aubo_dashboard_msgs__srv__IsInRemoteControl_Response__Sequence__fini(aubo_dashboard_msgs__srv__IsInRemoteControl_Response__Sequence * array);

/// Create array of srv/IsInRemoteControl messages.
/**
 * It allocates the memory for the array and calls
 * aubo_dashboard_msgs__srv__IsInRemoteControl_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_aubo_dashboard_msgs
aubo_dashboard_msgs__srv__IsInRemoteControl_Response__Sequence *
aubo_dashboard_msgs__srv__IsInRemoteControl_Response__Sequence__create(size_t size);

/// Destroy array of srv/IsInRemoteControl messages.
/**
 * It calls
 * aubo_dashboard_msgs__srv__IsInRemoteControl_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_aubo_dashboard_msgs
void
aubo_dashboard_msgs__srv__IsInRemoteControl_Response__Sequence__destroy(aubo_dashboard_msgs__srv__IsInRemoteControl_Response__Sequence * array);

/// Check for srv/IsInRemoteControl message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_aubo_dashboard_msgs
bool
aubo_dashboard_msgs__srv__IsInRemoteControl_Response__Sequence__are_equal(const aubo_dashboard_msgs__srv__IsInRemoteControl_Response__Sequence * lhs, const aubo_dashboard_msgs__srv__IsInRemoteControl_Response__Sequence * rhs);

/// Copy an array of srv/IsInRemoteControl messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_aubo_dashboard_msgs
bool
aubo_dashboard_msgs__srv__IsInRemoteControl_Response__Sequence__copy(
  const aubo_dashboard_msgs__srv__IsInRemoteControl_Response__Sequence * input,
  aubo_dashboard_msgs__srv__IsInRemoteControl_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // AUBO_DASHBOARD_MSGS__SRV__DETAIL__IS_IN_REMOTE_CONTROL__FUNCTIONS_H_
