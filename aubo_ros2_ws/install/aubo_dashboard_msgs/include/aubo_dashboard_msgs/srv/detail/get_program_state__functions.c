// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from aubo_dashboard_msgs:srv/GetProgramState.idl
// generated code does not contain a copyright notice
#include "aubo_dashboard_msgs/srv/detail/get_program_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
aubo_dashboard_msgs__srv__GetProgramState_Request__init(aubo_dashboard_msgs__srv__GetProgramState_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
aubo_dashboard_msgs__srv__GetProgramState_Request__fini(aubo_dashboard_msgs__srv__GetProgramState_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
aubo_dashboard_msgs__srv__GetProgramState_Request__are_equal(const aubo_dashboard_msgs__srv__GetProgramState_Request * lhs, const aubo_dashboard_msgs__srv__GetProgramState_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
aubo_dashboard_msgs__srv__GetProgramState_Request__copy(
  const aubo_dashboard_msgs__srv__GetProgramState_Request * input,
  aubo_dashboard_msgs__srv__GetProgramState_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

aubo_dashboard_msgs__srv__GetProgramState_Request *
aubo_dashboard_msgs__srv__GetProgramState_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aubo_dashboard_msgs__srv__GetProgramState_Request * msg = (aubo_dashboard_msgs__srv__GetProgramState_Request *)allocator.allocate(sizeof(aubo_dashboard_msgs__srv__GetProgramState_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(aubo_dashboard_msgs__srv__GetProgramState_Request));
  bool success = aubo_dashboard_msgs__srv__GetProgramState_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
aubo_dashboard_msgs__srv__GetProgramState_Request__destroy(aubo_dashboard_msgs__srv__GetProgramState_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    aubo_dashboard_msgs__srv__GetProgramState_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
aubo_dashboard_msgs__srv__GetProgramState_Request__Sequence__init(aubo_dashboard_msgs__srv__GetProgramState_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aubo_dashboard_msgs__srv__GetProgramState_Request * data = NULL;

  if (size) {
    data = (aubo_dashboard_msgs__srv__GetProgramState_Request *)allocator.zero_allocate(size, sizeof(aubo_dashboard_msgs__srv__GetProgramState_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = aubo_dashboard_msgs__srv__GetProgramState_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        aubo_dashboard_msgs__srv__GetProgramState_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
aubo_dashboard_msgs__srv__GetProgramState_Request__Sequence__fini(aubo_dashboard_msgs__srv__GetProgramState_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      aubo_dashboard_msgs__srv__GetProgramState_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

aubo_dashboard_msgs__srv__GetProgramState_Request__Sequence *
aubo_dashboard_msgs__srv__GetProgramState_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aubo_dashboard_msgs__srv__GetProgramState_Request__Sequence * array = (aubo_dashboard_msgs__srv__GetProgramState_Request__Sequence *)allocator.allocate(sizeof(aubo_dashboard_msgs__srv__GetProgramState_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = aubo_dashboard_msgs__srv__GetProgramState_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
aubo_dashboard_msgs__srv__GetProgramState_Request__Sequence__destroy(aubo_dashboard_msgs__srv__GetProgramState_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    aubo_dashboard_msgs__srv__GetProgramState_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
aubo_dashboard_msgs__srv__GetProgramState_Request__Sequence__are_equal(const aubo_dashboard_msgs__srv__GetProgramState_Request__Sequence * lhs, const aubo_dashboard_msgs__srv__GetProgramState_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!aubo_dashboard_msgs__srv__GetProgramState_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
aubo_dashboard_msgs__srv__GetProgramState_Request__Sequence__copy(
  const aubo_dashboard_msgs__srv__GetProgramState_Request__Sequence * input,
  aubo_dashboard_msgs__srv__GetProgramState_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(aubo_dashboard_msgs__srv__GetProgramState_Request);
    aubo_dashboard_msgs__srv__GetProgramState_Request * data =
      (aubo_dashboard_msgs__srv__GetProgramState_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!aubo_dashboard_msgs__srv__GetProgramState_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          aubo_dashboard_msgs__srv__GetProgramState_Request__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!aubo_dashboard_msgs__srv__GetProgramState_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `state`
#include "aubo_dashboard_msgs/msg/detail/program_state__functions.h"
// Member `program_name`
// Member `answer`
#include "rosidl_runtime_c/string_functions.h"

bool
aubo_dashboard_msgs__srv__GetProgramState_Response__init(aubo_dashboard_msgs__srv__GetProgramState_Response * msg)
{
  if (!msg) {
    return false;
  }
  // state
  if (!aubo_dashboard_msgs__msg__ProgramState__init(&msg->state)) {
    aubo_dashboard_msgs__srv__GetProgramState_Response__fini(msg);
    return false;
  }
  // program_name
  if (!rosidl_runtime_c__String__init(&msg->program_name)) {
    aubo_dashboard_msgs__srv__GetProgramState_Response__fini(msg);
    return false;
  }
  // answer
  if (!rosidl_runtime_c__String__init(&msg->answer)) {
    aubo_dashboard_msgs__srv__GetProgramState_Response__fini(msg);
    return false;
  }
  // success
  return true;
}

void
aubo_dashboard_msgs__srv__GetProgramState_Response__fini(aubo_dashboard_msgs__srv__GetProgramState_Response * msg)
{
  if (!msg) {
    return;
  }
  // state
  aubo_dashboard_msgs__msg__ProgramState__fini(&msg->state);
  // program_name
  rosidl_runtime_c__String__fini(&msg->program_name);
  // answer
  rosidl_runtime_c__String__fini(&msg->answer);
  // success
}

bool
aubo_dashboard_msgs__srv__GetProgramState_Response__are_equal(const aubo_dashboard_msgs__srv__GetProgramState_Response * lhs, const aubo_dashboard_msgs__srv__GetProgramState_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // state
  if (!aubo_dashboard_msgs__msg__ProgramState__are_equal(
      &(lhs->state), &(rhs->state)))
  {
    return false;
  }
  // program_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->program_name), &(rhs->program_name)))
  {
    return false;
  }
  // answer
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->answer), &(rhs->answer)))
  {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
aubo_dashboard_msgs__srv__GetProgramState_Response__copy(
  const aubo_dashboard_msgs__srv__GetProgramState_Response * input,
  aubo_dashboard_msgs__srv__GetProgramState_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // state
  if (!aubo_dashboard_msgs__msg__ProgramState__copy(
      &(input->state), &(output->state)))
  {
    return false;
  }
  // program_name
  if (!rosidl_runtime_c__String__copy(
      &(input->program_name), &(output->program_name)))
  {
    return false;
  }
  // answer
  if (!rosidl_runtime_c__String__copy(
      &(input->answer), &(output->answer)))
  {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

aubo_dashboard_msgs__srv__GetProgramState_Response *
aubo_dashboard_msgs__srv__GetProgramState_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aubo_dashboard_msgs__srv__GetProgramState_Response * msg = (aubo_dashboard_msgs__srv__GetProgramState_Response *)allocator.allocate(sizeof(aubo_dashboard_msgs__srv__GetProgramState_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(aubo_dashboard_msgs__srv__GetProgramState_Response));
  bool success = aubo_dashboard_msgs__srv__GetProgramState_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
aubo_dashboard_msgs__srv__GetProgramState_Response__destroy(aubo_dashboard_msgs__srv__GetProgramState_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    aubo_dashboard_msgs__srv__GetProgramState_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
aubo_dashboard_msgs__srv__GetProgramState_Response__Sequence__init(aubo_dashboard_msgs__srv__GetProgramState_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aubo_dashboard_msgs__srv__GetProgramState_Response * data = NULL;

  if (size) {
    data = (aubo_dashboard_msgs__srv__GetProgramState_Response *)allocator.zero_allocate(size, sizeof(aubo_dashboard_msgs__srv__GetProgramState_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = aubo_dashboard_msgs__srv__GetProgramState_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        aubo_dashboard_msgs__srv__GetProgramState_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
aubo_dashboard_msgs__srv__GetProgramState_Response__Sequence__fini(aubo_dashboard_msgs__srv__GetProgramState_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      aubo_dashboard_msgs__srv__GetProgramState_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

aubo_dashboard_msgs__srv__GetProgramState_Response__Sequence *
aubo_dashboard_msgs__srv__GetProgramState_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aubo_dashboard_msgs__srv__GetProgramState_Response__Sequence * array = (aubo_dashboard_msgs__srv__GetProgramState_Response__Sequence *)allocator.allocate(sizeof(aubo_dashboard_msgs__srv__GetProgramState_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = aubo_dashboard_msgs__srv__GetProgramState_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
aubo_dashboard_msgs__srv__GetProgramState_Response__Sequence__destroy(aubo_dashboard_msgs__srv__GetProgramState_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    aubo_dashboard_msgs__srv__GetProgramState_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
aubo_dashboard_msgs__srv__GetProgramState_Response__Sequence__are_equal(const aubo_dashboard_msgs__srv__GetProgramState_Response__Sequence * lhs, const aubo_dashboard_msgs__srv__GetProgramState_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!aubo_dashboard_msgs__srv__GetProgramState_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
aubo_dashboard_msgs__srv__GetProgramState_Response__Sequence__copy(
  const aubo_dashboard_msgs__srv__GetProgramState_Response__Sequence * input,
  aubo_dashboard_msgs__srv__GetProgramState_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(aubo_dashboard_msgs__srv__GetProgramState_Response);
    aubo_dashboard_msgs__srv__GetProgramState_Response * data =
      (aubo_dashboard_msgs__srv__GetProgramState_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!aubo_dashboard_msgs__srv__GetProgramState_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          aubo_dashboard_msgs__srv__GetProgramState_Response__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!aubo_dashboard_msgs__srv__GetProgramState_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
