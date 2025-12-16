// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from aubo_dashboard_msgs:msg/ProgramState.idl
// generated code does not contain a copyright notice
#include "aubo_dashboard_msgs/msg/detail/program_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `state`
#include "rosidl_runtime_c/string_functions.h"

bool
aubo_dashboard_msgs__msg__ProgramState__init(aubo_dashboard_msgs__msg__ProgramState * msg)
{
  if (!msg) {
    return false;
  }
  // state
  if (!rosidl_runtime_c__String__init(&msg->state)) {
    aubo_dashboard_msgs__msg__ProgramState__fini(msg);
    return false;
  }
  return true;
}

void
aubo_dashboard_msgs__msg__ProgramState__fini(aubo_dashboard_msgs__msg__ProgramState * msg)
{
  if (!msg) {
    return;
  }
  // state
  rosidl_runtime_c__String__fini(&msg->state);
}

bool
aubo_dashboard_msgs__msg__ProgramState__are_equal(const aubo_dashboard_msgs__msg__ProgramState * lhs, const aubo_dashboard_msgs__msg__ProgramState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->state), &(rhs->state)))
  {
    return false;
  }
  return true;
}

bool
aubo_dashboard_msgs__msg__ProgramState__copy(
  const aubo_dashboard_msgs__msg__ProgramState * input,
  aubo_dashboard_msgs__msg__ProgramState * output)
{
  if (!input || !output) {
    return false;
  }
  // state
  if (!rosidl_runtime_c__String__copy(
      &(input->state), &(output->state)))
  {
    return false;
  }
  return true;
}

aubo_dashboard_msgs__msg__ProgramState *
aubo_dashboard_msgs__msg__ProgramState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aubo_dashboard_msgs__msg__ProgramState * msg = (aubo_dashboard_msgs__msg__ProgramState *)allocator.allocate(sizeof(aubo_dashboard_msgs__msg__ProgramState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(aubo_dashboard_msgs__msg__ProgramState));
  bool success = aubo_dashboard_msgs__msg__ProgramState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
aubo_dashboard_msgs__msg__ProgramState__destroy(aubo_dashboard_msgs__msg__ProgramState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    aubo_dashboard_msgs__msg__ProgramState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
aubo_dashboard_msgs__msg__ProgramState__Sequence__init(aubo_dashboard_msgs__msg__ProgramState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aubo_dashboard_msgs__msg__ProgramState * data = NULL;

  if (size) {
    data = (aubo_dashboard_msgs__msg__ProgramState *)allocator.zero_allocate(size, sizeof(aubo_dashboard_msgs__msg__ProgramState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = aubo_dashboard_msgs__msg__ProgramState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        aubo_dashboard_msgs__msg__ProgramState__fini(&data[i - 1]);
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
aubo_dashboard_msgs__msg__ProgramState__Sequence__fini(aubo_dashboard_msgs__msg__ProgramState__Sequence * array)
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
      aubo_dashboard_msgs__msg__ProgramState__fini(&array->data[i]);
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

aubo_dashboard_msgs__msg__ProgramState__Sequence *
aubo_dashboard_msgs__msg__ProgramState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aubo_dashboard_msgs__msg__ProgramState__Sequence * array = (aubo_dashboard_msgs__msg__ProgramState__Sequence *)allocator.allocate(sizeof(aubo_dashboard_msgs__msg__ProgramState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = aubo_dashboard_msgs__msg__ProgramState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
aubo_dashboard_msgs__msg__ProgramState__Sequence__destroy(aubo_dashboard_msgs__msg__ProgramState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    aubo_dashboard_msgs__msg__ProgramState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
aubo_dashboard_msgs__msg__ProgramState__Sequence__are_equal(const aubo_dashboard_msgs__msg__ProgramState__Sequence * lhs, const aubo_dashboard_msgs__msg__ProgramState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!aubo_dashboard_msgs__msg__ProgramState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
aubo_dashboard_msgs__msg__ProgramState__Sequence__copy(
  const aubo_dashboard_msgs__msg__ProgramState__Sequence * input,
  aubo_dashboard_msgs__msg__ProgramState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(aubo_dashboard_msgs__msg__ProgramState);
    aubo_dashboard_msgs__msg__ProgramState * data =
      (aubo_dashboard_msgs__msg__ProgramState *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!aubo_dashboard_msgs__msg__ProgramState__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          aubo_dashboard_msgs__msg__ProgramState__fini(&data[i]);
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
    if (!aubo_dashboard_msgs__msg__ProgramState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
