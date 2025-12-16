// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from aubo_dashboard_msgs:msg/SafetyMode.idl
// generated code does not contain a copyright notice
#include "aubo_dashboard_msgs/msg/detail/safety_mode__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
aubo_dashboard_msgs__msg__SafetyMode__init(aubo_dashboard_msgs__msg__SafetyMode * msg)
{
  if (!msg) {
    return false;
  }
  // mode
  return true;
}

void
aubo_dashboard_msgs__msg__SafetyMode__fini(aubo_dashboard_msgs__msg__SafetyMode * msg)
{
  if (!msg) {
    return;
  }
  // mode
}

bool
aubo_dashboard_msgs__msg__SafetyMode__are_equal(const aubo_dashboard_msgs__msg__SafetyMode * lhs, const aubo_dashboard_msgs__msg__SafetyMode * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // mode
  if (lhs->mode != rhs->mode) {
    return false;
  }
  return true;
}

bool
aubo_dashboard_msgs__msg__SafetyMode__copy(
  const aubo_dashboard_msgs__msg__SafetyMode * input,
  aubo_dashboard_msgs__msg__SafetyMode * output)
{
  if (!input || !output) {
    return false;
  }
  // mode
  output->mode = input->mode;
  return true;
}

aubo_dashboard_msgs__msg__SafetyMode *
aubo_dashboard_msgs__msg__SafetyMode__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aubo_dashboard_msgs__msg__SafetyMode * msg = (aubo_dashboard_msgs__msg__SafetyMode *)allocator.allocate(sizeof(aubo_dashboard_msgs__msg__SafetyMode), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(aubo_dashboard_msgs__msg__SafetyMode));
  bool success = aubo_dashboard_msgs__msg__SafetyMode__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
aubo_dashboard_msgs__msg__SafetyMode__destroy(aubo_dashboard_msgs__msg__SafetyMode * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    aubo_dashboard_msgs__msg__SafetyMode__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
aubo_dashboard_msgs__msg__SafetyMode__Sequence__init(aubo_dashboard_msgs__msg__SafetyMode__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aubo_dashboard_msgs__msg__SafetyMode * data = NULL;

  if (size) {
    data = (aubo_dashboard_msgs__msg__SafetyMode *)allocator.zero_allocate(size, sizeof(aubo_dashboard_msgs__msg__SafetyMode), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = aubo_dashboard_msgs__msg__SafetyMode__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        aubo_dashboard_msgs__msg__SafetyMode__fini(&data[i - 1]);
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
aubo_dashboard_msgs__msg__SafetyMode__Sequence__fini(aubo_dashboard_msgs__msg__SafetyMode__Sequence * array)
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
      aubo_dashboard_msgs__msg__SafetyMode__fini(&array->data[i]);
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

aubo_dashboard_msgs__msg__SafetyMode__Sequence *
aubo_dashboard_msgs__msg__SafetyMode__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aubo_dashboard_msgs__msg__SafetyMode__Sequence * array = (aubo_dashboard_msgs__msg__SafetyMode__Sequence *)allocator.allocate(sizeof(aubo_dashboard_msgs__msg__SafetyMode__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = aubo_dashboard_msgs__msg__SafetyMode__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
aubo_dashboard_msgs__msg__SafetyMode__Sequence__destroy(aubo_dashboard_msgs__msg__SafetyMode__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    aubo_dashboard_msgs__msg__SafetyMode__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
aubo_dashboard_msgs__msg__SafetyMode__Sequence__are_equal(const aubo_dashboard_msgs__msg__SafetyMode__Sequence * lhs, const aubo_dashboard_msgs__msg__SafetyMode__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!aubo_dashboard_msgs__msg__SafetyMode__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
aubo_dashboard_msgs__msg__SafetyMode__Sequence__copy(
  const aubo_dashboard_msgs__msg__SafetyMode__Sequence * input,
  aubo_dashboard_msgs__msg__SafetyMode__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(aubo_dashboard_msgs__msg__SafetyMode);
    aubo_dashboard_msgs__msg__SafetyMode * data =
      (aubo_dashboard_msgs__msg__SafetyMode *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!aubo_dashboard_msgs__msg__SafetyMode__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          aubo_dashboard_msgs__msg__SafetyMode__fini(&data[i]);
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
    if (!aubo_dashboard_msgs__msg__SafetyMode__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
