// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from aubo_msgs:msg/Analog.idl
// generated code does not contain a copyright notice
#include "aubo_msgs/msg/detail/analog__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
aubo_msgs__msg__Analog__init(aubo_msgs__msg__Analog * msg)
{
  if (!msg) {
    return false;
  }
  // pin
  // domain
  // state
  return true;
}

void
aubo_msgs__msg__Analog__fini(aubo_msgs__msg__Analog * msg)
{
  if (!msg) {
    return;
  }
  // pin
  // domain
  // state
}

bool
aubo_msgs__msg__Analog__are_equal(const aubo_msgs__msg__Analog * lhs, const aubo_msgs__msg__Analog * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pin
  if (lhs->pin != rhs->pin) {
    return false;
  }
  // domain
  if (lhs->domain != rhs->domain) {
    return false;
  }
  // state
  if (lhs->state != rhs->state) {
    return false;
  }
  return true;
}

bool
aubo_msgs__msg__Analog__copy(
  const aubo_msgs__msg__Analog * input,
  aubo_msgs__msg__Analog * output)
{
  if (!input || !output) {
    return false;
  }
  // pin
  output->pin = input->pin;
  // domain
  output->domain = input->domain;
  // state
  output->state = input->state;
  return true;
}

aubo_msgs__msg__Analog *
aubo_msgs__msg__Analog__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aubo_msgs__msg__Analog * msg = (aubo_msgs__msg__Analog *)allocator.allocate(sizeof(aubo_msgs__msg__Analog), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(aubo_msgs__msg__Analog));
  bool success = aubo_msgs__msg__Analog__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
aubo_msgs__msg__Analog__destroy(aubo_msgs__msg__Analog * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    aubo_msgs__msg__Analog__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
aubo_msgs__msg__Analog__Sequence__init(aubo_msgs__msg__Analog__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aubo_msgs__msg__Analog * data = NULL;

  if (size) {
    data = (aubo_msgs__msg__Analog *)allocator.zero_allocate(size, sizeof(aubo_msgs__msg__Analog), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = aubo_msgs__msg__Analog__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        aubo_msgs__msg__Analog__fini(&data[i - 1]);
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
aubo_msgs__msg__Analog__Sequence__fini(aubo_msgs__msg__Analog__Sequence * array)
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
      aubo_msgs__msg__Analog__fini(&array->data[i]);
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

aubo_msgs__msg__Analog__Sequence *
aubo_msgs__msg__Analog__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aubo_msgs__msg__Analog__Sequence * array = (aubo_msgs__msg__Analog__Sequence *)allocator.allocate(sizeof(aubo_msgs__msg__Analog__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = aubo_msgs__msg__Analog__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
aubo_msgs__msg__Analog__Sequence__destroy(aubo_msgs__msg__Analog__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    aubo_msgs__msg__Analog__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
aubo_msgs__msg__Analog__Sequence__are_equal(const aubo_msgs__msg__Analog__Sequence * lhs, const aubo_msgs__msg__Analog__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!aubo_msgs__msg__Analog__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
aubo_msgs__msg__Analog__Sequence__copy(
  const aubo_msgs__msg__Analog__Sequence * input,
  aubo_msgs__msg__Analog__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(aubo_msgs__msg__Analog);
    aubo_msgs__msg__Analog * data =
      (aubo_msgs__msg__Analog *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!aubo_msgs__msg__Analog__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          aubo_msgs__msg__Analog__fini(&data[i]);
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
    if (!aubo_msgs__msg__Analog__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
