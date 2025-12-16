// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from aubo_msgs:msg/Digital.idl
// generated code does not contain a copyright notice
#include "aubo_msgs/msg/detail/digital__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
aubo_msgs__msg__Digital__init(aubo_msgs__msg__Digital * msg)
{
  if (!msg) {
    return false;
  }
  // pin
  // state
  return true;
}

void
aubo_msgs__msg__Digital__fini(aubo_msgs__msg__Digital * msg)
{
  if (!msg) {
    return;
  }
  // pin
  // state
}

bool
aubo_msgs__msg__Digital__are_equal(const aubo_msgs__msg__Digital * lhs, const aubo_msgs__msg__Digital * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pin
  if (lhs->pin != rhs->pin) {
    return false;
  }
  // state
  if (lhs->state != rhs->state) {
    return false;
  }
  return true;
}

bool
aubo_msgs__msg__Digital__copy(
  const aubo_msgs__msg__Digital * input,
  aubo_msgs__msg__Digital * output)
{
  if (!input || !output) {
    return false;
  }
  // pin
  output->pin = input->pin;
  // state
  output->state = input->state;
  return true;
}

aubo_msgs__msg__Digital *
aubo_msgs__msg__Digital__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aubo_msgs__msg__Digital * msg = (aubo_msgs__msg__Digital *)allocator.allocate(sizeof(aubo_msgs__msg__Digital), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(aubo_msgs__msg__Digital));
  bool success = aubo_msgs__msg__Digital__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
aubo_msgs__msg__Digital__destroy(aubo_msgs__msg__Digital * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    aubo_msgs__msg__Digital__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
aubo_msgs__msg__Digital__Sequence__init(aubo_msgs__msg__Digital__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aubo_msgs__msg__Digital * data = NULL;

  if (size) {
    data = (aubo_msgs__msg__Digital *)allocator.zero_allocate(size, sizeof(aubo_msgs__msg__Digital), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = aubo_msgs__msg__Digital__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        aubo_msgs__msg__Digital__fini(&data[i - 1]);
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
aubo_msgs__msg__Digital__Sequence__fini(aubo_msgs__msg__Digital__Sequence * array)
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
      aubo_msgs__msg__Digital__fini(&array->data[i]);
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

aubo_msgs__msg__Digital__Sequence *
aubo_msgs__msg__Digital__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aubo_msgs__msg__Digital__Sequence * array = (aubo_msgs__msg__Digital__Sequence *)allocator.allocate(sizeof(aubo_msgs__msg__Digital__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = aubo_msgs__msg__Digital__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
aubo_msgs__msg__Digital__Sequence__destroy(aubo_msgs__msg__Digital__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    aubo_msgs__msg__Digital__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
aubo_msgs__msg__Digital__Sequence__are_equal(const aubo_msgs__msg__Digital__Sequence * lhs, const aubo_msgs__msg__Digital__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!aubo_msgs__msg__Digital__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
aubo_msgs__msg__Digital__Sequence__copy(
  const aubo_msgs__msg__Digital__Sequence * input,
  aubo_msgs__msg__Digital__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(aubo_msgs__msg__Digital);
    aubo_msgs__msg__Digital * data =
      (aubo_msgs__msg__Digital *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!aubo_msgs__msg__Digital__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          aubo_msgs__msg__Digital__fini(&data[i]);
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
    if (!aubo_msgs__msg__Digital__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
