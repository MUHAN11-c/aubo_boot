// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from aubo_msgs:msg/IOStates.idl
// generated code does not contain a copyright notice
#include "aubo_msgs/msg/detail/io_states__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `digital_in_states`
// Member `digital_out_states`
// Member `flag_states`
#include "aubo_msgs/msg/detail/digital__functions.h"
// Member `analog_in_states`
// Member `analog_out_states`
#include "aubo_msgs/msg/detail/analog__functions.h"

bool
aubo_msgs__msg__IOStates__init(aubo_msgs__msg__IOStates * msg)
{
  if (!msg) {
    return false;
  }
  // digital_in_states
  if (!aubo_msgs__msg__Digital__Sequence__init(&msg->digital_in_states, 0)) {
    aubo_msgs__msg__IOStates__fini(msg);
    return false;
  }
  // digital_out_states
  if (!aubo_msgs__msg__Digital__Sequence__init(&msg->digital_out_states, 0)) {
    aubo_msgs__msg__IOStates__fini(msg);
    return false;
  }
  // flag_states
  if (!aubo_msgs__msg__Digital__Sequence__init(&msg->flag_states, 0)) {
    aubo_msgs__msg__IOStates__fini(msg);
    return false;
  }
  // analog_in_states
  if (!aubo_msgs__msg__Analog__Sequence__init(&msg->analog_in_states, 0)) {
    aubo_msgs__msg__IOStates__fini(msg);
    return false;
  }
  // analog_out_states
  if (!aubo_msgs__msg__Analog__Sequence__init(&msg->analog_out_states, 0)) {
    aubo_msgs__msg__IOStates__fini(msg);
    return false;
  }
  return true;
}

void
aubo_msgs__msg__IOStates__fini(aubo_msgs__msg__IOStates * msg)
{
  if (!msg) {
    return;
  }
  // digital_in_states
  aubo_msgs__msg__Digital__Sequence__fini(&msg->digital_in_states);
  // digital_out_states
  aubo_msgs__msg__Digital__Sequence__fini(&msg->digital_out_states);
  // flag_states
  aubo_msgs__msg__Digital__Sequence__fini(&msg->flag_states);
  // analog_in_states
  aubo_msgs__msg__Analog__Sequence__fini(&msg->analog_in_states);
  // analog_out_states
  aubo_msgs__msg__Analog__Sequence__fini(&msg->analog_out_states);
}

bool
aubo_msgs__msg__IOStates__are_equal(const aubo_msgs__msg__IOStates * lhs, const aubo_msgs__msg__IOStates * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // digital_in_states
  if (!aubo_msgs__msg__Digital__Sequence__are_equal(
      &(lhs->digital_in_states), &(rhs->digital_in_states)))
  {
    return false;
  }
  // digital_out_states
  if (!aubo_msgs__msg__Digital__Sequence__are_equal(
      &(lhs->digital_out_states), &(rhs->digital_out_states)))
  {
    return false;
  }
  // flag_states
  if (!aubo_msgs__msg__Digital__Sequence__are_equal(
      &(lhs->flag_states), &(rhs->flag_states)))
  {
    return false;
  }
  // analog_in_states
  if (!aubo_msgs__msg__Analog__Sequence__are_equal(
      &(lhs->analog_in_states), &(rhs->analog_in_states)))
  {
    return false;
  }
  // analog_out_states
  if (!aubo_msgs__msg__Analog__Sequence__are_equal(
      &(lhs->analog_out_states), &(rhs->analog_out_states)))
  {
    return false;
  }
  return true;
}

bool
aubo_msgs__msg__IOStates__copy(
  const aubo_msgs__msg__IOStates * input,
  aubo_msgs__msg__IOStates * output)
{
  if (!input || !output) {
    return false;
  }
  // digital_in_states
  if (!aubo_msgs__msg__Digital__Sequence__copy(
      &(input->digital_in_states), &(output->digital_in_states)))
  {
    return false;
  }
  // digital_out_states
  if (!aubo_msgs__msg__Digital__Sequence__copy(
      &(input->digital_out_states), &(output->digital_out_states)))
  {
    return false;
  }
  // flag_states
  if (!aubo_msgs__msg__Digital__Sequence__copy(
      &(input->flag_states), &(output->flag_states)))
  {
    return false;
  }
  // analog_in_states
  if (!aubo_msgs__msg__Analog__Sequence__copy(
      &(input->analog_in_states), &(output->analog_in_states)))
  {
    return false;
  }
  // analog_out_states
  if (!aubo_msgs__msg__Analog__Sequence__copy(
      &(input->analog_out_states), &(output->analog_out_states)))
  {
    return false;
  }
  return true;
}

aubo_msgs__msg__IOStates *
aubo_msgs__msg__IOStates__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aubo_msgs__msg__IOStates * msg = (aubo_msgs__msg__IOStates *)allocator.allocate(sizeof(aubo_msgs__msg__IOStates), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(aubo_msgs__msg__IOStates));
  bool success = aubo_msgs__msg__IOStates__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
aubo_msgs__msg__IOStates__destroy(aubo_msgs__msg__IOStates * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    aubo_msgs__msg__IOStates__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
aubo_msgs__msg__IOStates__Sequence__init(aubo_msgs__msg__IOStates__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aubo_msgs__msg__IOStates * data = NULL;

  if (size) {
    data = (aubo_msgs__msg__IOStates *)allocator.zero_allocate(size, sizeof(aubo_msgs__msg__IOStates), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = aubo_msgs__msg__IOStates__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        aubo_msgs__msg__IOStates__fini(&data[i - 1]);
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
aubo_msgs__msg__IOStates__Sequence__fini(aubo_msgs__msg__IOStates__Sequence * array)
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
      aubo_msgs__msg__IOStates__fini(&array->data[i]);
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

aubo_msgs__msg__IOStates__Sequence *
aubo_msgs__msg__IOStates__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aubo_msgs__msg__IOStates__Sequence * array = (aubo_msgs__msg__IOStates__Sequence *)allocator.allocate(sizeof(aubo_msgs__msg__IOStates__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = aubo_msgs__msg__IOStates__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
aubo_msgs__msg__IOStates__Sequence__destroy(aubo_msgs__msg__IOStates__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    aubo_msgs__msg__IOStates__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
aubo_msgs__msg__IOStates__Sequence__are_equal(const aubo_msgs__msg__IOStates__Sequence * lhs, const aubo_msgs__msg__IOStates__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!aubo_msgs__msg__IOStates__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
aubo_msgs__msg__IOStates__Sequence__copy(
  const aubo_msgs__msg__IOStates__Sequence * input,
  aubo_msgs__msg__IOStates__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(aubo_msgs__msg__IOStates);
    aubo_msgs__msg__IOStates * data =
      (aubo_msgs__msg__IOStates *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!aubo_msgs__msg__IOStates__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          aubo_msgs__msg__IOStates__fini(&data[i]);
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
    if (!aubo_msgs__msg__IOStates__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
