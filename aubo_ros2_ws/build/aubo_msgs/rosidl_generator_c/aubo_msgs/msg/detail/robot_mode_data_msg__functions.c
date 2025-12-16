// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from aubo_msgs:msg/RobotModeDataMsg.idl
// generated code does not contain a copyright notice
#include "aubo_msgs/msg/detail/robot_mode_data_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
aubo_msgs__msg__RobotModeDataMsg__init(aubo_msgs__msg__RobotModeDataMsg * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // is_robot_connected
  // is_real_robot_enabled
  // is_power_on_robot
  // is_emergency_stopped
  // is_protective_stopped
  // is_program_running
  // is_program_paused
  return true;
}

void
aubo_msgs__msg__RobotModeDataMsg__fini(aubo_msgs__msg__RobotModeDataMsg * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // is_robot_connected
  // is_real_robot_enabled
  // is_power_on_robot
  // is_emergency_stopped
  // is_protective_stopped
  // is_program_running
  // is_program_paused
}

bool
aubo_msgs__msg__RobotModeDataMsg__are_equal(const aubo_msgs__msg__RobotModeDataMsg * lhs, const aubo_msgs__msg__RobotModeDataMsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // is_robot_connected
  if (lhs->is_robot_connected != rhs->is_robot_connected) {
    return false;
  }
  // is_real_robot_enabled
  if (lhs->is_real_robot_enabled != rhs->is_real_robot_enabled) {
    return false;
  }
  // is_power_on_robot
  if (lhs->is_power_on_robot != rhs->is_power_on_robot) {
    return false;
  }
  // is_emergency_stopped
  if (lhs->is_emergency_stopped != rhs->is_emergency_stopped) {
    return false;
  }
  // is_protective_stopped
  if (lhs->is_protective_stopped != rhs->is_protective_stopped) {
    return false;
  }
  // is_program_running
  if (lhs->is_program_running != rhs->is_program_running) {
    return false;
  }
  // is_program_paused
  if (lhs->is_program_paused != rhs->is_program_paused) {
    return false;
  }
  return true;
}

bool
aubo_msgs__msg__RobotModeDataMsg__copy(
  const aubo_msgs__msg__RobotModeDataMsg * input,
  aubo_msgs__msg__RobotModeDataMsg * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // is_robot_connected
  output->is_robot_connected = input->is_robot_connected;
  // is_real_robot_enabled
  output->is_real_robot_enabled = input->is_real_robot_enabled;
  // is_power_on_robot
  output->is_power_on_robot = input->is_power_on_robot;
  // is_emergency_stopped
  output->is_emergency_stopped = input->is_emergency_stopped;
  // is_protective_stopped
  output->is_protective_stopped = input->is_protective_stopped;
  // is_program_running
  output->is_program_running = input->is_program_running;
  // is_program_paused
  output->is_program_paused = input->is_program_paused;
  return true;
}

aubo_msgs__msg__RobotModeDataMsg *
aubo_msgs__msg__RobotModeDataMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aubo_msgs__msg__RobotModeDataMsg * msg = (aubo_msgs__msg__RobotModeDataMsg *)allocator.allocate(sizeof(aubo_msgs__msg__RobotModeDataMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(aubo_msgs__msg__RobotModeDataMsg));
  bool success = aubo_msgs__msg__RobotModeDataMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
aubo_msgs__msg__RobotModeDataMsg__destroy(aubo_msgs__msg__RobotModeDataMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    aubo_msgs__msg__RobotModeDataMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
aubo_msgs__msg__RobotModeDataMsg__Sequence__init(aubo_msgs__msg__RobotModeDataMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aubo_msgs__msg__RobotModeDataMsg * data = NULL;

  if (size) {
    data = (aubo_msgs__msg__RobotModeDataMsg *)allocator.zero_allocate(size, sizeof(aubo_msgs__msg__RobotModeDataMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = aubo_msgs__msg__RobotModeDataMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        aubo_msgs__msg__RobotModeDataMsg__fini(&data[i - 1]);
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
aubo_msgs__msg__RobotModeDataMsg__Sequence__fini(aubo_msgs__msg__RobotModeDataMsg__Sequence * array)
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
      aubo_msgs__msg__RobotModeDataMsg__fini(&array->data[i]);
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

aubo_msgs__msg__RobotModeDataMsg__Sequence *
aubo_msgs__msg__RobotModeDataMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aubo_msgs__msg__RobotModeDataMsg__Sequence * array = (aubo_msgs__msg__RobotModeDataMsg__Sequence *)allocator.allocate(sizeof(aubo_msgs__msg__RobotModeDataMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = aubo_msgs__msg__RobotModeDataMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
aubo_msgs__msg__RobotModeDataMsg__Sequence__destroy(aubo_msgs__msg__RobotModeDataMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    aubo_msgs__msg__RobotModeDataMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
aubo_msgs__msg__RobotModeDataMsg__Sequence__are_equal(const aubo_msgs__msg__RobotModeDataMsg__Sequence * lhs, const aubo_msgs__msg__RobotModeDataMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!aubo_msgs__msg__RobotModeDataMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
aubo_msgs__msg__RobotModeDataMsg__Sequence__copy(
  const aubo_msgs__msg__RobotModeDataMsg__Sequence * input,
  aubo_msgs__msg__RobotModeDataMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(aubo_msgs__msg__RobotModeDataMsg);
    aubo_msgs__msg__RobotModeDataMsg * data =
      (aubo_msgs__msg__RobotModeDataMsg *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!aubo_msgs__msg__RobotModeDataMsg__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          aubo_msgs__msg__RobotModeDataMsg__fini(&data[i]);
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
    if (!aubo_msgs__msg__RobotModeDataMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
