// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from aubo_msgs:msg/MasterboardDataMsg.idl
// generated code does not contain a copyright notice
#include "aubo_msgs/msg/detail/masterboard_data_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
aubo_msgs__msg__MasterboardDataMsg__init(aubo_msgs__msg__MasterboardDataMsg * msg)
{
  if (!msg) {
    return false;
  }
  // digital_input_bits
  // digital_output_bits
  // analog_input_range0
  // analog_input_range1
  // analog_input0
  // analog_input1
  // analog_output_domain0
  // analog_output_domain1
  // analog_output0
  // analog_output1
  // masterboard_temperature
  // robot_voltage_48v
  // robot_current
  // master_io_current
  // master_safety_state
  // master_onoff_state
  return true;
}

void
aubo_msgs__msg__MasterboardDataMsg__fini(aubo_msgs__msg__MasterboardDataMsg * msg)
{
  if (!msg) {
    return;
  }
  // digital_input_bits
  // digital_output_bits
  // analog_input_range0
  // analog_input_range1
  // analog_input0
  // analog_input1
  // analog_output_domain0
  // analog_output_domain1
  // analog_output0
  // analog_output1
  // masterboard_temperature
  // robot_voltage_48v
  // robot_current
  // master_io_current
  // master_safety_state
  // master_onoff_state
}

bool
aubo_msgs__msg__MasterboardDataMsg__are_equal(const aubo_msgs__msg__MasterboardDataMsg * lhs, const aubo_msgs__msg__MasterboardDataMsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // digital_input_bits
  if (lhs->digital_input_bits != rhs->digital_input_bits) {
    return false;
  }
  // digital_output_bits
  if (lhs->digital_output_bits != rhs->digital_output_bits) {
    return false;
  }
  // analog_input_range0
  if (lhs->analog_input_range0 != rhs->analog_input_range0) {
    return false;
  }
  // analog_input_range1
  if (lhs->analog_input_range1 != rhs->analog_input_range1) {
    return false;
  }
  // analog_input0
  if (lhs->analog_input0 != rhs->analog_input0) {
    return false;
  }
  // analog_input1
  if (lhs->analog_input1 != rhs->analog_input1) {
    return false;
  }
  // analog_output_domain0
  if (lhs->analog_output_domain0 != rhs->analog_output_domain0) {
    return false;
  }
  // analog_output_domain1
  if (lhs->analog_output_domain1 != rhs->analog_output_domain1) {
    return false;
  }
  // analog_output0
  if (lhs->analog_output0 != rhs->analog_output0) {
    return false;
  }
  // analog_output1
  if (lhs->analog_output1 != rhs->analog_output1) {
    return false;
  }
  // masterboard_temperature
  if (lhs->masterboard_temperature != rhs->masterboard_temperature) {
    return false;
  }
  // robot_voltage_48v
  if (lhs->robot_voltage_48v != rhs->robot_voltage_48v) {
    return false;
  }
  // robot_current
  if (lhs->robot_current != rhs->robot_current) {
    return false;
  }
  // master_io_current
  if (lhs->master_io_current != rhs->master_io_current) {
    return false;
  }
  // master_safety_state
  if (lhs->master_safety_state != rhs->master_safety_state) {
    return false;
  }
  // master_onoff_state
  if (lhs->master_onoff_state != rhs->master_onoff_state) {
    return false;
  }
  return true;
}

bool
aubo_msgs__msg__MasterboardDataMsg__copy(
  const aubo_msgs__msg__MasterboardDataMsg * input,
  aubo_msgs__msg__MasterboardDataMsg * output)
{
  if (!input || !output) {
    return false;
  }
  // digital_input_bits
  output->digital_input_bits = input->digital_input_bits;
  // digital_output_bits
  output->digital_output_bits = input->digital_output_bits;
  // analog_input_range0
  output->analog_input_range0 = input->analog_input_range0;
  // analog_input_range1
  output->analog_input_range1 = input->analog_input_range1;
  // analog_input0
  output->analog_input0 = input->analog_input0;
  // analog_input1
  output->analog_input1 = input->analog_input1;
  // analog_output_domain0
  output->analog_output_domain0 = input->analog_output_domain0;
  // analog_output_domain1
  output->analog_output_domain1 = input->analog_output_domain1;
  // analog_output0
  output->analog_output0 = input->analog_output0;
  // analog_output1
  output->analog_output1 = input->analog_output1;
  // masterboard_temperature
  output->masterboard_temperature = input->masterboard_temperature;
  // robot_voltage_48v
  output->robot_voltage_48v = input->robot_voltage_48v;
  // robot_current
  output->robot_current = input->robot_current;
  // master_io_current
  output->master_io_current = input->master_io_current;
  // master_safety_state
  output->master_safety_state = input->master_safety_state;
  // master_onoff_state
  output->master_onoff_state = input->master_onoff_state;
  return true;
}

aubo_msgs__msg__MasterboardDataMsg *
aubo_msgs__msg__MasterboardDataMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aubo_msgs__msg__MasterboardDataMsg * msg = (aubo_msgs__msg__MasterboardDataMsg *)allocator.allocate(sizeof(aubo_msgs__msg__MasterboardDataMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(aubo_msgs__msg__MasterboardDataMsg));
  bool success = aubo_msgs__msg__MasterboardDataMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
aubo_msgs__msg__MasterboardDataMsg__destroy(aubo_msgs__msg__MasterboardDataMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    aubo_msgs__msg__MasterboardDataMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
aubo_msgs__msg__MasterboardDataMsg__Sequence__init(aubo_msgs__msg__MasterboardDataMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aubo_msgs__msg__MasterboardDataMsg * data = NULL;

  if (size) {
    data = (aubo_msgs__msg__MasterboardDataMsg *)allocator.zero_allocate(size, sizeof(aubo_msgs__msg__MasterboardDataMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = aubo_msgs__msg__MasterboardDataMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        aubo_msgs__msg__MasterboardDataMsg__fini(&data[i - 1]);
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
aubo_msgs__msg__MasterboardDataMsg__Sequence__fini(aubo_msgs__msg__MasterboardDataMsg__Sequence * array)
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
      aubo_msgs__msg__MasterboardDataMsg__fini(&array->data[i]);
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

aubo_msgs__msg__MasterboardDataMsg__Sequence *
aubo_msgs__msg__MasterboardDataMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aubo_msgs__msg__MasterboardDataMsg__Sequence * array = (aubo_msgs__msg__MasterboardDataMsg__Sequence *)allocator.allocate(sizeof(aubo_msgs__msg__MasterboardDataMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = aubo_msgs__msg__MasterboardDataMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
aubo_msgs__msg__MasterboardDataMsg__Sequence__destroy(aubo_msgs__msg__MasterboardDataMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    aubo_msgs__msg__MasterboardDataMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
aubo_msgs__msg__MasterboardDataMsg__Sequence__are_equal(const aubo_msgs__msg__MasterboardDataMsg__Sequence * lhs, const aubo_msgs__msg__MasterboardDataMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!aubo_msgs__msg__MasterboardDataMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
aubo_msgs__msg__MasterboardDataMsg__Sequence__copy(
  const aubo_msgs__msg__MasterboardDataMsg__Sequence * input,
  aubo_msgs__msg__MasterboardDataMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(aubo_msgs__msg__MasterboardDataMsg);
    aubo_msgs__msg__MasterboardDataMsg * data =
      (aubo_msgs__msg__MasterboardDataMsg *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!aubo_msgs__msg__MasterboardDataMsg__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          aubo_msgs__msg__MasterboardDataMsg__fini(&data[i]);
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
    if (!aubo_msgs__msg__MasterboardDataMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
