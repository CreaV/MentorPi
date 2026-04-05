// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mentorpi_msgs:msg/MotorStatus.idl
// generated code does not contain a copyright notice
#include "mentorpi_msgs/msg/detail/motor_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
mentorpi_msgs__msg__MotorStatus__init(mentorpi_msgs__msg__MotorStatus * msg)
{
  if (!msg) {
    return false;
  }
  // wheel_speed
  // wheel_position
  return true;
}

void
mentorpi_msgs__msg__MotorStatus__fini(mentorpi_msgs__msg__MotorStatus * msg)
{
  if (!msg) {
    return;
  }
  // wheel_speed
  // wheel_position
}

bool
mentorpi_msgs__msg__MotorStatus__are_equal(const mentorpi_msgs__msg__MotorStatus * lhs, const mentorpi_msgs__msg__MotorStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // wheel_speed
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->wheel_speed[i] != rhs->wheel_speed[i]) {
      return false;
    }
  }
  // wheel_position
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->wheel_position[i] != rhs->wheel_position[i]) {
      return false;
    }
  }
  return true;
}

bool
mentorpi_msgs__msg__MotorStatus__copy(
  const mentorpi_msgs__msg__MotorStatus * input,
  mentorpi_msgs__msg__MotorStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // wheel_speed
  for (size_t i = 0; i < 4; ++i) {
    output->wheel_speed[i] = input->wheel_speed[i];
  }
  // wheel_position
  for (size_t i = 0; i < 4; ++i) {
    output->wheel_position[i] = input->wheel_position[i];
  }
  return true;
}

mentorpi_msgs__msg__MotorStatus *
mentorpi_msgs__msg__MotorStatus__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mentorpi_msgs__msg__MotorStatus * msg = (mentorpi_msgs__msg__MotorStatus *)allocator.allocate(sizeof(mentorpi_msgs__msg__MotorStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mentorpi_msgs__msg__MotorStatus));
  bool success = mentorpi_msgs__msg__MotorStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mentorpi_msgs__msg__MotorStatus__destroy(mentorpi_msgs__msg__MotorStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mentorpi_msgs__msg__MotorStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mentorpi_msgs__msg__MotorStatus__Sequence__init(mentorpi_msgs__msg__MotorStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mentorpi_msgs__msg__MotorStatus * data = NULL;

  if (size) {
    data = (mentorpi_msgs__msg__MotorStatus *)allocator.zero_allocate(size, sizeof(mentorpi_msgs__msg__MotorStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mentorpi_msgs__msg__MotorStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mentorpi_msgs__msg__MotorStatus__fini(&data[i - 1]);
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
mentorpi_msgs__msg__MotorStatus__Sequence__fini(mentorpi_msgs__msg__MotorStatus__Sequence * array)
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
      mentorpi_msgs__msg__MotorStatus__fini(&array->data[i]);
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

mentorpi_msgs__msg__MotorStatus__Sequence *
mentorpi_msgs__msg__MotorStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mentorpi_msgs__msg__MotorStatus__Sequence * array = (mentorpi_msgs__msg__MotorStatus__Sequence *)allocator.allocate(sizeof(mentorpi_msgs__msg__MotorStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mentorpi_msgs__msg__MotorStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mentorpi_msgs__msg__MotorStatus__Sequence__destroy(mentorpi_msgs__msg__MotorStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mentorpi_msgs__msg__MotorStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mentorpi_msgs__msg__MotorStatus__Sequence__are_equal(const mentorpi_msgs__msg__MotorStatus__Sequence * lhs, const mentorpi_msgs__msg__MotorStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mentorpi_msgs__msg__MotorStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mentorpi_msgs__msg__MotorStatus__Sequence__copy(
  const mentorpi_msgs__msg__MotorStatus__Sequence * input,
  mentorpi_msgs__msg__MotorStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mentorpi_msgs__msg__MotorStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mentorpi_msgs__msg__MotorStatus * data =
      (mentorpi_msgs__msg__MotorStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mentorpi_msgs__msg__MotorStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mentorpi_msgs__msg__MotorStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mentorpi_msgs__msg__MotorStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
