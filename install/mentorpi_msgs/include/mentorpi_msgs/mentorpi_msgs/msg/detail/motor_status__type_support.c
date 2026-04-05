// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mentorpi_msgs:msg/MotorStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mentorpi_msgs/msg/detail/motor_status__rosidl_typesupport_introspection_c.h"
#include "mentorpi_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mentorpi_msgs/msg/detail/motor_status__functions.h"
#include "mentorpi_msgs/msg/detail/motor_status__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__MotorStatus_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mentorpi_msgs__msg__MotorStatus__init(message_memory);
}

void mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__MotorStatus_fini_function(void * message_memory)
{
  mentorpi_msgs__msg__MotorStatus__fini(message_memory);
}

size_t mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__size_function__MotorStatus__wheel_speed(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__get_const_function__MotorStatus__wheel_speed(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__get_function__MotorStatus__wheel_speed(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__fetch_function__MotorStatus__wheel_speed(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__get_const_function__MotorStatus__wheel_speed(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__assign_function__MotorStatus__wheel_speed(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__get_function__MotorStatus__wheel_speed(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__size_function__MotorStatus__wheel_position(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__get_const_function__MotorStatus__wheel_position(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__get_function__MotorStatus__wheel_position(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__fetch_function__MotorStatus__wheel_position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__get_const_function__MotorStatus__wheel_position(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__assign_function__MotorStatus__wheel_position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__get_function__MotorStatus__wheel_position(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__MotorStatus_message_member_array[2] = {
  {
    "wheel_speed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(mentorpi_msgs__msg__MotorStatus, wheel_speed),  // bytes offset in struct
    NULL,  // default value
    mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__size_function__MotorStatus__wheel_speed,  // size() function pointer
    mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__get_const_function__MotorStatus__wheel_speed,  // get_const(index) function pointer
    mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__get_function__MotorStatus__wheel_speed,  // get(index) function pointer
    mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__fetch_function__MotorStatus__wheel_speed,  // fetch(index, &value) function pointer
    mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__assign_function__MotorStatus__wheel_speed,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "wheel_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(mentorpi_msgs__msg__MotorStatus, wheel_position),  // bytes offset in struct
    NULL,  // default value
    mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__size_function__MotorStatus__wheel_position,  // size() function pointer
    mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__get_const_function__MotorStatus__wheel_position,  // get_const(index) function pointer
    mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__get_function__MotorStatus__wheel_position,  // get(index) function pointer
    mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__fetch_function__MotorStatus__wheel_position,  // fetch(index, &value) function pointer
    mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__assign_function__MotorStatus__wheel_position,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__MotorStatus_message_members = {
  "mentorpi_msgs__msg",  // message namespace
  "MotorStatus",  // message name
  2,  // number of fields
  sizeof(mentorpi_msgs__msg__MotorStatus),
  false,  // has_any_key_member_
  mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__MotorStatus_message_member_array,  // message members
  mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__MotorStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__MotorStatus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__MotorStatus_message_type_support_handle = {
  0,
  &mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__MotorStatus_message_members,
  get_message_typesupport_handle_function,
  &mentorpi_msgs__msg__MotorStatus__get_type_hash,
  &mentorpi_msgs__msg__MotorStatus__get_type_description,
  &mentorpi_msgs__msg__MotorStatus__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mentorpi_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mentorpi_msgs, msg, MotorStatus)() {
  if (!mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__MotorStatus_message_type_support_handle.typesupport_identifier) {
    mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__MotorStatus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mentorpi_msgs__msg__MotorStatus__rosidl_typesupport_introspection_c__MotorStatus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
