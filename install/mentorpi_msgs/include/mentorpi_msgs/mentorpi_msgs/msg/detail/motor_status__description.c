// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mentorpi_msgs:msg/MotorStatus.idl
// generated code does not contain a copyright notice

#include "mentorpi_msgs/msg/detail/motor_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mentorpi_msgs
const rosidl_type_hash_t *
mentorpi_msgs__msg__MotorStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x5c, 0xe5, 0xf9, 0xb8, 0x7a, 0x2f, 0xf8, 0x08,
      0x5f, 0xbf, 0xa7, 0xb4, 0x46, 0xf4, 0x65, 0x1c,
      0x7d, 0xc4, 0x44, 0xdb, 0x45, 0x5b, 0x25, 0x4d,
      0x3c, 0xa4, 0xa6, 0x47, 0xda, 0xdb, 0xca, 0xf0,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char mentorpi_msgs__msg__MotorStatus__TYPE_NAME[] = "mentorpi_msgs/msg/MotorStatus";

// Define type names, field names, and default values
static char mentorpi_msgs__msg__MotorStatus__FIELD_NAME__wheel_speed[] = "wheel_speed";
static char mentorpi_msgs__msg__MotorStatus__FIELD_NAME__wheel_position[] = "wheel_position";

static rosidl_runtime_c__type_description__Field mentorpi_msgs__msg__MotorStatus__FIELDS[] = {
  {
    {mentorpi_msgs__msg__MotorStatus__FIELD_NAME__wheel_speed, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mentorpi_msgs__msg__MotorStatus__FIELD_NAME__wheel_position, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mentorpi_msgs__msg__MotorStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mentorpi_msgs__msg__MotorStatus__TYPE_NAME, 29, 29},
      {mentorpi_msgs__msg__MotorStatus__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64[4] wheel_speed\n"
  "float64[4] wheel_position";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mentorpi_msgs__msg__MotorStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mentorpi_msgs__msg__MotorStatus__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 49, 49},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mentorpi_msgs__msg__MotorStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mentorpi_msgs__msg__MotorStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
