// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mentorpi_msgs:msg/Gimbal.idl
// generated code does not contain a copyright notice

#include "mentorpi_msgs/msg/detail/gimbal__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mentorpi_msgs
const rosidl_type_hash_t *
mentorpi_msgs__msg__Gimbal__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x74, 0x5c, 0x2c, 0x36, 0xf7, 0x75, 0xca, 0x37,
      0x47, 0xf4, 0x27, 0xf6, 0x7d, 0xb6, 0x8d, 0x9f,
      0x05, 0x9e, 0xf9, 0x7d, 0x1b, 0x81, 0xd9, 0xdd,
      0xc5, 0x90, 0x26, 0xd3, 0xec, 0x3c, 0x37, 0x07,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char mentorpi_msgs__msg__Gimbal__TYPE_NAME[] = "mentorpi_msgs/msg/Gimbal";

// Define type names, field names, and default values
static char mentorpi_msgs__msg__Gimbal__FIELD_NAME__pitch[] = "pitch";
static char mentorpi_msgs__msg__Gimbal__FIELD_NAME__yaw[] = "yaw";

static rosidl_runtime_c__type_description__Field mentorpi_msgs__msg__Gimbal__FIELDS[] = {
  {
    {mentorpi_msgs__msg__Gimbal__FIELD_NAME__pitch, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mentorpi_msgs__msg__Gimbal__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mentorpi_msgs__msg__Gimbal__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mentorpi_msgs__msg__Gimbal__TYPE_NAME, 24, 24},
      {mentorpi_msgs__msg__Gimbal__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64 pitch\n"
  "float64 yaw";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mentorpi_msgs__msg__Gimbal__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mentorpi_msgs__msg__Gimbal__TYPE_NAME, 24, 24},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 26, 26},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mentorpi_msgs__msg__Gimbal__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mentorpi_msgs__msg__Gimbal__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
