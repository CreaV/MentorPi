// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from mentorpi_msgs:msg/MotorStatus.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "mentorpi_msgs/msg/detail/motor_status__functions.h"
#include "mentorpi_msgs/msg/detail/motor_status__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace mentorpi_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void MotorStatus_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mentorpi_msgs::msg::MotorStatus(_init);
}

void MotorStatus_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mentorpi_msgs::msg::MotorStatus *>(message_memory);
  typed_message->~MotorStatus();
}

size_t size_function__MotorStatus__wheel_speed(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__MotorStatus__wheel_speed(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorStatus__wheel_speed(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorStatus__wheel_speed(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__MotorStatus__wheel_speed(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__MotorStatus__wheel_speed(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__MotorStatus__wheel_speed(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__MotorStatus__wheel_position(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__MotorStatus__wheel_position(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorStatus__wheel_position(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorStatus__wheel_position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__MotorStatus__wheel_position(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__MotorStatus__wheel_position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__MotorStatus__wheel_position(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MotorStatus_message_member_array[2] = {
  {
    "wheel_speed",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(mentorpi_msgs::msg::MotorStatus, wheel_speed),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorStatus__wheel_speed,  // size() function pointer
    get_const_function__MotorStatus__wheel_speed,  // get_const(index) function pointer
    get_function__MotorStatus__wheel_speed,  // get(index) function pointer
    fetch_function__MotorStatus__wheel_speed,  // fetch(index, &value) function pointer
    assign_function__MotorStatus__wheel_speed,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "wheel_position",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(mentorpi_msgs::msg::MotorStatus, wheel_position),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorStatus__wheel_position,  // size() function pointer
    get_const_function__MotorStatus__wheel_position,  // get_const(index) function pointer
    get_function__MotorStatus__wheel_position,  // get(index) function pointer
    fetch_function__MotorStatus__wheel_position,  // fetch(index, &value) function pointer
    assign_function__MotorStatus__wheel_position,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MotorStatus_message_members = {
  "mentorpi_msgs::msg",  // message namespace
  "MotorStatus",  // message name
  2,  // number of fields
  sizeof(mentorpi_msgs::msg::MotorStatus),
  false,  // has_any_key_member_
  MotorStatus_message_member_array,  // message members
  MotorStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  MotorStatus_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MotorStatus_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MotorStatus_message_members,
  get_message_typesupport_handle_function,
  &mentorpi_msgs__msg__MotorStatus__get_type_hash,
  &mentorpi_msgs__msg__MotorStatus__get_type_description,
  &mentorpi_msgs__msg__MotorStatus__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace mentorpi_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mentorpi_msgs::msg::MotorStatus>()
{
  return &::mentorpi_msgs::msg::rosidl_typesupport_introspection_cpp::MotorStatus_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mentorpi_msgs, msg, MotorStatus)() {
  return &::mentorpi_msgs::msg::rosidl_typesupport_introspection_cpp::MotorStatus_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
