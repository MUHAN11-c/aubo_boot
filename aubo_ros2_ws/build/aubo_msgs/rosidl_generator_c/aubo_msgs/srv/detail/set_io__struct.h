// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from aubo_msgs:srv/SetIO.idl
// generated code does not contain a copyright notice

#ifndef AUBO_MSGS__SRV__DETAIL__SET_IO__STRUCT_H_
#define AUBO_MSGS__SRV__DETAIL__SET_IO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'PIN_AOUT0'.
enum
{
  aubo_msgs__srv__SetIO_Request__PIN_AOUT0 = 0
};

/// Constant 'PIN_AOUT1'.
enum
{
  aubo_msgs__srv__SetIO_Request__PIN_AOUT1 = 1
};

/// Constant 'PIN_DOUT0'.
enum
{
  aubo_msgs__srv__SetIO_Request__PIN_DOUT0 = 0
};

/// Constant 'PIN_DOUT1'.
enum
{
  aubo_msgs__srv__SetIO_Request__PIN_DOUT1 = 1
};

/// Constant 'PIN_DOUT2'.
enum
{
  aubo_msgs__srv__SetIO_Request__PIN_DOUT2 = 2
};

/// Constant 'PIN_DOUT3'.
enum
{
  aubo_msgs__srv__SetIO_Request__PIN_DOUT3 = 3
};

/// Constant 'PIN_DOUT4'.
enum
{
  aubo_msgs__srv__SetIO_Request__PIN_DOUT4 = 4
};

/// Constant 'PIN_DOUT5'.
enum
{
  aubo_msgs__srv__SetIO_Request__PIN_DOUT5 = 5
};

/// Constant 'PIN_DOUT6'.
enum
{
  aubo_msgs__srv__SetIO_Request__PIN_DOUT6 = 6
};

/// Constant 'PIN_DOUT7'.
enum
{
  aubo_msgs__srv__SetIO_Request__PIN_DOUT7 = 7
};

/// Constant 'PIN_CONF_OUT0'.
enum
{
  aubo_msgs__srv__SetIO_Request__PIN_CONF_OUT0 = 8
};

/// Constant 'PIN_CONF_OUT1'.
enum
{
  aubo_msgs__srv__SetIO_Request__PIN_CONF_OUT1 = 9
};

/// Constant 'PIN_CONF_OUT2'.
enum
{
  aubo_msgs__srv__SetIO_Request__PIN_CONF_OUT2 = 10
};

/// Constant 'PIN_CONF_OUT3'.
enum
{
  aubo_msgs__srv__SetIO_Request__PIN_CONF_OUT3 = 11
};

/// Constant 'PIN_CONF_OUT4'.
enum
{
  aubo_msgs__srv__SetIO_Request__PIN_CONF_OUT4 = 12
};

/// Constant 'PIN_CONF_OUT5'.
enum
{
  aubo_msgs__srv__SetIO_Request__PIN_CONF_OUT5 = 13
};

/// Constant 'PIN_CONF_OUT6'.
enum
{
  aubo_msgs__srv__SetIO_Request__PIN_CONF_OUT6 = 14
};

/// Constant 'PIN_CONF_OUT7'.
enum
{
  aubo_msgs__srv__SetIO_Request__PIN_CONF_OUT7 = 15
};

/// Constant 'PIN_TOOL_DOUT0'.
enum
{
  aubo_msgs__srv__SetIO_Request__PIN_TOOL_DOUT0 = 16
};

/// Constant 'PIN_TOOL_DOUT1'.
enum
{
  aubo_msgs__srv__SetIO_Request__PIN_TOOL_DOUT1 = 17
};

/// Constant 'FUN_SET_DIGITAL_OUT'.
enum
{
  aubo_msgs__srv__SetIO_Request__FUN_SET_DIGITAL_OUT = 1
};

/// Constant 'FUN_SET_FLAG'.
enum
{
  aubo_msgs__srv__SetIO_Request__FUN_SET_FLAG = 2
};

/// Constant 'FUN_SET_ANALOG_OUT'.
enum
{
  aubo_msgs__srv__SetIO_Request__FUN_SET_ANALOG_OUT = 3
};

/// Constant 'FUN_SET_TOOL_VOLTAGE'.
enum
{
  aubo_msgs__srv__SetIO_Request__FUN_SET_TOOL_VOLTAGE = 4
};

/// Constant 'STATE_OFF'.
enum
{
  aubo_msgs__srv__SetIO_Request__STATE_OFF = 0
};

/// Constant 'STATE_ON'.
enum
{
  aubo_msgs__srv__SetIO_Request__STATE_ON = 1
};

/// Constant 'STATE_TOOL_VOLTAGE_0V'.
enum
{
  aubo_msgs__srv__SetIO_Request__STATE_TOOL_VOLTAGE_0V = 0
};

/// Constant 'STATE_TOOL_VOLTAGE_12V'.
enum
{
  aubo_msgs__srv__SetIO_Request__STATE_TOOL_VOLTAGE_12V = 12
};

/// Constant 'STATE_TOOL_VOLTAGE_24V'.
enum
{
  aubo_msgs__srv__SetIO_Request__STATE_TOOL_VOLTAGE_24V = 24
};

// Struct defined in srv/SetIO in the package aubo_msgs.
typedef struct aubo_msgs__srv__SetIO_Request
{
  int8_t fun;
  int8_t pin;
  float state;
} aubo_msgs__srv__SetIO_Request;

// Struct for a sequence of aubo_msgs__srv__SetIO_Request.
typedef struct aubo_msgs__srv__SetIO_Request__Sequence
{
  aubo_msgs__srv__SetIO_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aubo_msgs__srv__SetIO_Request__Sequence;


// Constants defined in the message

// Struct defined in srv/SetIO in the package aubo_msgs.
typedef struct aubo_msgs__srv__SetIO_Response
{
  bool success;
} aubo_msgs__srv__SetIO_Response;

// Struct for a sequence of aubo_msgs__srv__SetIO_Response.
typedef struct aubo_msgs__srv__SetIO_Response__Sequence
{
  aubo_msgs__srv__SetIO_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aubo_msgs__srv__SetIO_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUBO_MSGS__SRV__DETAIL__SET_IO__STRUCT_H_
