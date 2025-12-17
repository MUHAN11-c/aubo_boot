// Auto-generated. Do not edit!

// (in-package demo_interface.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ToolIOStatus = require('./ToolIOStatus.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class RobotIOStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.digital_inputs = null;
      this.digital_outputs = null;
      this.analog_inputs = null;
      this.analog_outputs = null;
      this.tool_io_status = null;
      this.is_connected = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('digital_inputs')) {
        this.digital_inputs = initObj.digital_inputs
      }
      else {
        this.digital_inputs = [];
      }
      if (initObj.hasOwnProperty('digital_outputs')) {
        this.digital_outputs = initObj.digital_outputs
      }
      else {
        this.digital_outputs = [];
      }
      if (initObj.hasOwnProperty('analog_inputs')) {
        this.analog_inputs = initObj.analog_inputs
      }
      else {
        this.analog_inputs = [];
      }
      if (initObj.hasOwnProperty('analog_outputs')) {
        this.analog_outputs = initObj.analog_outputs
      }
      else {
        this.analog_outputs = [];
      }
      if (initObj.hasOwnProperty('tool_io_status')) {
        this.tool_io_status = initObj.tool_io_status
      }
      else {
        this.tool_io_status = new ToolIOStatus();
      }
      if (initObj.hasOwnProperty('is_connected')) {
        this.is_connected = initObj.is_connected
      }
      else {
        this.is_connected = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotIOStatus
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [digital_inputs]
    bufferOffset = _arraySerializer.bool(obj.digital_inputs, buffer, bufferOffset, null);
    // Serialize message field [digital_outputs]
    bufferOffset = _arraySerializer.bool(obj.digital_outputs, buffer, bufferOffset, null);
    // Serialize message field [analog_inputs]
    bufferOffset = _arraySerializer.float32(obj.analog_inputs, buffer, bufferOffset, null);
    // Serialize message field [analog_outputs]
    bufferOffset = _arraySerializer.float32(obj.analog_outputs, buffer, bufferOffset, null);
    // Serialize message field [tool_io_status]
    bufferOffset = ToolIOStatus.serialize(obj.tool_io_status, buffer, bufferOffset);
    // Serialize message field [is_connected]
    bufferOffset = _serializer.bool(obj.is_connected, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotIOStatus
    let len;
    let data = new RobotIOStatus(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [digital_inputs]
    data.digital_inputs = _arrayDeserializer.bool(buffer, bufferOffset, null)
    // Deserialize message field [digital_outputs]
    data.digital_outputs = _arrayDeserializer.bool(buffer, bufferOffset, null)
    // Deserialize message field [analog_inputs]
    data.analog_inputs = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [analog_outputs]
    data.analog_outputs = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [tool_io_status]
    data.tool_io_status = ToolIOStatus.deserialize(buffer, bufferOffset);
    // Deserialize message field [is_connected]
    data.is_connected = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.digital_inputs.length;
    length += object.digital_outputs.length;
    length += 4 * object.analog_inputs.length;
    length += 4 * object.analog_outputs.length;
    length += ToolIOStatus.getMessageSize(object.tool_io_status);
    return length + 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'demo_interface/RobotIOStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '241d2a7d7ac2df6c1d3ad9602be6f25a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    bool[] digital_inputs
    bool[] digital_outputs
    float32[] analog_inputs
    float32[] analog_outputs
    ToolIOStatus tool_io_status
    bool is_connected
    
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: demo_interface/ToolIOStatus
    bool[] digital_inputs
    bool[] digital_outputs
    float32[] analog_inputs
    float32[] analog_outputs
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RobotIOStatus(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.digital_inputs !== undefined) {
      resolved.digital_inputs = msg.digital_inputs;
    }
    else {
      resolved.digital_inputs = []
    }

    if (msg.digital_outputs !== undefined) {
      resolved.digital_outputs = msg.digital_outputs;
    }
    else {
      resolved.digital_outputs = []
    }

    if (msg.analog_inputs !== undefined) {
      resolved.analog_inputs = msg.analog_inputs;
    }
    else {
      resolved.analog_inputs = []
    }

    if (msg.analog_outputs !== undefined) {
      resolved.analog_outputs = msg.analog_outputs;
    }
    else {
      resolved.analog_outputs = []
    }

    if (msg.tool_io_status !== undefined) {
      resolved.tool_io_status = ToolIOStatus.Resolve(msg.tool_io_status)
    }
    else {
      resolved.tool_io_status = new ToolIOStatus()
    }

    if (msg.is_connected !== undefined) {
      resolved.is_connected = msg.is_connected;
    }
    else {
      resolved.is_connected = false
    }

    return resolved;
    }
};

module.exports = RobotIOStatus;
