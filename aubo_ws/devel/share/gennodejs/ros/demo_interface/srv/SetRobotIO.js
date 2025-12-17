// Auto-generated. Do not edit!

// (in-package demo_interface.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetRobotIORequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.io_type = null;
      this.io_index = null;
      this.value = null;
    }
    else {
      if (initObj.hasOwnProperty('io_type')) {
        this.io_type = initObj.io_type
      }
      else {
        this.io_type = '';
      }
      if (initObj.hasOwnProperty('io_index')) {
        this.io_index = initObj.io_index
      }
      else {
        this.io_index = 0;
      }
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetRobotIORequest
    // Serialize message field [io_type]
    bufferOffset = _serializer.string(obj.io_type, buffer, bufferOffset);
    // Serialize message field [io_index]
    bufferOffset = _serializer.int32(obj.io_index, buffer, bufferOffset);
    // Serialize message field [value]
    bufferOffset = _serializer.float64(obj.value, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetRobotIORequest
    let len;
    let data = new SetRobotIORequest(null);
    // Deserialize message field [io_type]
    data.io_type = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [io_index]
    data.io_index = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [value]
    data.value = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.io_type);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'demo_interface/SetRobotIORequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd14c5359d5a6bcfa1747111fae77ea69';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string io_type
    int32 io_index
    float64 value
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetRobotIORequest(null);
    if (msg.io_type !== undefined) {
      resolved.io_type = msg.io_type;
    }
    else {
      resolved.io_type = ''
    }

    if (msg.io_index !== undefined) {
      resolved.io_index = msg.io_index;
    }
    else {
      resolved.io_index = 0
    }

    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = 0.0
    }

    return resolved;
    }
};

class SetRobotIOResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.error_code = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('error_code')) {
        this.error_code = initObj.error_code
      }
      else {
        this.error_code = 0;
      }
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetRobotIOResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [error_code]
    bufferOffset = _serializer.int32(obj.error_code, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetRobotIOResponse
    let len;
    let data = new SetRobotIOResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [error_code]
    data.error_code = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'demo_interface/SetRobotIOResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9df8f352a4ad3d1b171a2f58ee3b8ad6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    int32 error_code
    string message
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetRobotIOResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.error_code !== undefined) {
      resolved.error_code = msg.error_code;
    }
    else {
      resolved.error_code = 0
    }

    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: SetRobotIORequest,
  Response: SetRobotIOResponse,
  md5sum() { return '2e3250f0a25c75ced7db303a09e99be6'; },
  datatype() { return 'demo_interface/SetRobotIO'; }
};
