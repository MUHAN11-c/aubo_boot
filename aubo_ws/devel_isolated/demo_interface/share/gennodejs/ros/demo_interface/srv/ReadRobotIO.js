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

class ReadRobotIORequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.io_type = null;
      this.io_index = null;
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
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ReadRobotIORequest
    // Serialize message field [io_type]
    bufferOffset = _serializer.string(obj.io_type, buffer, bufferOffset);
    // Serialize message field [io_index]
    bufferOffset = _serializer.int32(obj.io_index, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ReadRobotIORequest
    let len;
    let data = new ReadRobotIORequest(null);
    // Deserialize message field [io_type]
    data.io_type = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [io_index]
    data.io_index = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.io_type);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'demo_interface/ReadRobotIORequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a92292e8000354f152bf5ec9d99d343c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string io_type
    int32 io_index
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ReadRobotIORequest(null);
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

    return resolved;
    }
};

class ReadRobotIOResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.value = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = 0.0;
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
    // Serializes a message object of type ReadRobotIOResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [value]
    bufferOffset = _serializer.float64(obj.value, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ReadRobotIOResponse
    let len;
    let data = new ReadRobotIOResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [value]
    data.value = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    return length + 13;
  }

  static datatype() {
    // Returns string type for a service object
    return 'demo_interface/ReadRobotIOResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4578b31c1e59f9e1beb8709ba78b7124';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    float64 value
    string message
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ReadRobotIOResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = 0.0
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
  Request: ReadRobotIORequest,
  Response: ReadRobotIOResponse,
  md5sum() { return '5d10bf6c42285a91d7053eb3125941e8'; },
  datatype() { return 'demo_interface/ReadRobotIO'; }
};
