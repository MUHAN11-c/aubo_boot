// Auto-generated. Do not edit!

// (in-package rqt_py_common.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Val {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.floats = null;
    }
    else {
      if (initObj.hasOwnProperty('floats')) {
        this.floats = initObj.floats
      }
      else {
        this.floats = new Array(5).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Val
    // Check that the constant length array field [floats] has the right length
    if (obj.floats.length !== 5) {
      throw new Error('Unable to serialize array field floats - length must be 5')
    }
    // Serialize message field [floats]
    bufferOffset = _arraySerializer.float64(obj.floats, buffer, bufferOffset, 5);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Val
    let len;
    let data = new Val(null);
    // Deserialize message field [floats]
    data.floats = _arrayDeserializer.float64(buffer, bufferOffset, 5)
    return data;
  }

  static getMessageSize(object) {
    return 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rqt_py_common/Val';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4ca05234743a5086bfd02946376b9ff6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[5] floats
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Val(null);
    if (msg.floats !== undefined) {
      resolved.floats = msg.floats;
    }
    else {
      resolved.floats = new Array(5).fill(0)
    }

    return resolved;
    }
};

module.exports = Val;
