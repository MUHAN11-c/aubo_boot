// Auto-generated. Do not edit!

// (in-package rqt_py_common.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Val = require('./Val.js');

//-----------------------------------------------------------

class ArrayVal {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.vals = null;
    }
    else {
      if (initObj.hasOwnProperty('vals')) {
        this.vals = initObj.vals
      }
      else {
        this.vals = new Array(5).fill(new Val());
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ArrayVal
    // Check that the constant length array field [vals] has the right length
    if (obj.vals.length !== 5) {
      throw new Error('Unable to serialize array field vals - length must be 5')
    }
    // Serialize message field [vals]
    obj.vals.forEach((val) => {
      bufferOffset = Val.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ArrayVal
    let len;
    let data = new ArrayVal(null);
    // Deserialize message field [vals]
    len = 5;
    data.vals = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.vals[i] = Val.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    return 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rqt_py_common/ArrayVal';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e8748d44f2f20aabca0c342b053289ff';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Val[5] vals
    
    ================================================================================
    MSG: rqt_py_common/Val
    float64[5] floats
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ArrayVal(null);
    if (msg.vals !== undefined) {
      resolved.vals = new Array(5)
      for (let i = 0; i < resolved.vals.length; ++i) {
        if (msg.vals.length > i) {
          resolved.vals[i] = Val.Resolve(msg.vals[i]);
        }
        else {
          resolved.vals[i] = new Val();
        }
      }
    }
    else {
      resolved.vals = new Array(5).fill(new Val())
    }

    return resolved;
    }
};

module.exports = ArrayVal;
