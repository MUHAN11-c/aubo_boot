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

let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class GetCurrentStateRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetCurrentStateRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetCurrentStateRequest
    let len;
    let data = new GetCurrentStateRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'demo_interface/GetCurrentStateRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetCurrentStateRequest(null);
    return resolved;
    }
};

class GetCurrentStateResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.joint_position_rad = null;
      this.cartesian_position = null;
      this.velocity = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('joint_position_rad')) {
        this.joint_position_rad = initObj.joint_position_rad
      }
      else {
        this.joint_position_rad = [];
      }
      if (initObj.hasOwnProperty('cartesian_position')) {
        this.cartesian_position = initObj.cartesian_position
      }
      else {
        this.cartesian_position = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = [];
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
    // Serializes a message object of type GetCurrentStateResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [joint_position_rad]
    bufferOffset = _arraySerializer.float64(obj.joint_position_rad, buffer, bufferOffset, null);
    // Serialize message field [cartesian_position]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.cartesian_position, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = _arraySerializer.float64(obj.velocity, buffer, bufferOffset, null);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetCurrentStateResponse
    let len;
    let data = new GetCurrentStateResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [joint_position_rad]
    data.joint_position_rad = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [cartesian_position]
    data.cartesian_position = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.joint_position_rad.length;
    length += 8 * object.velocity.length;
    length += _getByteLength(object.message);
    return length + 69;
  }

  static datatype() {
    // Returns string type for a service object
    return 'demo_interface/GetCurrentStateResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '54871f675804e280e9a4e64601214b7d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    float64[] joint_position_rad
    geometry_msgs/Pose cartesian_position
    float64[] velocity
    string message
    
    
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetCurrentStateResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.joint_position_rad !== undefined) {
      resolved.joint_position_rad = msg.joint_position_rad;
    }
    else {
      resolved.joint_position_rad = []
    }

    if (msg.cartesian_position !== undefined) {
      resolved.cartesian_position = geometry_msgs.msg.Pose.Resolve(msg.cartesian_position)
    }
    else {
      resolved.cartesian_position = new geometry_msgs.msg.Pose()
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = msg.velocity;
    }
    else {
      resolved.velocity = []
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
  Request: GetCurrentStateRequest,
  Response: GetCurrentStateResponse,
  md5sum() { return '54871f675804e280e9a4e64601214b7d'; },
  datatype() { return 'demo_interface/GetCurrentState'; }
};
