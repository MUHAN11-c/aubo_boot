// Auto-generated. Do not edit!

// (in-package demo_interface.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class MoveToPoseRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.target_pose = null;
      this.use_joints = null;
      this.velocity_factor = null;
      this.acceleration_factor = null;
    }
    else {
      if (initObj.hasOwnProperty('target_pose')) {
        this.target_pose = initObj.target_pose
      }
      else {
        this.target_pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('use_joints')) {
        this.use_joints = initObj.use_joints
      }
      else {
        this.use_joints = false;
      }
      if (initObj.hasOwnProperty('velocity_factor')) {
        this.velocity_factor = initObj.velocity_factor
      }
      else {
        this.velocity_factor = 0.0;
      }
      if (initObj.hasOwnProperty('acceleration_factor')) {
        this.acceleration_factor = initObj.acceleration_factor
      }
      else {
        this.acceleration_factor = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MoveToPoseRequest
    // Serialize message field [target_pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.target_pose, buffer, bufferOffset);
    // Serialize message field [use_joints]
    bufferOffset = _serializer.bool(obj.use_joints, buffer, bufferOffset);
    // Serialize message field [velocity_factor]
    bufferOffset = _serializer.float32(obj.velocity_factor, buffer, bufferOffset);
    // Serialize message field [acceleration_factor]
    bufferOffset = _serializer.float32(obj.acceleration_factor, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveToPoseRequest
    let len;
    let data = new MoveToPoseRequest(null);
    // Deserialize message field [target_pose]
    data.target_pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [use_joints]
    data.use_joints = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [velocity_factor]
    data.velocity_factor = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [acceleration_factor]
    data.acceleration_factor = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 65;
  }

  static datatype() {
    // Returns string type for a service object
    return 'demo_interface/MoveToPoseRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '00a28e9e2a6c5b4a00ef87a9fb467542';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Pose target_pose
    bool use_joints
    float32 velocity_factor
    float32 acceleration_factor
    
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
    const resolved = new MoveToPoseRequest(null);
    if (msg.target_pose !== undefined) {
      resolved.target_pose = geometry_msgs.msg.Pose.Resolve(msg.target_pose)
    }
    else {
      resolved.target_pose = new geometry_msgs.msg.Pose()
    }

    if (msg.use_joints !== undefined) {
      resolved.use_joints = msg.use_joints;
    }
    else {
      resolved.use_joints = false
    }

    if (msg.velocity_factor !== undefined) {
      resolved.velocity_factor = msg.velocity_factor;
    }
    else {
      resolved.velocity_factor = 0.0
    }

    if (msg.acceleration_factor !== undefined) {
      resolved.acceleration_factor = msg.acceleration_factor;
    }
    else {
      resolved.acceleration_factor = 0.0
    }

    return resolved;
    }
};

class MoveToPoseResponse {
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
    // Serializes a message object of type MoveToPoseResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [error_code]
    bufferOffset = _serializer.int32(obj.error_code, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveToPoseResponse
    let len;
    let data = new MoveToPoseResponse(null);
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
    return 'demo_interface/MoveToPoseResponse';
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
    const resolved = new MoveToPoseResponse(null);
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
  Request: MoveToPoseRequest,
  Response: MoveToPoseResponse,
  md5sum() { return '13d651572c46e71643945d975979b6fc'; },
  datatype() { return 'demo_interface/MoveToPose'; }
};
