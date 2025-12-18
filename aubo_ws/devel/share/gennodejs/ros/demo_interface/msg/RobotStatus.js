// Auto-generated. Do not edit!

// (in-package demo_interface.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class RobotStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.is_online = null;
      this.enable = null;
      this.in_motion = null;
      this.planning_status = null;
      this.joint_position_rad = null;
      this.joint_position_deg = null;
      this.cartesian_position = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('is_online')) {
        this.is_online = initObj.is_online
      }
      else {
        this.is_online = false;
      }
      if (initObj.hasOwnProperty('enable')) {
        this.enable = initObj.enable
      }
      else {
        this.enable = false;
      }
      if (initObj.hasOwnProperty('in_motion')) {
        this.in_motion = initObj.in_motion
      }
      else {
        this.in_motion = false;
      }
      if (initObj.hasOwnProperty('planning_status')) {
        this.planning_status = initObj.planning_status
      }
      else {
        this.planning_status = '';
      }
      if (initObj.hasOwnProperty('joint_position_rad')) {
        this.joint_position_rad = initObj.joint_position_rad
      }
      else {
        this.joint_position_rad = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('joint_position_deg')) {
        this.joint_position_deg = initObj.joint_position_deg
      }
      else {
        this.joint_position_deg = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('cartesian_position')) {
        this.cartesian_position = initObj.cartesian_position
      }
      else {
        this.cartesian_position = new geometry_msgs.msg.Pose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotStatus
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [is_online]
    bufferOffset = _serializer.bool(obj.is_online, buffer, bufferOffset);
    // Serialize message field [enable]
    bufferOffset = _serializer.bool(obj.enable, buffer, bufferOffset);
    // Serialize message field [in_motion]
    bufferOffset = _serializer.bool(obj.in_motion, buffer, bufferOffset);
    // Serialize message field [planning_status]
    bufferOffset = _serializer.string(obj.planning_status, buffer, bufferOffset);
    // Check that the constant length array field [joint_position_rad] has the right length
    if (obj.joint_position_rad.length !== 6) {
      throw new Error('Unable to serialize array field joint_position_rad - length must be 6')
    }
    // Serialize message field [joint_position_rad]
    bufferOffset = _arraySerializer.float64(obj.joint_position_rad, buffer, bufferOffset, 6);
    // Check that the constant length array field [joint_position_deg] has the right length
    if (obj.joint_position_deg.length !== 6) {
      throw new Error('Unable to serialize array field joint_position_deg - length must be 6')
    }
    // Serialize message field [joint_position_deg]
    bufferOffset = _arraySerializer.float64(obj.joint_position_deg, buffer, bufferOffset, 6);
    // Serialize message field [cartesian_position]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.cartesian_position, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotStatus
    let len;
    let data = new RobotStatus(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [is_online]
    data.is_online = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [enable]
    data.enable = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [in_motion]
    data.in_motion = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [planning_status]
    data.planning_status = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [joint_position_rad]
    data.joint_position_rad = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [joint_position_deg]
    data.joint_position_deg = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [cartesian_position]
    data.cartesian_position = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.planning_status);
    return length + 159;
  }

  static datatype() {
    // Returns string type for a message object
    return 'demo_interface/RobotStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5cf9bff7fe374641a90823ee8d3a68eb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    bool is_online
    bool enable
    bool in_motion
    string planning_status
    float64[6] joint_position_rad
    float64[6] joint_position_deg
    geometry_msgs/Pose cartesian_position
    
    
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
    const resolved = new RobotStatus(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.is_online !== undefined) {
      resolved.is_online = msg.is_online;
    }
    else {
      resolved.is_online = false
    }

    if (msg.enable !== undefined) {
      resolved.enable = msg.enable;
    }
    else {
      resolved.enable = false
    }

    if (msg.in_motion !== undefined) {
      resolved.in_motion = msg.in_motion;
    }
    else {
      resolved.in_motion = false
    }

    if (msg.planning_status !== undefined) {
      resolved.planning_status = msg.planning_status;
    }
    else {
      resolved.planning_status = ''
    }

    if (msg.joint_position_rad !== undefined) {
      resolved.joint_position_rad = msg.joint_position_rad;
    }
    else {
      resolved.joint_position_rad = new Array(6).fill(0)
    }

    if (msg.joint_position_deg !== undefined) {
      resolved.joint_position_deg = msg.joint_position_deg;
    }
    else {
      resolved.joint_position_deg = new Array(6).fill(0)
    }

    if (msg.cartesian_position !== undefined) {
      resolved.cartesian_position = geometry_msgs.msg.Pose.Resolve(msg.cartesian_position)
    }
    else {
      resolved.cartesian_position = new geometry_msgs.msg.Pose()
    }

    return resolved;
    }
};

module.exports = RobotStatus;
