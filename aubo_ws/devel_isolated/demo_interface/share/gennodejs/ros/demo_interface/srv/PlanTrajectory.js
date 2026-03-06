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

let trajectory_msgs = _finder('trajectory_msgs');

//-----------------------------------------------------------

class PlanTrajectoryRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.target_pose = null;
      this.use_joints = null;
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
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PlanTrajectoryRequest
    // Serialize message field [target_pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.target_pose, buffer, bufferOffset);
    // Serialize message field [use_joints]
    bufferOffset = _serializer.bool(obj.use_joints, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlanTrajectoryRequest
    let len;
    let data = new PlanTrajectoryRequest(null);
    // Deserialize message field [target_pose]
    data.target_pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [use_joints]
    data.use_joints = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 57;
  }

  static datatype() {
    // Returns string type for a service object
    return 'demo_interface/PlanTrajectoryRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '37144d5bbdb0e17dcf68bcb5b6275b64';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Pose target_pose
    bool use_joints
    
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
    const resolved = new PlanTrajectoryRequest(null);
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

    return resolved;
    }
};

class PlanTrajectoryResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.trajectory = null;
      this.planning_time = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('trajectory')) {
        this.trajectory = initObj.trajectory
      }
      else {
        this.trajectory = new trajectory_msgs.msg.JointTrajectory();
      }
      if (initObj.hasOwnProperty('planning_time')) {
        this.planning_time = initObj.planning_time
      }
      else {
        this.planning_time = 0.0;
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
    // Serializes a message object of type PlanTrajectoryResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [trajectory]
    bufferOffset = trajectory_msgs.msg.JointTrajectory.serialize(obj.trajectory, buffer, bufferOffset);
    // Serialize message field [planning_time]
    bufferOffset = _serializer.float32(obj.planning_time, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlanTrajectoryResponse
    let len;
    let data = new PlanTrajectoryResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [trajectory]
    data.trajectory = trajectory_msgs.msg.JointTrajectory.deserialize(buffer, bufferOffset);
    // Deserialize message field [planning_time]
    data.planning_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += trajectory_msgs.msg.JointTrajectory.getMessageSize(object.trajectory);
    length += _getByteLength(object.message);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'demo_interface/PlanTrajectoryResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9b473b52d160d5a4af175e589436f196';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    trajectory_msgs/JointTrajectory trajectory
    float32 planning_time
    string message
    
    
    
    ================================================================================
    MSG: trajectory_msgs/JointTrajectory
    Header header
    string[] joint_names
    JointTrajectoryPoint[] points
    
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
    MSG: trajectory_msgs/JointTrajectoryPoint
    # Each trajectory point specifies either positions[, velocities[, accelerations]]
    # or positions[, effort] for the trajectory to be executed.
    # All specified values are in the same order as the joint names in JointTrajectory.msg
    
    float64[] positions
    float64[] velocities
    float64[] accelerations
    float64[] effort
    duration time_from_start
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PlanTrajectoryResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.trajectory !== undefined) {
      resolved.trajectory = trajectory_msgs.msg.JointTrajectory.Resolve(msg.trajectory)
    }
    else {
      resolved.trajectory = new trajectory_msgs.msg.JointTrajectory()
    }

    if (msg.planning_time !== undefined) {
      resolved.planning_time = msg.planning_time;
    }
    else {
      resolved.planning_time = 0.0
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
  Request: PlanTrajectoryRequest,
  Response: PlanTrajectoryResponse,
  md5sum() { return '07df15be4b07a5a918366cc9c76a3ccd'; },
  datatype() { return 'demo_interface/PlanTrajectory'; }
};
