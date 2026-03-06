// Auto-generated. Do not edit!

// (in-package object_recognition_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Table {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.pose = null;
      this.convex_hull = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('convex_hull')) {
        this.convex_hull = initObj.convex_hull
      }
      else {
        this.convex_hull = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Table
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose, buffer, bufferOffset);
    // Serialize message field [convex_hull]
    // Serialize the length for message field [convex_hull]
    bufferOffset = _serializer.uint32(obj.convex_hull.length, buffer, bufferOffset);
    obj.convex_hull.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Table
    let len;
    let data = new Table(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [convex_hull]
    // Deserialize array length for message field [convex_hull]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.convex_hull = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.convex_hull[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 24 * object.convex_hull.length;
    return length + 60;
  }

  static datatype() {
    // Returns string type for a message object
    return 'object_recognition_msgs/Table';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '39efebc7d51e44bd2d72f2df6c7823a2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Informs that a planar table has been detected at a given location
    
    Header header
    
    # The pose gives you the transform that take you to the coordinate system
    # of the table, with the origin somewhere in the table plane and the 
    # z axis normal to the plane
    geometry_msgs/Pose pose
    
    # There is no guarantee that the table does NOT extend further than the
    # convex hull; this is just as far as we've observed it.
    # The origin of the table coordinate system is inside the convex hull
    
    # Set of points forming the convex hull of the table
    geometry_msgs/Point[] convex_hull
    
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
    const resolved = new Table(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.Pose.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.Pose()
    }

    if (msg.convex_hull !== undefined) {
      resolved.convex_hull = new Array(msg.convex_hull.length);
      for (let i = 0; i < resolved.convex_hull.length; ++i) {
        resolved.convex_hull[i] = geometry_msgs.msg.Point.Resolve(msg.convex_hull[i]);
      }
    }
    else {
      resolved.convex_hull = []
    }

    return resolved;
    }
};

module.exports = Table;
