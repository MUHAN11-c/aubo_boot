// Auto-generated. Do not edit!

// (in-package graph_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Edges = require('./Edges.js');
let std_msgs = _finder('std_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class GeometryGraph {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.nodes = null;
      this.edges = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('nodes')) {
        this.nodes = initObj.nodes
      }
      else {
        this.nodes = [];
      }
      if (initObj.hasOwnProperty('edges')) {
        this.edges = initObj.edges
      }
      else {
        this.edges = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GeometryGraph
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [nodes]
    // Serialize the length for message field [nodes]
    bufferOffset = _serializer.uint32(obj.nodes.length, buffer, bufferOffset);
    obj.nodes.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [edges]
    // Serialize the length for message field [edges]
    bufferOffset = _serializer.uint32(obj.edges.length, buffer, bufferOffset);
    obj.edges.forEach((val) => {
      bufferOffset = Edges.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GeometryGraph
    let len;
    let data = new GeometryGraph(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [nodes]
    // Deserialize array length for message field [nodes]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.nodes = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.nodes[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [edges]
    // Deserialize array length for message field [edges]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.edges = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.edges[i] = Edges.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 24 * object.nodes.length;
    object.edges.forEach((val) => {
      length += Edges.getMessageSize(val);
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'graph_msgs/GeometryGraph';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '78739617daca94d38915923795eada2d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # A reference coordinate frame and timestamp
    Header header
    
    # 3D spacial graph
    geometry_msgs/Point[] nodes
    
    # This vector should be the same length as nodes, above, and represents all the connecting nodes
    Edges[] edges
    
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
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: graph_msgs/Edges
    #base-zero index of all the verticies this particular vertice connects to (edges)
    uint32[] node_ids
    
    # optional cost/weight of each edge. if vector is empty assume all weights are equal (1)
    float64[] weights
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GeometryGraph(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.nodes !== undefined) {
      resolved.nodes = new Array(msg.nodes.length);
      for (let i = 0; i < resolved.nodes.length; ++i) {
        resolved.nodes[i] = geometry_msgs.msg.Point.Resolve(msg.nodes[i]);
      }
    }
    else {
      resolved.nodes = []
    }

    if (msg.edges !== undefined) {
      resolved.edges = new Array(msg.edges.length);
      for (let i = 0; i < resolved.edges.length; ++i) {
        resolved.edges[i] = Edges.Resolve(msg.edges[i]);
      }
    }
    else {
      resolved.edges = []
    }

    return resolved;
    }
};

module.exports = GeometryGraph;
