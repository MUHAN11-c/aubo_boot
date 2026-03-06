// Auto-generated. Do not edit!

// (in-package graph_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Edges {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.node_ids = null;
      this.weights = null;
    }
    else {
      if (initObj.hasOwnProperty('node_ids')) {
        this.node_ids = initObj.node_ids
      }
      else {
        this.node_ids = [];
      }
      if (initObj.hasOwnProperty('weights')) {
        this.weights = initObj.weights
      }
      else {
        this.weights = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Edges
    // Serialize message field [node_ids]
    bufferOffset = _arraySerializer.uint32(obj.node_ids, buffer, bufferOffset, null);
    // Serialize message field [weights]
    bufferOffset = _arraySerializer.float64(obj.weights, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Edges
    let len;
    let data = new Edges(null);
    // Deserialize message field [node_ids]
    data.node_ids = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    // Deserialize message field [weights]
    data.weights = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.node_ids.length;
    length += 8 * object.weights.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'graph_msgs/Edges';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1dcd54afd0b0c0fbebeb59dbdda4c026';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new Edges(null);
    if (msg.node_ids !== undefined) {
      resolved.node_ids = msg.node_ids;
    }
    else {
      resolved.node_ids = []
    }

    if (msg.weights !== undefined) {
      resolved.weights = msg.weights;
    }
    else {
      resolved.weights = []
    }

    return resolved;
    }
};

module.exports = Edges;
