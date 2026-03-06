// Auto-generated. Do not edit!

// (in-package object_recognition_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sensor_msgs = _finder('sensor_msgs');
let shape_msgs = _finder('shape_msgs');

//-----------------------------------------------------------

class ObjectInformation {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.name = null;
      this.ground_truth_mesh = null;
      this.ground_truth_point_cloud = null;
    }
    else {
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('ground_truth_mesh')) {
        this.ground_truth_mesh = initObj.ground_truth_mesh
      }
      else {
        this.ground_truth_mesh = new shape_msgs.msg.Mesh();
      }
      if (initObj.hasOwnProperty('ground_truth_point_cloud')) {
        this.ground_truth_point_cloud = initObj.ground_truth_point_cloud
      }
      else {
        this.ground_truth_point_cloud = new sensor_msgs.msg.PointCloud2();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ObjectInformation
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [ground_truth_mesh]
    bufferOffset = shape_msgs.msg.Mesh.serialize(obj.ground_truth_mesh, buffer, bufferOffset);
    // Serialize message field [ground_truth_point_cloud]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.ground_truth_point_cloud, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ObjectInformation
    let len;
    let data = new ObjectInformation(null);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [ground_truth_mesh]
    data.ground_truth_mesh = shape_msgs.msg.Mesh.deserialize(buffer, bufferOffset);
    // Deserialize message field [ground_truth_point_cloud]
    data.ground_truth_point_cloud = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.name);
    length += shape_msgs.msg.Mesh.getMessageSize(object.ground_truth_mesh);
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.ground_truth_point_cloud);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'object_recognition_msgs/ObjectInformation';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '921ec39f51c7b927902059cf3300ecde';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ############################################## VISUALIZATION INFO ######################################################
    ################### THIS INFO SHOULD BE OBTAINED INDEPENDENTLY FROM THE CORE, LIKE IN AN RVIZ PLUGIN ###################
    
    # The human readable name of the object
    string name
    
    # The full mesh of the object: this can be useful for display purposes, augmented reality ... but it can be big
    # Make sure the type is MESH
    shape_msgs/Mesh ground_truth_mesh
    
    # Sometimes, you only have a cloud in the DB
    # Make sure the type is POINTS
    sensor_msgs/PointCloud2 ground_truth_point_cloud
    
    ================================================================================
    MSG: shape_msgs/Mesh
    # Definition of a mesh
    
    # list of triangles; the index values refer to positions in vertices[]
    MeshTriangle[] triangles
    
    # the actual vertices that make up the mesh
    geometry_msgs/Point[] vertices
    
    ================================================================================
    MSG: shape_msgs/MeshTriangle
    # Definition of a triangle's vertices
    uint32[3] vertex_indices
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: sensor_msgs/PointCloud2
    # This message holds a collection of N-dimensional points, which may
    # contain additional information such as normals, intensity, etc. The
    # point data is stored as a binary blob, its layout described by the
    # contents of the "fields" array.
    
    # The point cloud data may be organized 2d (image-like) or 1d
    # (unordered). Point clouds organized as 2d images may be produced by
    # camera depth sensors such as stereo or time-of-flight.
    
    # Time of sensor data acquisition, and the coordinate frame ID (for 3d
    # points).
    Header header
    
    # 2D structure of the point cloud. If the cloud is unordered, height is
    # 1 and width is the length of the point cloud.
    uint32 height
    uint32 width
    
    # Describes the channels and their layout in the binary data blob.
    PointField[] fields
    
    bool    is_bigendian # Is this data bigendian?
    uint32  point_step   # Length of a point in bytes
    uint32  row_step     # Length of a row in bytes
    uint8[] data         # Actual point data, size is (row_step*height)
    
    bool is_dense        # True if there are no invalid points
    
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
    MSG: sensor_msgs/PointField
    # This message holds the description of one point entry in the
    # PointCloud2 message format.
    uint8 INT8    = 1
    uint8 UINT8   = 2
    uint8 INT16   = 3
    uint8 UINT16  = 4
    uint8 INT32   = 5
    uint8 UINT32  = 6
    uint8 FLOAT32 = 7
    uint8 FLOAT64 = 8
    
    string name      # Name of field
    uint32 offset    # Offset from start of point struct
    uint8  datatype  # Datatype enumeration, see above
    uint32 count     # How many elements in the field
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ObjectInformation(null);
    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.ground_truth_mesh !== undefined) {
      resolved.ground_truth_mesh = shape_msgs.msg.Mesh.Resolve(msg.ground_truth_mesh)
    }
    else {
      resolved.ground_truth_mesh = new shape_msgs.msg.Mesh()
    }

    if (msg.ground_truth_point_cloud !== undefined) {
      resolved.ground_truth_point_cloud = sensor_msgs.msg.PointCloud2.Resolve(msg.ground_truth_point_cloud)
    }
    else {
      resolved.ground_truth_point_cloud = new sensor_msgs.msg.PointCloud2()
    }

    return resolved;
    }
};

module.exports = ObjectInformation;
