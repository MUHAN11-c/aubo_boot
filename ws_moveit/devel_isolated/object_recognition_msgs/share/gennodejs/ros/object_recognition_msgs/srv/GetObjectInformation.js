// Auto-generated. Do not edit!

// (in-package object_recognition_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ObjectType = require('../msg/ObjectType.js');

//-----------------------------------------------------------

let ObjectInformation = require('../msg/ObjectInformation.js');

//-----------------------------------------------------------

class GetObjectInformationRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.type = null;
    }
    else {
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = new ObjectType();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetObjectInformationRequest
    // Serialize message field [type]
    bufferOffset = ObjectType.serialize(obj.type, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetObjectInformationRequest
    let len;
    let data = new GetObjectInformationRequest(null);
    // Deserialize message field [type]
    data.type = ObjectType.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += ObjectType.getMessageSize(object.type);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'object_recognition_msgs/GetObjectInformationRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0d72b69e80da0fe473b0bdcdd7a28d4d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Retrieve extra data from the DB for a given object
    
    # The type of the object to retrieve info from
    object_recognition_msgs/ObjectType type
    
    
    ================================================================================
    MSG: object_recognition_msgs/ObjectType
    ################################################## OBJECT ID #########################################################
    
    # Contains information about the type of a found object. Those two sets of parameters together uniquely define an
    # object
    
    # The key of the found object: the unique identifier in the given db
    string key
    
    # The db parameters stored as a JSON/compressed YAML string. An object id does not make sense without the corresponding
    # database. E.g., in object_recognition, it can look like: "{'type':'CouchDB', 'root':'http://localhost'}"
    # There is no conventional format for those parameters and it's nice to keep that flexibility.
    # The object_recognition_core as a generic DB type that can read those fields
    # Current examples:
    # For CouchDB:
    #   type: 'CouchDB'
    #   root: 'http://localhost:5984'
    #   collection: 'object_recognition'
    # For SQL household database:
    #   type: 'SqlHousehold'
    #   host: 'wgs36'
    #   port: 5432
    #   user: 'willow'
    #   password: 'willow'
    #   name: 'household_objects'
    #   module: 'tabletop'
    string db
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetObjectInformationRequest(null);
    if (msg.type !== undefined) {
      resolved.type = ObjectType.Resolve(msg.type)
    }
    else {
      resolved.type = new ObjectType()
    }

    return resolved;
    }
};

class GetObjectInformationResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.information = null;
    }
    else {
      if (initObj.hasOwnProperty('information')) {
        this.information = initObj.information
      }
      else {
        this.information = new ObjectInformation();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetObjectInformationResponse
    // Serialize message field [information]
    bufferOffset = ObjectInformation.serialize(obj.information, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetObjectInformationResponse
    let len;
    let data = new GetObjectInformationResponse(null);
    // Deserialize message field [information]
    data.information = ObjectInformation.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += ObjectInformation.getMessageSize(object.information);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'object_recognition_msgs/GetObjectInformationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a62c5d1c41e250373b3e8e912a13a9cb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    # Extra object info 
    object_recognition_msgs/ObjectInformation information
    
    
    ================================================================================
    MSG: object_recognition_msgs/ObjectInformation
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
    const resolved = new GetObjectInformationResponse(null);
    if (msg.information !== undefined) {
      resolved.information = ObjectInformation.Resolve(msg.information)
    }
    else {
      resolved.information = new ObjectInformation()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetObjectInformationRequest,
  Response: GetObjectInformationResponse,
  md5sum() { return 'dd7d344324fd86c32836f4fe1bc7b322'; },
  datatype() { return 'object_recognition_msgs/GetObjectInformation'; }
};
