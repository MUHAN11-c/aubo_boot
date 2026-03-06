// Auto-generated. Do not edit!

// (in-package object_recognition_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let RecognizedObject = require('./RecognizedObject.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class RecognizedObjectArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.objects = null;
      this.cooccurrence = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('objects')) {
        this.objects = initObj.objects
      }
      else {
        this.objects = [];
      }
      if (initObj.hasOwnProperty('cooccurrence')) {
        this.cooccurrence = initObj.cooccurrence
      }
      else {
        this.cooccurrence = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RecognizedObjectArray
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [objects]
    // Serialize the length for message field [objects]
    bufferOffset = _serializer.uint32(obj.objects.length, buffer, bufferOffset);
    obj.objects.forEach((val) => {
      bufferOffset = RecognizedObject.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [cooccurrence]
    bufferOffset = _arraySerializer.float32(obj.cooccurrence, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RecognizedObjectArray
    let len;
    let data = new RecognizedObjectArray(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [objects]
    // Deserialize array length for message field [objects]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.objects = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.objects[i] = RecognizedObject.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [cooccurrence]
    data.cooccurrence = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.objects.forEach((val) => {
      length += RecognizedObject.getMessageSize(val);
    });
    length += 4 * object.cooccurrence.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'object_recognition_msgs/RecognizedObjectArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bad6b1546b9ebcabb49fb3b858d78964';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ##################################################### HEADER ###########################################################
    Header header
    
    # This message type describes a potential scene configuration: a set of objects that can explain the scene
    object_recognition_msgs/RecognizedObject[] objects
    
    ##################################################### SEARCH ###########################################################
    
    # The co-occurrence matrix between the recognized objects
    float32[] cooccurrence
    
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
    MSG: object_recognition_msgs/RecognizedObject
    ##################################################### HEADER ###########################################################
    
    # The header frame corresponds to the pose frame, NOT the point_cloud frame.
    Header header
    
    ################################################## OBJECT INFO #########################################################
    
    # Contains information about the type and the position of a found object
    # Some of those fields might not be filled because the used techniques do not fill them or because the user does not
    # request them
    
    # The type of the found object
    object_recognition_msgs/ObjectType type
    
    #confidence: how sure you are it is that object and not another one.
    # It is between 0 and 1 and the closer to one it is the better
    float32 confidence
    
    ################################################ OBJECT CLUSTERS #######################################################
    
    # Sometimes you can extract the 3d points that belong to the object, in the frames of the original sensors
    # (it is an array as you might have several sensors)
    sensor_msgs/PointCloud2[] point_clouds
    
    # Sometimes, you can only provide a bounding box/shape, even in 3d
    # This is in the pose frame
    shape_msgs/Mesh bounding_mesh
    
    # Sometimes, you only have 2d input so you can't really give a pose, you just get a contour, or a box
    # The last point will be linked to the first one automatically
    geometry_msgs/Point[] bounding_contours
    
    #################################################### POSE INFO #########################################################
    
    # This is the result that everybody expects : the pose in some frame given with the input. The units are radian/meters
    # as usual
    geometry_msgs/PoseWithCovarianceStamped pose
    
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
    MSG: geometry_msgs/PoseWithCovarianceStamped
    # This expresses an estimated pose with a reference coordinate frame and timestamp
    
    Header header
    PoseWithCovariance pose
    
    ================================================================================
    MSG: geometry_msgs/PoseWithCovariance
    # This represents a pose in free space with uncertainty.
    
    Pose pose
    
    # Row-major representation of the 6x6 covariance matrix
    # The orientation parameters use a fixed-axis representation.
    # In order, the parameters are:
    # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    float64[36] covariance
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
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
    const resolved = new RecognizedObjectArray(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.objects !== undefined) {
      resolved.objects = new Array(msg.objects.length);
      for (let i = 0; i < resolved.objects.length; ++i) {
        resolved.objects[i] = RecognizedObject.Resolve(msg.objects[i]);
      }
    }
    else {
      resolved.objects = []
    }

    if (msg.cooccurrence !== undefined) {
      resolved.cooccurrence = msg.cooccurrence;
    }
    else {
      resolved.cooccurrence = []
    }

    return resolved;
    }
};

module.exports = RecognizedObjectArray;
