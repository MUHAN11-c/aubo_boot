// Auto-generated. Do not edit!

// (in-package object_recognition_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ObjectType = require('./ObjectType.js');
let geometry_msgs = _finder('geometry_msgs');
let shape_msgs = _finder('shape_msgs');
let std_msgs = _finder('std_msgs');
let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------

class RecognizedObject {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.type = null;
      this.confidence = null;
      this.point_clouds = null;
      this.bounding_mesh = null;
      this.bounding_contours = null;
      this.pose = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = new ObjectType();
      }
      if (initObj.hasOwnProperty('confidence')) {
        this.confidence = initObj.confidence
      }
      else {
        this.confidence = 0.0;
      }
      if (initObj.hasOwnProperty('point_clouds')) {
        this.point_clouds = initObj.point_clouds
      }
      else {
        this.point_clouds = [];
      }
      if (initObj.hasOwnProperty('bounding_mesh')) {
        this.bounding_mesh = initObj.bounding_mesh
      }
      else {
        this.bounding_mesh = new shape_msgs.msg.Mesh();
      }
      if (initObj.hasOwnProperty('bounding_contours')) {
        this.bounding_contours = initObj.bounding_contours
      }
      else {
        this.bounding_contours = [];
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.PoseWithCovarianceStamped();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RecognizedObject
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [type]
    bufferOffset = ObjectType.serialize(obj.type, buffer, bufferOffset);
    // Serialize message field [confidence]
    bufferOffset = _serializer.float32(obj.confidence, buffer, bufferOffset);
    // Serialize message field [point_clouds]
    // Serialize the length for message field [point_clouds]
    bufferOffset = _serializer.uint32(obj.point_clouds.length, buffer, bufferOffset);
    obj.point_clouds.forEach((val) => {
      bufferOffset = sensor_msgs.msg.PointCloud2.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [bounding_mesh]
    bufferOffset = shape_msgs.msg.Mesh.serialize(obj.bounding_mesh, buffer, bufferOffset);
    // Serialize message field [bounding_contours]
    // Serialize the length for message field [bounding_contours]
    bufferOffset = _serializer.uint32(obj.bounding_contours.length, buffer, bufferOffset);
    obj.bounding_contours.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.PoseWithCovarianceStamped.serialize(obj.pose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RecognizedObject
    let len;
    let data = new RecognizedObject(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [type]
    data.type = ObjectType.deserialize(buffer, bufferOffset);
    // Deserialize message field [confidence]
    data.confidence = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [point_clouds]
    // Deserialize array length for message field [point_clouds]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.point_clouds = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.point_clouds[i] = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [bounding_mesh]
    data.bounding_mesh = shape_msgs.msg.Mesh.deserialize(buffer, bufferOffset);
    // Deserialize message field [bounding_contours]
    // Deserialize array length for message field [bounding_contours]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.bounding_contours = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.bounding_contours[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.PoseWithCovarianceStamped.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += ObjectType.getMessageSize(object.type);
    object.point_clouds.forEach((val) => {
      length += sensor_msgs.msg.PointCloud2.getMessageSize(val);
    });
    length += shape_msgs.msg.Mesh.getMessageSize(object.bounding_mesh);
    length += 24 * object.bounding_contours.length;
    length += geometry_msgs.msg.PoseWithCovarianceStamped.getMessageSize(object.pose);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'object_recognition_msgs/RecognizedObject';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f92c4cb29ba11f26c5f7219de97e900d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new RecognizedObject(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.type !== undefined) {
      resolved.type = ObjectType.Resolve(msg.type)
    }
    else {
      resolved.type = new ObjectType()
    }

    if (msg.confidence !== undefined) {
      resolved.confidence = msg.confidence;
    }
    else {
      resolved.confidence = 0.0
    }

    if (msg.point_clouds !== undefined) {
      resolved.point_clouds = new Array(msg.point_clouds.length);
      for (let i = 0; i < resolved.point_clouds.length; ++i) {
        resolved.point_clouds[i] = sensor_msgs.msg.PointCloud2.Resolve(msg.point_clouds[i]);
      }
    }
    else {
      resolved.point_clouds = []
    }

    if (msg.bounding_mesh !== undefined) {
      resolved.bounding_mesh = shape_msgs.msg.Mesh.Resolve(msg.bounding_mesh)
    }
    else {
      resolved.bounding_mesh = new shape_msgs.msg.Mesh()
    }

    if (msg.bounding_contours !== undefined) {
      resolved.bounding_contours = new Array(msg.bounding_contours.length);
      for (let i = 0; i < resolved.bounding_contours.length; ++i) {
        resolved.bounding_contours[i] = geometry_msgs.msg.Point.Resolve(msg.bounding_contours[i]);
      }
    }
    else {
      resolved.bounding_contours = []
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.PoseWithCovarianceStamped.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.PoseWithCovarianceStamped()
    }

    return resolved;
    }
};

module.exports = RecognizedObject;
