// Auto-generated. Do not edit!

// (in-package cooperative_driving_vision.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Region = require('./Region.js');
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Features {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.image_width = null;
      this.image_height = null;
      this.Hlines = null;
      this.Vlines = null;
      this.PFPS = null;
      this.regions = null;
      this.box_width = null;
      this.box_height = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('image_width')) {
        this.image_width = initObj.image_width
      }
      else {
        this.image_width = 0;
      }
      if (initObj.hasOwnProperty('image_height')) {
        this.image_height = initObj.image_height
      }
      else {
        this.image_height = 0;
      }
      if (initObj.hasOwnProperty('Hlines')) {
        this.Hlines = initObj.Hlines
      }
      else {
        this.Hlines = [];
      }
      if (initObj.hasOwnProperty('Vlines')) {
        this.Vlines = initObj.Vlines
      }
      else {
        this.Vlines = [];
      }
      if (initObj.hasOwnProperty('PFPS')) {
        this.PFPS = initObj.PFPS
      }
      else {
        this.PFPS = 0;
      }
      if (initObj.hasOwnProperty('regions')) {
        this.regions = initObj.regions
      }
      else {
        this.regions = [];
      }
      if (initObj.hasOwnProperty('box_width')) {
        this.box_width = initObj.box_width
      }
      else {
        this.box_width = 0;
      }
      if (initObj.hasOwnProperty('box_height')) {
        this.box_height = initObj.box_height
      }
      else {
        this.box_height = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Features
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [image_width]
    bufferOffset = _serializer.uint16(obj.image_width, buffer, bufferOffset);
    // Serialize message field [image_height]
    bufferOffset = _serializer.uint16(obj.image_height, buffer, bufferOffset);
    // Serialize message field [Hlines]
    // Serialize the length for message field [Hlines]
    bufferOffset = _serializer.uint32(obj.Hlines.length, buffer, bufferOffset);
    obj.Hlines.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [Vlines]
    // Serialize the length for message field [Vlines]
    bufferOffset = _serializer.uint32(obj.Vlines.length, buffer, bufferOffset);
    obj.Vlines.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [PFPS]
    bufferOffset = _serializer.uint16(obj.PFPS, buffer, bufferOffset);
    // Serialize message field [regions]
    // Serialize the length for message field [regions]
    bufferOffset = _serializer.uint32(obj.regions.length, buffer, bufferOffset);
    obj.regions.forEach((val) => {
      bufferOffset = Region.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [box_width]
    bufferOffset = _serializer.uint16(obj.box_width, buffer, bufferOffset);
    // Serialize message field [box_height]
    bufferOffset = _serializer.uint16(obj.box_height, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Features
    let len;
    let data = new Features(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [image_width]
    data.image_width = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [image_height]
    data.image_height = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [Hlines]
    // Deserialize array length for message field [Hlines]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.Hlines = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.Hlines[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [Vlines]
    // Deserialize array length for message field [Vlines]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.Vlines = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.Vlines[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [PFPS]
    data.PFPS = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [regions]
    // Deserialize array length for message field [regions]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.regions = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.regions[i] = Region.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [box_width]
    data.box_width = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [box_height]
    data.box_height = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 24 * object.Hlines.length;
    length += 24 * object.Vlines.length;
    length += 56 * object.regions.length;
    return length + 22;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cooperative_driving_vision/Features';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b06b1ffd262cf7d0705951b2a3148fc6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Exchange format for extracted features
    
    # Standard ROS message header
    Header header
    # Width of the image the features where extracted from
    uint16 image_width
    # Height of the image the features where extracted from
    uint16 image_height
    # List of points in the image at which a vertical line
    # (the lane marker) has been extracted. The list is ordered
    # inversely by the y-coordinate, i.e. from bottom of the
    # image to the top.
    geometry_msgs/Point[] Hlines # z-coordinate unused
    geometry_msgs/Point[] Vlines
    # The processed frames per second
    uint16 PFPS
    # List of colored regions found in the image
    Region[] regions
    uint16 box_width
    uint16 box_height
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
    # 0: no frame
    # 1: global frame
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: cooperative_driving_vision/Region
    # Custom message type to represent an extracted region used for publishing
    
    # The region's average color
    std_msgs/ColorRGBA color
    # The geometrical moment describing the extents of the region
    Moment moment
    
    ================================================================================
    MSG: std_msgs/ColorRGBA
    float32 r
    float32 g
    float32 b
    float32 a
    
    ================================================================================
    MSG: cooperative_driving_vision/Moment
    # Custom message type to represent a geometrical moment
    
    # (0, 0)th raw moment, i.e. number of pixels
    float32 m00
    # (1, 0)th raw moment
    float32 m10
    # (0, 1)th raw moment
    float32 m01
    # (1, 1)th raw moment
    float32 m11
    # (2, 0)th raw moment
    float32 m20
    # (0, 2)th raw moment
    float32 m02
    # (2, 1)th raw moment
    float32 m21
    # (1, 2)th raw moment
    float32 m12
    # (3, 0)th raw moment
    float32 m30
    # (0, 3)th raw moment
    float32 m03
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Features(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.image_width !== undefined) {
      resolved.image_width = msg.image_width;
    }
    else {
      resolved.image_width = 0
    }

    if (msg.image_height !== undefined) {
      resolved.image_height = msg.image_height;
    }
    else {
      resolved.image_height = 0
    }

    if (msg.Hlines !== undefined) {
      resolved.Hlines = new Array(msg.Hlines.length);
      for (let i = 0; i < resolved.Hlines.length; ++i) {
        resolved.Hlines[i] = geometry_msgs.msg.Point.Resolve(msg.Hlines[i]);
      }
    }
    else {
      resolved.Hlines = []
    }

    if (msg.Vlines !== undefined) {
      resolved.Vlines = new Array(msg.Vlines.length);
      for (let i = 0; i < resolved.Vlines.length; ++i) {
        resolved.Vlines[i] = geometry_msgs.msg.Point.Resolve(msg.Vlines[i]);
      }
    }
    else {
      resolved.Vlines = []
    }

    if (msg.PFPS !== undefined) {
      resolved.PFPS = msg.PFPS;
    }
    else {
      resolved.PFPS = 0
    }

    if (msg.regions !== undefined) {
      resolved.regions = new Array(msg.regions.length);
      for (let i = 0; i < resolved.regions.length; ++i) {
        resolved.regions[i] = Region.Resolve(msg.regions[i]);
      }
    }
    else {
      resolved.regions = []
    }

    if (msg.box_width !== undefined) {
      resolved.box_width = msg.box_width;
    }
    else {
      resolved.box_width = 0
    }

    if (msg.box_height !== undefined) {
      resolved.box_height = msg.box_height;
    }
    else {
      resolved.box_height = 0
    }

    return resolved;
    }
};

module.exports = Features;
