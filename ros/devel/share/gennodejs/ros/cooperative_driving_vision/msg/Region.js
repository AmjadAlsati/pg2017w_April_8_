// Auto-generated. Do not edit!

// (in-package cooperative_driving_vision.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Moment = require('./Moment.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Region {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.color = null;
      this.moment = null;
    }
    else {
      if (initObj.hasOwnProperty('color')) {
        this.color = initObj.color
      }
      else {
        this.color = new std_msgs.msg.ColorRGBA();
      }
      if (initObj.hasOwnProperty('moment')) {
        this.moment = initObj.moment
      }
      else {
        this.moment = new Moment();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Region
    // Serialize message field [color]
    bufferOffset = std_msgs.msg.ColorRGBA.serialize(obj.color, buffer, bufferOffset);
    // Serialize message field [moment]
    bufferOffset = Moment.serialize(obj.moment, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Region
    let len;
    let data = new Region(null);
    // Deserialize message field [color]
    data.color = std_msgs.msg.ColorRGBA.deserialize(buffer, bufferOffset);
    // Deserialize message field [moment]
    data.moment = Moment.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 56;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cooperative_driving_vision/Region';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b981b501a8a1ad1a59b87231a82a1888';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new Region(null);
    if (msg.color !== undefined) {
      resolved.color = std_msgs.msg.ColorRGBA.Resolve(msg.color)
    }
    else {
      resolved.color = new std_msgs.msg.ColorRGBA()
    }

    if (msg.moment !== undefined) {
      resolved.moment = Moment.Resolve(msg.moment)
    }
    else {
      resolved.moment = new Moment()
    }

    return resolved;
    }
};

module.exports = Region;
