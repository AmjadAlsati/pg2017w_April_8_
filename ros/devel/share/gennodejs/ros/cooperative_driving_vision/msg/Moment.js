// Auto-generated. Do not edit!

// (in-package cooperative_driving_vision.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Moment {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.m00 = null;
      this.m10 = null;
      this.m01 = null;
      this.m11 = null;
      this.m20 = null;
      this.m02 = null;
      this.m21 = null;
      this.m12 = null;
      this.m30 = null;
      this.m03 = null;
    }
    else {
      if (initObj.hasOwnProperty('m00')) {
        this.m00 = initObj.m00
      }
      else {
        this.m00 = 0.0;
      }
      if (initObj.hasOwnProperty('m10')) {
        this.m10 = initObj.m10
      }
      else {
        this.m10 = 0.0;
      }
      if (initObj.hasOwnProperty('m01')) {
        this.m01 = initObj.m01
      }
      else {
        this.m01 = 0.0;
      }
      if (initObj.hasOwnProperty('m11')) {
        this.m11 = initObj.m11
      }
      else {
        this.m11 = 0.0;
      }
      if (initObj.hasOwnProperty('m20')) {
        this.m20 = initObj.m20
      }
      else {
        this.m20 = 0.0;
      }
      if (initObj.hasOwnProperty('m02')) {
        this.m02 = initObj.m02
      }
      else {
        this.m02 = 0.0;
      }
      if (initObj.hasOwnProperty('m21')) {
        this.m21 = initObj.m21
      }
      else {
        this.m21 = 0.0;
      }
      if (initObj.hasOwnProperty('m12')) {
        this.m12 = initObj.m12
      }
      else {
        this.m12 = 0.0;
      }
      if (initObj.hasOwnProperty('m30')) {
        this.m30 = initObj.m30
      }
      else {
        this.m30 = 0.0;
      }
      if (initObj.hasOwnProperty('m03')) {
        this.m03 = initObj.m03
      }
      else {
        this.m03 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Moment
    // Serialize message field [m00]
    bufferOffset = _serializer.float32(obj.m00, buffer, bufferOffset);
    // Serialize message field [m10]
    bufferOffset = _serializer.float32(obj.m10, buffer, bufferOffset);
    // Serialize message field [m01]
    bufferOffset = _serializer.float32(obj.m01, buffer, bufferOffset);
    // Serialize message field [m11]
    bufferOffset = _serializer.float32(obj.m11, buffer, bufferOffset);
    // Serialize message field [m20]
    bufferOffset = _serializer.float32(obj.m20, buffer, bufferOffset);
    // Serialize message field [m02]
    bufferOffset = _serializer.float32(obj.m02, buffer, bufferOffset);
    // Serialize message field [m21]
    bufferOffset = _serializer.float32(obj.m21, buffer, bufferOffset);
    // Serialize message field [m12]
    bufferOffset = _serializer.float32(obj.m12, buffer, bufferOffset);
    // Serialize message field [m30]
    bufferOffset = _serializer.float32(obj.m30, buffer, bufferOffset);
    // Serialize message field [m03]
    bufferOffset = _serializer.float32(obj.m03, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Moment
    let len;
    let data = new Moment(null);
    // Deserialize message field [m00]
    data.m00 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [m10]
    data.m10 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [m01]
    data.m01 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [m11]
    data.m11 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [m20]
    data.m20 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [m02]
    data.m02 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [m21]
    data.m21 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [m12]
    data.m12 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [m30]
    data.m30 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [m03]
    data.m03 = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cooperative_driving_vision/Moment';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '15ca49fd130f761e715cfaf6f1985ad7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new Moment(null);
    if (msg.m00 !== undefined) {
      resolved.m00 = msg.m00;
    }
    else {
      resolved.m00 = 0.0
    }

    if (msg.m10 !== undefined) {
      resolved.m10 = msg.m10;
    }
    else {
      resolved.m10 = 0.0
    }

    if (msg.m01 !== undefined) {
      resolved.m01 = msg.m01;
    }
    else {
      resolved.m01 = 0.0
    }

    if (msg.m11 !== undefined) {
      resolved.m11 = msg.m11;
    }
    else {
      resolved.m11 = 0.0
    }

    if (msg.m20 !== undefined) {
      resolved.m20 = msg.m20;
    }
    else {
      resolved.m20 = 0.0
    }

    if (msg.m02 !== undefined) {
      resolved.m02 = msg.m02;
    }
    else {
      resolved.m02 = 0.0
    }

    if (msg.m21 !== undefined) {
      resolved.m21 = msg.m21;
    }
    else {
      resolved.m21 = 0.0
    }

    if (msg.m12 !== undefined) {
      resolved.m12 = msg.m12;
    }
    else {
      resolved.m12 = 0.0
    }

    if (msg.m30 !== undefined) {
      resolved.m30 = msg.m30;
    }
    else {
      resolved.m30 = 0.0
    }

    if (msg.m03 !== undefined) {
      resolved.m03 = msg.m03;
    }
    else {
      resolved.m03 = 0.0
    }

    return resolved;
    }
};

module.exports = Moment;
