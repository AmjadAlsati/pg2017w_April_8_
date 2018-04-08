// Auto-generated. Do not edit!

// (in-package cooperative_driving_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Direction = require('./Direction.js');

//-----------------------------------------------------------

class Directions {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.directions = null;
    }
    else {
      if (initObj.hasOwnProperty('directions')) {
        this.directions = initObj.directions
      }
      else {
        this.directions = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Directions
    // Serialize message field [directions]
    // Serialize the length for message field [directions]
    bufferOffset = _serializer.uint32(obj.directions.length, buffer, bufferOffset);
    obj.directions.forEach((val) => {
      bufferOffset = Direction.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Directions
    let len;
    let data = new Directions(null);
    // Deserialize message field [directions]
    // Deserialize array length for message field [directions]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.directions = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.directions[i] = Direction.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.directions.forEach((val) => {
      length += Direction.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cooperative_driving_msgs/Directions';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f467a95e7546b9935ac1d69c0341e385';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ## List all directions of a detected crossing
    
    # Array of all directions of a crossing 
    Direction[] directions
    ================================================================================
    MSG: cooperative_driving_msgs/Direction
    ## Define a direction of a detected crossing, i.e., a turn that can possibly be taken
    
    # Possible turn at a crossing ('straight', 'left', or 'right')
    string direction
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Directions(null);
    if (msg.directions !== undefined) {
      resolved.directions = new Array(msg.directions.length);
      for (let i = 0; i < resolved.directions.length; ++i) {
        resolved.directions[i] = Direction.Resolve(msg.directions[i]);
      }
    }
    else {
      resolved.directions = []
    }

    return resolved;
    }
};

module.exports = Directions;
