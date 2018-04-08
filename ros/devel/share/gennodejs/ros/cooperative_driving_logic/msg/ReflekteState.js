// Auto-generated. Do not edit!

// (in-package cooperative_driving_logic.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ReflekteState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.state = null;
      this.node_to_follow = null;
    }
    else {
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = '';
      }
      if (initObj.hasOwnProperty('node_to_follow')) {
        this.node_to_follow = initObj.node_to_follow
      }
      else {
        this.node_to_follow = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ReflekteState
    // Serialize message field [state]
    bufferOffset = _serializer.string(obj.state, buffer, bufferOffset);
    // Serialize message field [node_to_follow]
    bufferOffset = _serializer.int8(obj.node_to_follow, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ReflekteState
    let len;
    let data = new ReflekteState(null);
    // Deserialize message field [state]
    data.state = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [node_to_follow]
    data.node_to_follow = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.state.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cooperative_driving_logic/ReflekteState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '43cc3c773585d3b826a620cbff7f3808';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Used to publish the current state of the reactive behavior in the Reflekte node
    
    # The current state of the reactive behavior
    string state
    int8 node_to_follow
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ReflekteState(null);
    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = ''
    }

    if (msg.node_to_follow !== undefined) {
      resolved.node_to_follow = msg.node_to_follow;
    }
    else {
      resolved.node_to_follow = 0
    }

    return resolved;
    }
};

module.exports = ReflekteState;
