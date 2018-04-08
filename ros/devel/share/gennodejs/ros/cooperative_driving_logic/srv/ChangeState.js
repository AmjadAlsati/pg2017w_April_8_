// Auto-generated. Do not edit!

// (in-package cooperative_driving_logic.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class ChangeStateRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.target_state = null;
      this.tag_id_to_follow = null;
    }
    else {
      if (initObj.hasOwnProperty('target_state')) {
        this.target_state = initObj.target_state
      }
      else {
        this.target_state = '';
      }
      if (initObj.hasOwnProperty('tag_id_to_follow')) {
        this.tag_id_to_follow = initObj.tag_id_to_follow
      }
      else {
        this.tag_id_to_follow = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ChangeStateRequest
    // Serialize message field [target_state]
    bufferOffset = _serializer.string(obj.target_state, buffer, bufferOffset);
    // Serialize message field [tag_id_to_follow]
    bufferOffset = _serializer.int8(obj.tag_id_to_follow, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ChangeStateRequest
    let len;
    let data = new ChangeStateRequest(null);
    // Deserialize message field [target_state]
    data.target_state = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [tag_id_to_follow]
    data.tag_id_to_follow = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.target_state.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cooperative_driving_logic/ChangeStateRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '47b07bdab417ccb3c955c49a6d18057b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    string REMOTE_CONTROL=remote_control
    string FOLLOW_BLOB=follow_blob
    string FOLLOW_LINE=follow_line
    string PLATOONING=platooning
    string DYNAMIC_FOLLOW_LINE=dynamic_follow_line
    string IDLE=idle
    
    string target_state
    int8 tag_id_to_follow
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ChangeStateRequest(null);
    if (msg.target_state !== undefined) {
      resolved.target_state = msg.target_state;
    }
    else {
      resolved.target_state = ''
    }

    if (msg.tag_id_to_follow !== undefined) {
      resolved.tag_id_to_follow = msg.tag_id_to_follow;
    }
    else {
      resolved.tag_id_to_follow = 0
    }

    return resolved;
    }
};

// Constants for message
ChangeStateRequest.Constants = {
  REMOTE_CONTROL: 'remote_control',
  FOLLOW_BLOB: 'follow_blob',
  FOLLOW_LINE: 'follow_line',
  PLATOONING: 'platooning',
  DYNAMIC_FOLLOW_LINE: 'dynamic_follow_line',
  IDLE: 'idle',
}

class ChangeStateResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ChangeStateResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ChangeStateResponse
    let len;
    let data = new ChangeStateResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cooperative_driving_logic/ChangeStateResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ChangeStateResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: ChangeStateRequest,
  Response: ChangeStateResponse,
  md5sum() { return '47b07bdab417ccb3c955c49a6d18057b'; },
  datatype() { return 'cooperative_driving_logic/ChangeState'; }
};
