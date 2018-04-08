// Auto-generated. Do not edit!

// (in-package cooperative_driving_networking.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class CooperativeAwarenessMessage {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.message_id = null;
      this.sender_id = null;
      this.velocity = null;
      this.steering = null;
      this.posx = null;
      this.posy = null;
      this.posz = null;
      this.road_id = null;
      this.lane_id = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('message_id')) {
        this.message_id = initObj.message_id
      }
      else {
        this.message_id = 0;
      }
      if (initObj.hasOwnProperty('sender_id')) {
        this.sender_id = initObj.sender_id
      }
      else {
        this.sender_id = 0;
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = 0.0;
      }
      if (initObj.hasOwnProperty('steering')) {
        this.steering = initObj.steering
      }
      else {
        this.steering = 0.0;
      }
      if (initObj.hasOwnProperty('posx')) {
        this.posx = initObj.posx
      }
      else {
        this.posx = 0.0;
      }
      if (initObj.hasOwnProperty('posy')) {
        this.posy = initObj.posy
      }
      else {
        this.posy = 0.0;
      }
      if (initObj.hasOwnProperty('posz')) {
        this.posz = initObj.posz
      }
      else {
        this.posz = 0.0;
      }
      if (initObj.hasOwnProperty('road_id')) {
        this.road_id = initObj.road_id
      }
      else {
        this.road_id = 0;
      }
      if (initObj.hasOwnProperty('lane_id')) {
        this.lane_id = initObj.lane_id
      }
      else {
        this.lane_id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CooperativeAwarenessMessage
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [message_id]
    bufferOffset = _serializer.uint32(obj.message_id, buffer, bufferOffset);
    // Serialize message field [sender_id]
    bufferOffset = _serializer.uint8(obj.sender_id, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = _serializer.float32(obj.velocity, buffer, bufferOffset);
    // Serialize message field [steering]
    bufferOffset = _serializer.float32(obj.steering, buffer, bufferOffset);
    // Serialize message field [posx]
    bufferOffset = _serializer.float32(obj.posx, buffer, bufferOffset);
    // Serialize message field [posy]
    bufferOffset = _serializer.float32(obj.posy, buffer, bufferOffset);
    // Serialize message field [posz]
    bufferOffset = _serializer.float32(obj.posz, buffer, bufferOffset);
    // Serialize message field [road_id]
    bufferOffset = _serializer.uint8(obj.road_id, buffer, bufferOffset);
    // Serialize message field [lane_id]
    bufferOffset = _serializer.uint8(obj.lane_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CooperativeAwarenessMessage
    let len;
    let data = new CooperativeAwarenessMessage(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [message_id]
    data.message_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [sender_id]
    data.sender_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [steering]
    data.steering = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [posx]
    data.posx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [posy]
    data.posy = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [posz]
    data.posz = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [road_id]
    data.road_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [lane_id]
    data.lane_id = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 27;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cooperative_driving_networking/CooperativeAwarenessMessage';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '55c0c4a0b38cbbc32116f929677739b2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Used in the application base class to represent a simple ETSI like CAMs
    
    # Common message header
    # Standard ROS message header
    Header header
    # Applicationwise uniquie identifier of this message content
    uint32 message_id
    # The id of the robot which sends this message
    uint8 sender_id
    # Specific data of this message type
    # The current velocity value of this robot
    float32 velocity
    # The current steering value of this robot
    float32 steering
    # The current x coordinate of this robot
    float32 posx
    # The current y coordinate of this robot
    float32 posy
    # The current z coordinate of this robot
    float32 posz
    # The id of the road this robot is currently driving on
    uint8 road_id
    # The id of the lane this robot is currently driving on
    uint8 lane_id
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CooperativeAwarenessMessage(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.message_id !== undefined) {
      resolved.message_id = msg.message_id;
    }
    else {
      resolved.message_id = 0
    }

    if (msg.sender_id !== undefined) {
      resolved.sender_id = msg.sender_id;
    }
    else {
      resolved.sender_id = 0
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = msg.velocity;
    }
    else {
      resolved.velocity = 0.0
    }

    if (msg.steering !== undefined) {
      resolved.steering = msg.steering;
    }
    else {
      resolved.steering = 0.0
    }

    if (msg.posx !== undefined) {
      resolved.posx = msg.posx;
    }
    else {
      resolved.posx = 0.0
    }

    if (msg.posy !== undefined) {
      resolved.posy = msg.posy;
    }
    else {
      resolved.posy = 0.0
    }

    if (msg.posz !== undefined) {
      resolved.posz = msg.posz;
    }
    else {
      resolved.posz = 0.0
    }

    if (msg.road_id !== undefined) {
      resolved.road_id = msg.road_id;
    }
    else {
      resolved.road_id = 0
    }

    if (msg.lane_id !== undefined) {
      resolved.lane_id = msg.lane_id;
    }
    else {
      resolved.lane_id = 0
    }

    return resolved;
    }
};

module.exports = CooperativeAwarenessMessage;
