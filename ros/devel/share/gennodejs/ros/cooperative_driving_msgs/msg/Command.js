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
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Command {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.message_id = null;
      this.sender_id = null;
      this.command = null;
      this.turn = null;
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
      if (initObj.hasOwnProperty('command')) {
        this.command = initObj.command
      }
      else {
        this.command = '';
      }
      if (initObj.hasOwnProperty('turn')) {
        this.turn = initObj.turn
      }
      else {
        this.turn = new Direction();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Command
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [message_id]
    bufferOffset = _serializer.uint32(obj.message_id, buffer, bufferOffset);
    // Serialize message field [sender_id]
    bufferOffset = _serializer.uint8(obj.sender_id, buffer, bufferOffset);
    // Serialize message field [command]
    bufferOffset = _serializer.string(obj.command, buffer, bufferOffset);
    // Serialize message field [turn]
    bufferOffset = Direction.serialize(obj.turn, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Command
    let len;
    let data = new Command(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [message_id]
    data.message_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [sender_id]
    data.sender_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [command]
    data.command = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [turn]
    data.turn = Direction.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.command.length;
    length += Direction.getMessageSize(object.turn);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cooperative_driving_msgs/Command';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a2cc60514e9f00b5f108e5118c24aaa9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ## Command a direction change at an upcoming crossing
    # Common message header
    # Standard ROS message header
    Header header
    # Define the direction to take at the upcoming crossing
    uint32 message_id
    # The id of the robot which sends this message
    uint8 sender_id
    # Specific data of this message type
    
    # Commnand to be executed
    string command
    # Parameters for the command
    
    # Parameter for turn command
    Direction turn
    
    #TODO: Add other command parameters here
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
    const resolved = new Command(null);
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

    if (msg.command !== undefined) {
      resolved.command = msg.command;
    }
    else {
      resolved.command = ''
    }

    if (msg.turn !== undefined) {
      resolved.turn = Direction.Resolve(msg.turn)
    }
    else {
      resolved.turn = new Direction()
    }

    return resolved;
    }
};

module.exports = Command;
