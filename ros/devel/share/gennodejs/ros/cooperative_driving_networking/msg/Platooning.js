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

class Platooning {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.message_id = null;
      this.sender_id = null;
      this.platoon_id = null;
      this.leader_id = null;
      this.robot_role = null;
      this.predecessor_id = null;
      this.maneuver = null;
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
      if (initObj.hasOwnProperty('platoon_id')) {
        this.platoon_id = initObj.platoon_id
      }
      else {
        this.platoon_id = 0;
      }
      if (initObj.hasOwnProperty('leader_id')) {
        this.leader_id = initObj.leader_id
      }
      else {
        this.leader_id = 0;
      }
      if (initObj.hasOwnProperty('robot_role')) {
        this.robot_role = initObj.robot_role
      }
      else {
        this.robot_role = '';
      }
      if (initObj.hasOwnProperty('predecessor_id')) {
        this.predecessor_id = initObj.predecessor_id
      }
      else {
        this.predecessor_id = 0;
      }
      if (initObj.hasOwnProperty('maneuver')) {
        this.maneuver = initObj.maneuver
      }
      else {
        this.maneuver = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Platooning
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [message_id]
    bufferOffset = _serializer.uint32(obj.message_id, buffer, bufferOffset);
    // Serialize message field [sender_id]
    bufferOffset = _serializer.uint8(obj.sender_id, buffer, bufferOffset);
    // Serialize message field [platoon_id]
    bufferOffset = _serializer.uint8(obj.platoon_id, buffer, bufferOffset);
    // Serialize message field [leader_id]
    bufferOffset = _serializer.uint8(obj.leader_id, buffer, bufferOffset);
    // Serialize message field [robot_role]
    bufferOffset = _serializer.string(obj.robot_role, buffer, bufferOffset);
    // Serialize message field [predecessor_id]
    bufferOffset = _serializer.uint8(obj.predecessor_id, buffer, bufferOffset);
    // Serialize message field [maneuver]
    bufferOffset = _serializer.string(obj.maneuver, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Platooning
    let len;
    let data = new Platooning(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [message_id]
    data.message_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [sender_id]
    data.sender_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [platoon_id]
    data.platoon_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [leader_id]
    data.leader_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [robot_role]
    data.robot_role = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [predecessor_id]
    data.predecessor_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [maneuver]
    data.maneuver = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.robot_role.length;
    length += object.maneuver.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cooperative_driving_networking/Platooning';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ab1ba68fe20a61da2fb08021e8187972';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Used in the demo application for the token passing protocol
    
    # Common message header
    # Standard ROS message header
    Header header
    # Applicationwise uniquie identifier of this message content
    uint32 message_id
    # The id of the robot which sends this message
    uint8 sender_id
    # Specific data of this message type
    # TODO
    uint8 platoon_id
    # TODO
    uint8 leader_id
    # TODO
    string robot_role
    # TODO
    uint8 predecessor_id
    # TODO
    string maneuver
    
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
    const resolved = new Platooning(null);
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

    if (msg.platoon_id !== undefined) {
      resolved.platoon_id = msg.platoon_id;
    }
    else {
      resolved.platoon_id = 0
    }

    if (msg.leader_id !== undefined) {
      resolved.leader_id = msg.leader_id;
    }
    else {
      resolved.leader_id = 0
    }

    if (msg.robot_role !== undefined) {
      resolved.robot_role = msg.robot_role;
    }
    else {
      resolved.robot_role = ''
    }

    if (msg.predecessor_id !== undefined) {
      resolved.predecessor_id = msg.predecessor_id;
    }
    else {
      resolved.predecessor_id = 0
    }

    if (msg.maneuver !== undefined) {
      resolved.maneuver = msg.maneuver;
    }
    else {
      resolved.maneuver = ''
    }

    return resolved;
    }
};

module.exports = Platooning;
