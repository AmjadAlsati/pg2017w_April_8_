# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from cooperative_driving_networking/CooperativeAwarenessMessage.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class CooperativeAwarenessMessage(genpy.Message):
  _md5sum = "55c0c4a0b38cbbc32116f929677739b2"
  _type = "cooperative_driving_networking/CooperativeAwarenessMessage"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """# Used in the application base class to represent a simple ETSI like CAMs

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
"""
  __slots__ = ['header','message_id','sender_id','velocity','steering','posx','posy','posz','road_id','lane_id']
  _slot_types = ['std_msgs/Header','uint32','uint8','float32','float32','float32','float32','float32','uint8','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,message_id,sender_id,velocity,steering,posx,posy,posz,road_id,lane_id

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(CooperativeAwarenessMessage, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.message_id is None:
        self.message_id = 0
      if self.sender_id is None:
        self.sender_id = 0
      if self.velocity is None:
        self.velocity = 0.
      if self.steering is None:
        self.steering = 0.
      if self.posx is None:
        self.posx = 0.
      if self.posy is None:
        self.posy = 0.
      if self.posz is None:
        self.posz = 0.
      if self.road_id is None:
        self.road_id = 0
      if self.lane_id is None:
        self.lane_id = 0
    else:
      self.header = std_msgs.msg.Header()
      self.message_id = 0
      self.sender_id = 0
      self.velocity = 0.
      self.steering = 0.
      self.posx = 0.
      self.posy = 0.
      self.posz = 0.
      self.road_id = 0
      self.lane_id = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_IB5f2B().pack(_x.message_id, _x.sender_id, _x.velocity, _x.steering, _x.posx, _x.posy, _x.posz, _x.road_id, _x.lane_id))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 27
      (_x.message_id, _x.sender_id, _x.velocity, _x.steering, _x.posx, _x.posy, _x.posz, _x.road_id, _x.lane_id,) = _get_struct_IB5f2B().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_IB5f2B().pack(_x.message_id, _x.sender_id, _x.velocity, _x.steering, _x.posx, _x.posy, _x.posz, _x.road_id, _x.lane_id))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 27
      (_x.message_id, _x.sender_id, _x.velocity, _x.steering, _x.posx, _x.posy, _x.posz, _x.road_id, _x.lane_id,) = _get_struct_IB5f2B().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_IB5f2B = None
def _get_struct_IB5f2B():
    global _struct_IB5f2B
    if _struct_IB5f2B is None:
        _struct_IB5f2B = struct.Struct("<IB5f2B")
    return _struct_IB5f2B