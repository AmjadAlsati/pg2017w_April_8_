# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from cooperative_driving_msgs/Directions.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import cooperative_driving_msgs.msg

class Directions(genpy.Message):
  _md5sum = "f467a95e7546b9935ac1d69c0341e385"
  _type = "cooperative_driving_msgs/Directions"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """## List all directions of a detected crossing

# Array of all directions of a crossing 
Direction[] directions
================================================================================
MSG: cooperative_driving_msgs/Direction
## Define a direction of a detected crossing, i.e., a turn that can possibly be taken

# Possible turn at a crossing ('straight', 'left', or 'right')
string direction"""
  __slots__ = ['directions']
  _slot_types = ['cooperative_driving_msgs/Direction[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       directions

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Directions, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.directions is None:
        self.directions = []
    else:
      self.directions = []

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
      length = len(self.directions)
      buff.write(_struct_I.pack(length))
      for val1 in self.directions:
        _x = val1.direction
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.directions is None:
        self.directions = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.directions = []
      for i in range(0, length):
        val1 = cooperative_driving_msgs.msg.Direction()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.direction = str[start:end].decode('utf-8')
        else:
          val1.direction = str[start:end]
        self.directions.append(val1)
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
      length = len(self.directions)
      buff.write(_struct_I.pack(length))
      for val1 in self.directions:
        _x = val1.direction
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.directions is None:
        self.directions = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.directions = []
      for i in range(0, length):
        val1 = cooperative_driving_msgs.msg.Direction()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.direction = str[start:end].decode('utf-8')
        else:
          val1.direction = str[start:end]
        self.directions.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I