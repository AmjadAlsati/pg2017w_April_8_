# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from cooperative_driving_vision/Moment.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class Moment(genpy.Message):
  _md5sum = "15ca49fd130f761e715cfaf6f1985ad7"
  _type = "cooperative_driving_vision/Moment"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# Custom message type to represent a geometrical moment

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
"""
  __slots__ = ['m00','m10','m01','m11','m20','m02','m21','m12','m30','m03']
  _slot_types = ['float32','float32','float32','float32','float32','float32','float32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       m00,m10,m01,m11,m20,m02,m21,m12,m30,m03

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Moment, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.m00 is None:
        self.m00 = 0.
      if self.m10 is None:
        self.m10 = 0.
      if self.m01 is None:
        self.m01 = 0.
      if self.m11 is None:
        self.m11 = 0.
      if self.m20 is None:
        self.m20 = 0.
      if self.m02 is None:
        self.m02 = 0.
      if self.m21 is None:
        self.m21 = 0.
      if self.m12 is None:
        self.m12 = 0.
      if self.m30 is None:
        self.m30 = 0.
      if self.m03 is None:
        self.m03 = 0.
    else:
      self.m00 = 0.
      self.m10 = 0.
      self.m01 = 0.
      self.m11 = 0.
      self.m20 = 0.
      self.m02 = 0.
      self.m21 = 0.
      self.m12 = 0.
      self.m30 = 0.
      self.m03 = 0.

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
      buff.write(_get_struct_10f().pack(_x.m00, _x.m10, _x.m01, _x.m11, _x.m20, _x.m02, _x.m21, _x.m12, _x.m30, _x.m03))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 40
      (_x.m00, _x.m10, _x.m01, _x.m11, _x.m20, _x.m02, _x.m21, _x.m12, _x.m30, _x.m03,) = _get_struct_10f().unpack(str[start:end])
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
      buff.write(_get_struct_10f().pack(_x.m00, _x.m10, _x.m01, _x.m11, _x.m20, _x.m02, _x.m21, _x.m12, _x.m30, _x.m03))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 40
      (_x.m00, _x.m10, _x.m01, _x.m11, _x.m20, _x.m02, _x.m21, _x.m12, _x.m30, _x.m03,) = _get_struct_10f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_10f = None
def _get_struct_10f():
    global _struct_10f
    if _struct_10f is None:
        _struct_10f = struct.Struct("<10f")
    return _struct_10f
