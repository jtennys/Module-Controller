"""autogenerated by genmsg_py from GetArmTipRequest.msg. Do not edit."""
import roslib.message
import struct


class GetArmTipRequest(roslib.message.Message):
  _md5sum = "614b939b5fa1fbf0c2d50eee578a4e28"
  _type = "module_controller/GetArmTipRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int8 Nothing

"""
  __slots__ = ['Nothing']
  _slot_types = ['int8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       Nothing
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(GetArmTipRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.Nothing is None:
        self.Nothing = 0
    else:
      self.Nothing = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      buff.write(_struct_b.pack(self.Nothing))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      start = end
      end += 1
      (self.Nothing,) = _struct_b.unpack(str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      buff.write(_struct_b.pack(self.Nothing))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      end = 0
      start = end
      end += 1
      (self.Nothing,) = _struct_b.unpack(str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_b = struct.Struct("<b")
"""autogenerated by genmsg_py from GetArmTipResponse.msg. Do not edit."""
import roslib.message
import struct


class GetArmTipResponse(roslib.message.Message):
  _md5sum = "cc153912f1453b708d221682bc23d9ac"
  _type = "module_controller/GetArmTipResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float32 x
float32 y
float32 z


"""
  __slots__ = ['x','y','z']
  _slot_types = ['float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       x,y,z
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(GetArmTipResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.x is None:
        self.x = 0.
      if self.y is None:
        self.y = 0.
      if self.z is None:
        self.z = 0.
    else:
      self.x = 0.
      self.y = 0.
      self.z = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_3f.pack(_x.x, _x.y, _x.z))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      _x = self
      start = end
      end += 12
      (_x.x, _x.y, _x.z,) = _struct_3f.unpack(str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_3f.pack(_x.x, _x.y, _x.z))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 12
      (_x.x, _x.y, _x.z,) = _struct_3f.unpack(str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_3f = struct.Struct("<3f")
class GetArmTip(roslib.message.ServiceDefinition):
  _type          = 'module_controller/GetArmTip'
  _md5sum = 'be5e429ef59289bd2dccbf657856fbfc'
  _request_class  = GetArmTipRequest
  _response_class = GetArmTipResponse