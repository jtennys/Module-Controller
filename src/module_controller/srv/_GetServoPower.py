"""autogenerated by genmsg_py from GetServoPowerRequest.msg. Do not edit."""
import roslib.message
import struct


class GetServoPowerRequest(roslib.message.Message):
  _md5sum = "0cb5143ed23a5de01874507c8711c4d5"
  _type = "module_controller/GetServoPowerRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int8 ID

"""
  __slots__ = ['ID']
  _slot_types = ['int8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       ID
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(GetServoPowerRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.ID is None:
        self.ID = 0
    else:
      self.ID = 0

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
      buff.write(_struct_b.pack(self.ID))
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
      (self.ID,) = _struct_b.unpack(str[start:end])
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
      buff.write(_struct_b.pack(self.ID))
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
      (self.ID,) = _struct_b.unpack(str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_b = struct.Struct("<b")
"""autogenerated by genmsg_py from GetServoPowerResponse.msg. Do not edit."""
import roslib.message
import struct


class GetServoPowerResponse(roslib.message.Message):
  _md5sum = "1d4befc03e6691e2b4a0bfe78761754b"
  _type = "module_controller/GetServoPowerResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int8 Power


"""
  __slots__ = ['Power']
  _slot_types = ['int8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       Power
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(GetServoPowerResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.Power is None:
        self.Power = 0
    else:
      self.Power = 0

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
      buff.write(_struct_b.pack(self.Power))
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
      (self.Power,) = _struct_b.unpack(str[start:end])
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
      buff.write(_struct_b.pack(self.Power))
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
      (self.Power,) = _struct_b.unpack(str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_b = struct.Struct("<b")
class GetServoPower(roslib.message.ServiceDefinition):
  _type          = 'module_controller/GetServoPower'
  _md5sum = '63625bde14a8229e56f4eaf5667c7359'
  _request_class  = GetServoPowerRequest
  _response_class = GetServoPowerResponse
