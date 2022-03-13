import struct

from . import util

float32 = "f"
float64 = "d"
uint8   = "c"
uint32  = "I"
bytestr = object()

byteorder = "<"

class _Message(object):

    def __repr__(self):
        # TODO pretty function args instead of exploding a dictionary
        return "%s(**%s)" % (self.__class__.__name__, repr(self._data))

    def __getattr__(self, name):
        if name in self._data:
            return self._data[name]
        raise AttributeError(name)
    
    def _iter_data(self):
        for k in self._keys:
            yield k, self._spec[k], self._data[k]
    
    def write_into(self, writer):
        wrote = writer.write(struct.pack("<B", self._position))
        for k in self._keys:
            spec, v = self._spec[k], self._data[k]
            if spec is bytestr:
                wrote += writer.write(struct.pack(byteorder + uint32, len(v)))
                wrote += writer.write(v)
            else:
                wrote += writer.write(struct.pack(byteorder + spec, v))
        return wrote
        
    @staticmethod
    def read_from(reader):
        read = util.create_read_exactly(reader)
        position, = struct.unpack("<B", read(1))
        cls = message_types[position]
        result = {}
        for k in cls._keys:
            spec = cls._spec[k]
            if spec is bytestr:
                length, = struct.unpack(byteorder + uint32, read(4))
                result[k] = read(length)
            else:
                spec = byteorder + spec
                length = struct.calcsize(spec)
                result[k], = struct.unpack(spec, read(length))
        return cls(**result)

def msg(name, **spec):

    keys = sorted(spec.keys())
    key_set = set(keys)

    def init(self, **data):
        assert set(data.keys()) == key_set, "Expected keys: %s." % ", ".join(keys)
        self._data = data
        _Message.__init__(self)
        
    return type(name, (_Message,), {
        "_keys": keys,
        "_spec": spec,
        "__init__": init
    })
    
message_types = [
    msg("MotorVelocity", velocity=float32),
    msg("CameraInfo", width=uint32, height=uint32),
    msg("CameraImage", image=bytestr)
]

def read_next_message(reader):
    return _Message.read_from(reader)

# register all the messages as top-level classes
for ix, mt in enumerate(message_types):
    mt._position = ix
    vars()[mt.__name__] = mt