"""ZCM type definitions
This file automatically generated by zcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class POSITION(object):
    __slots__ = ["x", "y"]

    IS_LITTLE_ENDIAN = False;
    def __init__(self):
        self.x = 0.0
        self.y = 0.0

    def encode(self):
        buf = BytesIO()
        buf.write(POSITION._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">ff", self.x, self.y))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != POSITION._get_packed_fingerprint():
            raise ValueError("Decode error")
        return POSITION._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = POSITION()
        self.x, self.y = struct.unpack(">ff", buf.read(8))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if POSITION in parents: return 0
        tmphash = (0x81c24e6195f5bef1) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + ((tmphash>>63)&0x1)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if POSITION._packed_fingerprint is None:
            POSITION._packed_fingerprint = struct.pack(">Q", POSITION._get_hash_recursive([]))
        return POSITION._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

