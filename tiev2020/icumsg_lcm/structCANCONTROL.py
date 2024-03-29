"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class structCANCONTROL(object):
    __slots__ = ["timestamp", "aimsteer", "aimspeed"]

    __typenames__ = ["int64_t", "int32_t", "int32_t"]

    __dimensions__ = [None, None, None]

    def __init__(self):
        self.timestamp = 0
        self.aimsteer = 0
        self.aimspeed = 0

    def encode(self):
        buf = BytesIO()
        buf.write(structCANCONTROL._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qii", self.timestamp, self.aimsteer, self.aimspeed))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != structCANCONTROL._get_packed_fingerprint():
            raise ValueError("Decode error")
        return structCANCONTROL._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = structCANCONTROL()
        self.timestamp, self.aimsteer, self.aimspeed = struct.unpack(">qii", buf.read(16))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if structCANCONTROL in parents: return 0
        tmphash = (0x80a98df7d9e8b724) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if structCANCONTROL._packed_fingerprint is None:
            structCANCONTROL._packed_fingerprint = struct.pack(">Q", structCANCONTROL._get_hash_recursive([]))
        return structCANCONTROL._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

