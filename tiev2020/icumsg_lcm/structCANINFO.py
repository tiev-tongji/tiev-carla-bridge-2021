"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class structCANINFO(object):
    __slots__ = ["timestamp", "carsteer", "carspeed"]

    __typenames__ = ["int64_t", "int32_t", "int32_t"]

    __dimensions__ = [None, None, None]

    def __init__(self):
        self.timestamp = 0
        self.carsteer = 0
        self.carspeed = 0

    def encode(self):
        buf = BytesIO()
        buf.write(structCANINFO._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qii", self.timestamp, self.carsteer, self.carspeed))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != structCANINFO._get_packed_fingerprint():
            raise ValueError("Decode error")
        return structCANINFO._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = structCANINFO()
        self.timestamp, self.carsteer, self.carspeed = struct.unpack(">qii", buf.read(16))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if structCANINFO in parents: return 0
        tmphash = (0x10ea9421916cdc5e) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if structCANINFO._packed_fingerprint is None:
            structCANINFO._packed_fingerprint = struct.pack(">Q", structCANINFO._get_hash_recursive([]))
        return structCANINFO._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)
