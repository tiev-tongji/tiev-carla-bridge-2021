"""ZCM type definitions
This file automatically generated by zcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

from LinePoint import LinePoint

class LaneLine(object):
    __slots__ = ["line_type", "distance", "num", "points", "boundary_type", "boundary_confidence"]

    IS_LITTLE_ENDIAN = False;
    kTypeSolid = 0x00
    kTypeDashed = 0x01
    kTypeWhite = 0x00
    kTypeYellow = 0x02
    kTypeSolidWhite = 0x00
    kTypeSolidYellow = 0x02
    kTypeDashedWhite = 0x01
    kTypeDashedYellow = 0x03

    def __init__(self):
        self.line_type = 0
        self.distance = 0.0
        self.num = 0
        self.points = []
        self.boundary_type = 0
        self.boundary_confidence = 0

    def encode(self):
        buf = BytesIO()
        buf.write(LaneLine._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">ifi", self.line_type, self.distance, self.num))
        for i0 in range(self.num):
            assert self.points[i0]._get_packed_fingerprint() == LinePoint._get_packed_fingerprint()
            self.points[i0]._encode_one(buf)
        buf.write(struct.pack(">bi", self.boundary_type, self.boundary_confidence))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != LaneLine._get_packed_fingerprint():
            raise ValueError("Decode error")
        return LaneLine._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = LaneLine()
        self.line_type, self.distance, self.num = struct.unpack(">ifi", buf.read(12))
        self.points = []
        for i0 in range(self.num):
            self.points.append(LinePoint._decode_one(buf))
        self.boundary_type, self.boundary_confidence = struct.unpack(">bi", buf.read(5))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if LaneLine in parents: return 0
        newparents = parents + [LaneLine]
        tmphash = (0x751d2907c6cc9fa3+ LinePoint._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + ((tmphash>>63)&0x1)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if LaneLine._packed_fingerprint is None:
            LaneLine._packed_fingerprint = struct.pack(">Q", LaneLine._get_hash_recursive([]))
        return LaneLine._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

