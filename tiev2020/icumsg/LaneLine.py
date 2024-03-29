"""ZCM type definitions
This file automatically generated by zcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

from icumsg.LinePoint import LinePoint as icumsg_LinePoint

class LaneLine(object):
    __slots__ = ["line_type", "distance", "num", "points"]

    IS_LITTLE_ENDIAN = False;
    TYPE_SOLID = 0x00
    TYPE_DASHED = 0x01
    TYPE_WHITE = 0x00
    TYPE_YELLOW = 0x02
    TYPE_SOLID_WHITE = 0x00
    TYPE_SOLID_YELLOW = 0x02
    TYPE_DASHED_WHITE = 0x01
    TYPE_DASHED_YELLOW = 0x03

    def __init__(self):
        self.line_type = 0
        self.distance = 0.0
        self.num = 0
        self.points = []

    def encode(self):
        buf = BytesIO()
        buf.write(LaneLine._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">ifi", self.line_type, self.distance, self.num))
        for i0 in range(self.num):
            assert self.points[i0]._get_packed_fingerprint() == icumsg_LinePoint._get_packed_fingerprint()
            self.points[i0]._encode_one(buf)

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
            self.points.append(icumsg_LinePoint._decode_one(buf))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if LaneLine in parents: return 0
        newparents = parents + [LaneLine]
        tmphash = (0xa0e1d18725221fb+ icumsg_LinePoint._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + ((tmphash>>63)&0x1)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if LaneLine._packed_fingerprint is None:
            LaneLine._packed_fingerprint = struct.pack(">Q", LaneLine._get_hash_recursive([]))
        return LaneLine._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

