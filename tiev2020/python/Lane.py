"""ZCM type definitions
This file automatically generated by zcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

from LaneLine import LaneLine

class Lane(object):
    __slots__ = ["lane_type", "width", "left_line", "right_line"]

    IS_LITTLE_ENDIAN = False;
    kTypeNone = 0x00
    kTypeStraight = 0x01
    kTypeLeft = 0x02
    kTypeStraightLeft = 0x03
    kTypeRight = 0x04
    kTypeStraightRight = 0x05
    kTypeStraightLeftRight = 0x07
    kTypeUTurn = 0x08
    kTypeLeftRight = 0x06
    kTypeLeftUTurn = 0x09
    kTypeStraightUTurn = 0x0A
    kTypeMerge = 0x0B

    def __init__(self):
        self.lane_type = 0
        self.width = 0.0
        self.left_line = LaneLine()
        self.right_line = LaneLine()

    def encode(self):
        buf = BytesIO()
        buf.write(Lane._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">if", self.lane_type, self.width))
        assert self.left_line._get_packed_fingerprint() == LaneLine._get_packed_fingerprint()
        self.left_line._encode_one(buf)
        assert self.right_line._get_packed_fingerprint() == LaneLine._get_packed_fingerprint()
        self.right_line._encode_one(buf)

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != Lane._get_packed_fingerprint():
            raise ValueError("Decode error")
        return Lane._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = Lane()
        self.lane_type, self.width = struct.unpack(">if", buf.read(8))
        self.left_line = LaneLine._decode_one(buf)
        self.right_line = LaneLine._decode_one(buf)
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if Lane in parents: return 0
        newparents = parents + [Lane]
        tmphash = (0x2364f989fbf23e64+ LaneLine._get_hash_recursive(newparents)+ LaneLine._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + ((tmphash>>63)&0x1)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if Lane._packed_fingerprint is None:
            Lane._packed_fingerprint = struct.pack(">Q", Lane._get_hash_recursive([]))
        return Lane._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

