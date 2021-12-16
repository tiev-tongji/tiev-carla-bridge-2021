"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

import icumsg_lcm.LANE

class structLANES(object):
    __slots__ = ["current_lane_id", "num", "lanes"]

    __typenames__ = ["int32_t", "int32_t", "icumsg_lcm.LANE"]

    __dimensions__ = [None, None, ["num"]]

    def __init__(self):
        self.current_lane_id = 0
        self.num = 0
        self.lanes = []

    def encode(self):
        buf = BytesIO()
        buf.write(structLANES._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">ii", self.current_lane_id, self.num))
        for i0 in range(self.num):
            assert self.lanes[i0]._get_packed_fingerprint() == icumsg_lcm.LANE._get_packed_fingerprint()
            self.lanes[i0]._encode_one(buf)

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != structLANES._get_packed_fingerprint():
            raise ValueError("Decode error")
        return structLANES._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = structLANES()
        self.current_lane_id, self.num = struct.unpack(">ii", buf.read(8))
        self.lanes = []
        for i0 in range(self.num):
            self.lanes.append(icumsg_lcm.LANE._decode_one(buf))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if structLANES in parents: return 0
        newparents = parents + [structLANES]
        tmphash = (0x88b27e6f5c7b0044+ icumsg_lcm.LANE._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if structLANES._packed_fingerprint is None:
            structLANES._packed_fingerprint = struct.pack(">Q", structLANES._get_hash_recursive([]))
        return structLANES._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

