"""ZCM type definitions
This file automatically generated by zcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

from icumsg.OBJECT import OBJECT as icumsg_OBJECT

class structOBJECTLIST(object):
    __slots__ = ["timestamp", "data_source", "count", "obj"]

    IS_LITTLE_ENDIAN = False;
    def __init__(self):
        self.timestamp = 0
        self.data_source = 0
        self.count = 0
        self.obj = []

    def encode(self):
        buf = BytesIO()
        buf.write(structOBJECTLIST._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qBb", self.timestamp, self.data_source, self.count))
        for i0 in range(self.count):
            assert self.obj[i0]._get_packed_fingerprint() == icumsg_OBJECT._get_packed_fingerprint()
            self.obj[i0]._encode_one(buf)

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != structOBJECTLIST._get_packed_fingerprint():
            raise ValueError("Decode error")
        return structOBJECTLIST._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = structOBJECTLIST()
        self.timestamp, self.data_source, self.count = struct.unpack(">qBb", buf.read(10))
        self.obj = []
        for i0 in range(self.count):
            self.obj.append(icumsg_OBJECT._decode_one(buf))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if structOBJECTLIST in parents: return 0
        newparents = parents + [structOBJECTLIST]
        tmphash = (0xc1fea086499ed5d+ icumsg_OBJECT._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + ((tmphash>>63)&0x1)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if structOBJECTLIST._packed_fingerprint is None:
            structOBJECTLIST._packed_fingerprint = struct.pack(">Q", structOBJECTLIST._get_hash_recursive([]))
        return structOBJECTLIST._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)
