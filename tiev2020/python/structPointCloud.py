"""ZCM type definitions
This file automatically generated by zcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

from point3d import point3d

class structPointCloud(object):
    __slots__ = ["timestamp", "total", "frame"]

    IS_LITTLE_ENDIAN = False;
    def __init__(self):
        self.timestamp = 0
        self.total = 0
        self.frame = [ point3d() for dim0 in range(100000) ]

    def encode(self):
        buf = BytesIO()
        buf.write(structPointCloud._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qq", self.timestamp, self.total))
        for i0 in range(100000):
            assert self.frame[i0]._get_packed_fingerprint() == point3d._get_packed_fingerprint()
            self.frame[i0]._encode_one(buf)

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != structPointCloud._get_packed_fingerprint():
            raise ValueError("Decode error")
        return structPointCloud._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = structPointCloud()
        self.timestamp, self.total = struct.unpack(">qq", buf.read(16))
        self.frame = []
        for i0 in range(100000):
            self.frame.append(point3d._decode_one(buf))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if structPointCloud in parents: return 0
        newparents = parents + [structPointCloud]
        tmphash = (0xc1ad6a380b100ee3+ point3d._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + ((tmphash>>63)&0x1)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if structPointCloud._packed_fingerprint is None:
            structPointCloud._packed_fingerprint = struct.pack(">Q", structPointCloud._get_hash_recursive([]))
        return structPointCloud._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

