"""ZCM type definitions
This file automatically generated by zcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class structSLAMLOC(object):
    __slots__ = ["timestamp", "x", "y", "mHeading", "SLAMStatus"]

    IS_LITTLE_ENDIAN = False;
    def __init__(self):
        self.timestamp = 0
        self.x = 0.0
        self.y = 0.0
        self.mHeading = 0.0
        self.SLAMStatus = 0

    def encode(self):
        buf = BytesIO()
        buf.write(structSLAMLOC._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qdddB", self.timestamp, self.x, self.y, self.mHeading, self.SLAMStatus))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != structSLAMLOC._get_packed_fingerprint():
            raise ValueError("Decode error")
        return structSLAMLOC._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = structSLAMLOC()
        self.timestamp, self.x, self.y, self.mHeading, self.SLAMStatus = struct.unpack(">qdddB", buf.read(33))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if structSLAMLOC in parents: return 0
        tmphash = (0xd5b407f0d822811e) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + ((tmphash>>63)&0x1)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if structSLAMLOC._packed_fingerprint is None:
            structSLAMLOC._packed_fingerprint = struct.pack(">Q", structSLAMLOC._get_hash_recursive([]))
        return structSLAMLOC._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

