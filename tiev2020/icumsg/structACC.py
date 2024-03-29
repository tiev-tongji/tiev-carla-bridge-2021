"""ZCM type definitions
This file automatically generated by zcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class structACC(object):
    __slots__ = ["timestamp", "ACCspeedforward", "ACCspeedbackward"]

    IS_LITTLE_ENDIAN = False;
    def __init__(self):
        self.timestamp = 0
        self.ACCspeedforward = 0
        self.ACCspeedbackward = 0

    def encode(self):
        buf = BytesIO()
        buf.write(structACC._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qii", self.timestamp, self.ACCspeedforward, self.ACCspeedbackward))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != structACC._get_packed_fingerprint():
            raise ValueError("Decode error")
        return structACC._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = structACC()
        self.timestamp, self.ACCspeedforward, self.ACCspeedbackward = struct.unpack(">qii", buf.read(16))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if structACC in parents: return 0
        tmphash = (0x28d984fe58c9df5b) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + ((tmphash>>63)&0x1)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if structACC._packed_fingerprint is None:
            structACC._packed_fingerprint = struct.pack(">Q", structACC._get_hash_recursive([]))
        return structACC._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

