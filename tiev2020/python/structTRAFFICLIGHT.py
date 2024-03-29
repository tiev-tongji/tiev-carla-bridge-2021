"""ZCM type definitions
This file automatically generated by zcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class structTRAFFICLIGHT(object):
    __slots__ = ["timestamp", "raw_signal", "turn_signal"]

    IS_LITTLE_ENDIAN = False;
    def __init__(self):
        self.timestamp = 0
        self.raw_signal = 0
        self.turn_signal = 0

    def encode(self):
        buf = BytesIO()
        buf.write(structTRAFFICLIGHT._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qBB", self.timestamp, self.raw_signal, self.turn_signal))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != structTRAFFICLIGHT._get_packed_fingerprint():
            raise ValueError("Decode error")
        return structTRAFFICLIGHT._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = structTRAFFICLIGHT()
        self.timestamp, self.raw_signal, self.turn_signal = struct.unpack(">qBB", buf.read(10))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if structTRAFFICLIGHT in parents: return 0
        tmphash = (0xa221bdbd3326eaf) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + ((tmphash>>63)&0x1)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if structTRAFFICLIGHT._packed_fingerprint is None:
            structTRAFFICLIGHT._packed_fingerprint = struct.pack(">Q", structTRAFFICLIGHT._get_hash_recursive([]))
        return structTRAFFICLIGHT._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

