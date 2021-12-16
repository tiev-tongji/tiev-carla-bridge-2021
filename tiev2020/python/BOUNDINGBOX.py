"""ZCM type definitions
This file automatically generated by zcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

from POSITION import POSITION

class BOUNDINGBOX(object):
    __slots__ = ["p1", "p2", "p3", "p4"]

    IS_LITTLE_ENDIAN = False;
    def __init__(self):
        self.p1 = POSITION()
        self.p2 = POSITION()
        self.p3 = POSITION()
        self.p4 = POSITION()

    def encode(self):
        buf = BytesIO()
        buf.write(BOUNDINGBOX._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        assert self.p1._get_packed_fingerprint() == POSITION._get_packed_fingerprint()
        self.p1._encode_one(buf)
        assert self.p2._get_packed_fingerprint() == POSITION._get_packed_fingerprint()
        self.p2._encode_one(buf)
        assert self.p3._get_packed_fingerprint() == POSITION._get_packed_fingerprint()
        self.p3._encode_one(buf)
        assert self.p4._get_packed_fingerprint() == POSITION._get_packed_fingerprint()
        self.p4._encode_one(buf)

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != BOUNDINGBOX._get_packed_fingerprint():
            raise ValueError("Decode error")
        return BOUNDINGBOX._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = BOUNDINGBOX()
        self.p1 = POSITION._decode_one(buf)
        self.p2 = POSITION._decode_one(buf)
        self.p3 = POSITION._decode_one(buf)
        self.p4 = POSITION._decode_one(buf)
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if BOUNDINGBOX in parents: return 0
        newparents = parents + [BOUNDINGBOX]
        tmphash = (0x5dc6ee02e5a6ec83+ POSITION._get_hash_recursive(newparents)+ POSITION._get_hash_recursive(newparents)+ POSITION._get_hash_recursive(newparents)+ POSITION._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + ((tmphash>>63)&0x1)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if BOUNDINGBOX._packed_fingerprint is None:
            BOUNDINGBOX._packed_fingerprint = struct.pack(">Q", BOUNDINGBOX._get_hash_recursive([]))
        return BOUNDINGBOX._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

