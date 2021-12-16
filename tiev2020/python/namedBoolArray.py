"""ZCM type definitions
This file automatically generated by zcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class namedBoolArray(object):
    __slots__ = ["name", "val"]

    IS_LITTLE_ENDIAN = False;
    def __init__(self):
        self.name = ""
        self.val = False

    def encode(self):
        buf = BytesIO()
        buf.write(namedBoolArray._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        __name_encoded = self.name.encode('utf-8')
        buf.write(struct.pack('>I', len(__name_encoded)+1))
        buf.write(__name_encoded)
        buf.write(b"\0")
        buf.write(struct.pack(">b", self.val))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != namedBoolArray._get_packed_fingerprint():
            raise ValueError("Decode error")
        return namedBoolArray._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = namedBoolArray()
        __name_len = struct.unpack('>I', buf.read(4))[0]
        self.name = buf.read(__name_len)[:-1].decode('utf-8', 'replace')
        self.val = bool(struct.unpack('b', buf.read(1))[0])
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if namedBoolArray in parents: return 0
        tmphash = (0xb531878ad610d8d7) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + ((tmphash>>63)&0x1)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if namedBoolArray._packed_fingerprint is None:
            namedBoolArray._packed_fingerprint = struct.pack(">Q", namedBoolArray._get_hash_recursive([]))
        return namedBoolArray._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)
