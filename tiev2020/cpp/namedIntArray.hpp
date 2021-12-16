/** THIS IS AN AUTOMATICALLY GENERATED FILE.
 *  DO NOT MODIFY BY HAND!!
 *
 *  Generated by zcm-gen
 **/

#include <zcm/zcm_coretypes.h>

#ifndef __namedIntArray_hpp__
#define __namedIntArray_hpp__

#include <string>


class namedIntArray
{
    public:
        std::string name;

        int32_t    val;

    public:
        /**
         * Destructs a message properly if anything inherits from it
        */
        virtual ~namedIntArray() {}

        /**
         * Encode a message into binary form.
         *
         * @param buf The output buffer.
         * @param offset Encoding starts at thie byte offset into @p buf.
         * @param maxlen Maximum number of bytes to write.  This should generally be
         *  equal to getEncodedSize().
         * @return The number of bytes encoded, or <0 on error.
         */
        inline int encode(void* buf, uint32_t offset, uint32_t maxlen) const;

        /**
         * Check how many bytes are required to encode this message.
         */
        inline uint32_t getEncodedSize() const;

        /**
         * Decode a message from binary form into this instance.
         *
         * @param buf The buffer containing the encoded message.
         * @param offset The byte offset into @p buf where the encoded message starts.
         * @param maxlen The maximum number of bytes to reqad while decoding.
         * @return The number of bytes decoded, or <0 if an error occured.
         */
        inline int decode(const void* buf, uint32_t offset, uint32_t maxlen);

        /**
         * Retrieve the 64-bit fingerprint identifying the structure of the message.
         * Note that the fingerprint is the same for all instances of the same
         * message type, and is a fingerprint on the message type definition, not on
         * the message contents.
         */
        inline static int64_t getHash();

        /**
         * Returns "namedIntArray"
         */
        inline static const char* getTypeName();

        // ZCM support functions. Users should not call these
        inline int      _encodeNoHash(void* buf, uint32_t offset, uint32_t maxlen) const;
        inline uint32_t _getEncodedSizeNoHash() const;
        inline int      _decodeNoHash(const void* buf, uint32_t offset, uint32_t maxlen);
        inline static uint64_t _computeHash(const __zcm_hash_ptr* p);
};

int namedIntArray::encode(void* buf, uint32_t offset, uint32_t maxlen) const
{
    uint32_t pos = 0;
    int thislen;
    int64_t hash = (int64_t)getHash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int namedIntArray::decode(const void* buf, uint32_t offset, uint32_t maxlen)
{
    uint32_t pos = 0;
    int thislen;

    int64_t msg_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (msg_hash != getHash()) return -1;

    thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

uint32_t namedIntArray::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t namedIntArray::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* namedIntArray::getTypeName()
{
    return "namedIntArray";
}

int namedIntArray::_encodeNoHash(void* buf, uint32_t offset, uint32_t maxlen) const
{
    uint32_t pos = 0;
    int thislen;

    char* name_cstr = (char*) this->name.c_str();
    thislen = __string_encode_array(buf, offset + pos, maxlen - pos, &name_cstr, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->val, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int namedIntArray::_decodeNoHash(const void* buf, uint32_t offset, uint32_t maxlen)
{
    uint32_t pos = 0;
    int thislen;

    int32_t __name_len__;
    thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &__name_len__, 1);
    if(thislen < 0) return thislen; else pos += thislen;
    if((uint32_t)__name_len__ > maxlen - pos) return -1;
    this->name.assign(((const char*)buf) + offset + pos, __name_len__ - ZCM_CORETYPES_INT8_NUM_BYTES_ON_BUS);
    pos += __name_len__;

    thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->val, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    return pos;
}

uint32_t namedIntArray::_getEncodedSizeNoHash() const
{
    uint32_t enc_size = 0;
    enc_size += this->name.size() + ZCM_CORETYPES_INT32_NUM_BYTES_ON_BUS + ZCM_CORETYPES_INT8_NUM_BYTES_ON_BUS;
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    return enc_size;
}

uint64_t namedIntArray::_computeHash(const __zcm_hash_ptr*)
{
    uint64_t hash = (uint64_t)0x0191df251e9b08a9LL;
    return (hash<<1) + ((hash>>63)&1);
}

#endif