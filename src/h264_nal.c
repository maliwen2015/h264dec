/*
 * Annex-B NAL unit splitting and RBSP unescaping.
 */
#include "h264_internal.h"

static int find_start_code(const uint8_t *data, int size, int from)
{
    int i;
    for (i = from; i + 2 < size; i++) {
        if (data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1)
            return i;
    }
    return -1;
}

/*
 * Find the next NAL unit.
 * *pos is the search position; on success it is advanced past the NAL payload.
 * Returns 1 if a NAL was found, 0 at end of data.
 */
int h264_split_annexb(const uint8_t *data, int size, int *pos,
                      const uint8_t **nal, int *nal_len)
{
    int sc, start, end;
    int search = *pos;

    if (search < 0 || search >= size)
        return 0;

    sc = find_start_code(data, size, search);
    if (sc < 0) {
        if (search == 0) {
            /* Tolerate a raw NAL unit without start code. */
            *nal = data;
            *nal_len = size;
            *pos = size;
            return size > 0;
        }
        *pos = size;
        return 0;
    }

    start = sc + 3;

    /* Skip any additional zero bytes (part of a 4-byte start code). */
    while (start < size && data[start] == 0)
        start++;

    if (start >= size) {
        *pos = size;
        return 0;
    }

    end = find_start_code(data, size, start);
    if (end < 0)
        end = size;
    else {
        /* The zero byte immediately preceding a 4-byte start code belongs
         * to the start code, not to the NAL payload.  RBSP trailing bits
         * guarantee the NAL does not end with zero bytes. */
        while (end > start && data[end - 1] == 0)
            end--;
    }

    *nal = data + start;
    *nal_len = end - start;
    *pos = end;
    return 1;
}

/*
 * Remove emulation prevention bytes from an EBSP.
 * dst must have room for src_len + 8 bytes.  Returns the RBSP length.
 */
int h264_unescape_rbsp(uint8_t *dst, int dst_size, const uint8_t *src, int src_len)
{
    int i, o = 0, zeros = 0;

    for (i = 0; i < src_len; i++) {
        uint8_t b = src[i];
        if (zeros >= 2 && b == 3) {
            if (i + 1 < src_len && src[i + 1] <= 3) {
                zeros = 0;
                continue;
            }
        }
        if (o >= dst_size - 8)
            return -1;
        dst[o++] = b;
        zeros = b ? 0 : zeros + 1;
    }

    memset(dst + o, 0, 8);
    return o;
}

/* One NAL of a length prefixed stream (the AVCC form used in MP4). A final
 * unit that does not fit is not reported: half a NAL cannot be decoded, and
 * stopping here keeps the decoder from misreading padding as syntax. */
int h264_split_avcc(const uint8_t *data, int size, int *pos,
                    const uint8_t **nal, int *nal_len, int len_size)
{
    int i, len = 0;

    if (*pos < 0 || *pos + len_size > size)
        return 0;
    for (i = 0; i < len_size; i++)
        len = (len << 8) | data[*pos + i];
    if (len <= 0 || *pos + len_size + len > size)
        return 0;
    *nal = data + *pos + len_size;
    *nal_len = len;
    *pos += len_size + len;
    return 1;
}
