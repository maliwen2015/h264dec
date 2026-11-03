/*
 * SEI checks: hand made messages go through the walker and the picture timing /
 * recovery point readers. Covers the encoder build (which the reference
 * workarounds read), messages the decoder does not know, truncated and flooded
 * payloads, several messages in one NAL, and the field widths that picture
 * timing needs from the SPS.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "h264_internal.h"

static int fails;

static void check(const char *what, long got, long expect)
{
    printf("%-30s got=%ld expect=%ld %s\n", what, got, expect,
           got == expect ? "ok" : "FAIL");
    if (got != expect)
        fails++;
}

static H264Decoder *fresh(void)
{
    H264Decoder *h = (H264Decoder *)calloc(1, sizeof(*h));
    h->x264_build = -1;
    h264_sei_reset(h);
    return h;
}

static void add_payload(unsigned char *buf, int *pos, int type,
                        const unsigned char *data, int size)
{
    int t = type, s = size;
    while (t >= 255) { buf[(*pos)++] = 255; t -= 255; }
    buf[(*pos)++] = (unsigned char)t;
    while (s >= 255) { buf[(*pos)++] = 255; s -= 255; }
    buf[(*pos)++] = (unsigned char)s;
    if (data && size)
        memcpy(buf + *pos, data, size);
    *pos += size;
}

static int x264_sei(unsigned char *buf, int *pos, const char *text)
{
    unsigned char data[16 + 64];
    int n = (int)strlen(text);
    if (n > 64)
        return -1;
    memset(data, 0, 16);
    data[0] = 0xdc; data[1] = 0x45; data[2] = 0xe6; data[3] = 0xbd;
    data[4] = 0xe9; data[5] = 0xd9; data[6] = 0x12; data[7] = 0xb6;
    memcpy(data + 16, text, (size_t)n);
    add_payload(buf, pos, 5, data, 16 + n);
    return 0;
}

/* one SEI NAL out of the message bytes, walked as the decoder would */
static void feed(H264Decoder *h, const unsigned char *body, int len)
{
    unsigned char *nal = (unsigned char *)malloc((size_t)len + 4);
    int n = 0;

    nal[n++] = 6 << 5;                 /* nal_unit_type 6 */
    memcpy(nal + n, body, (size_t)len);
    n += len;
    nal[n++] = 0x80;                   /* rbsp stop bit */
    h264_decode_sei(h, nal, n);
    free(nal);
}

static void build_sps(H264SPS *sps, int hrd)
{
    memset(sps, 0, sizeof(*sps));
    sps->pic_struct_present_flag = 1;
    sps->nal_hrd_parameters_present_flag = hrd;
    sps->cpb_removal_delay_length = 8;
    sps->dpb_output_delay_length = 8;
    sps->time_offset_length = 24;
}

/* bits into a byte buffer, most significant bit first */
static void put_bits(unsigned char *buf, int *bitpos, int value, int n)
{
    int i;
    for (i = n - 1; i >= 0; i--) {
        int p = (*bitpos)++;
        if (value & (1 << i))
            buf[p >> 3] |= (unsigned char)(0x80 >> (p & 7));
    }
}

static H264SPS sps;

int main(void)
{
    unsigned char body[1024];
    unsigned char pt[64];
    H264PicTiming out;
    H264Decoder *h;
    int pos, i, bitpos;

    /* --- encoder identity, which the reference workarounds switch on --- */
    h = fresh();
    pos = 0;
    x264_sei(body, &pos, "x264 - core 164 r3191 7dd49d3");
    feed(h, body, pos);
    check("encoder identity", h->x264_build, 164);
    free(h);

    h = fresh();
    pos = 0;
    x264_sei(body, &pos, "x264 - core 0000 - enc.h264");
    /* sscanf reads 0, which is not a build number: the reference ignores this
     * string too (its 67 special case needs build == 1) */
    feed(h, body, pos);
    check("early x264 reports 0000", h->x264_build, -1);
    free(h);

    /* --- the walker must reach the message after a known one --- */
    h = fresh();
    pos = 0;
    add_payload(body, &pos, 1, (const unsigned char *)"\xff\xff\xff\xff", 4);
    x264_sei(body, &pos, "x264 - core 152");
    feed(h, body, pos);
    check("picture timing then user data", h->x264_build, 152);
    check("  timing payload buffered", h->sei.pt_present, 1);
    check("  timing payload size", h->sei.pt_size, 4);
    free(h);

    h = fresh();
    pos = 0;
    add_payload(body, &pos, 13, (const unsigned char *)"3dcc", 4);
    add_payload(body, &pos, 5, (const unsigned char *)"aaaaaaaaaaaaaaaa"
                  "x264 - core 148", 16 + 15);
    feed(h, body, pos);
    check("unknown type then user data", h->x264_build, 148);
    free(h);

    /* --- damaged messages must stop the walk, not run off the buffer --- */
    h = fresh();
    pos = 0;
    add_payload(body, &pos, 3, (const unsigned char *)"aaaa", 4);   /* filler */
    add_payload(body, &pos, 5, (const unsigned char *)"no uuid here", 12);
    feed(h, body, pos);
    check("short user data", h->x264_build, -1);
    free(h);

    h = fresh();
    pos = 0;
    body[pos++] = 5;
    body[pos++] = 200;                     /* claims more than it carries */
    for (i = 0; i < 20; i++)
        body[pos++] = 'x';
    feed(h, body, pos);
    check("truncated payload", h->x264_build, -1);
    free(h);

    h = fresh();
    pos = 0;
    for (i = 0; i < 40; i++)
        body[pos++] = 255;                 /* endless 0xFF extension */
    body[pos++] = 5;
    body[pos++] = 2;
    body[pos++] = 0;
    feed(h, body, pos);
    check("0xFF flood", h->x264_build, -1);
    free(h);

    h = fresh();
    pos = 0;
    for (i = 0; i < 300; i++)
        body[pos++] = 0;
    feed(h, body, pos);
    check("all zero", h->x264_build, -1);
    free(h);

    /* --- recovery point: ue(2), exact_match 1, broken 0, idc 0 --- */
    h = fresh();
    pos = 0;
    memset(pt, 0, sizeof(pt));
    bitpos = 0;
    put_bits(pt, &bitpos, 0, 1);           /* ue(2) = 0 1 1 */
    put_bits(pt, &bitpos, 1, 1);
    put_bits(pt, &bitpos, 1, 1);
    put_bits(pt, &bitpos, 1, 1);           /* exact_match_flag */
    put_bits(pt, &bitpos, 0, 1);           /* broken_recovery_flag */
    put_bits(pt, &bitpos, 0, 2);           /* changing_slice_group_idc */
    add_payload(body, &pos, 6, pt, 1);
    x264_sei(body, &pos, "x264 - core 155");
    feed(h, body, pos);
    check("recovery frame count", h->sei.recovery_frame_cnt, 2);
    check("recovery exact match", h->sei.exact_match, 1);
    check("recovery broken", h->sei.broken_recovery, 0);
    check("  other message still read", h->x264_build, 155);
    free(h);

    /* a recovery count of 2^16 or more is out of range, as the reference says */
    h = fresh();
    pos = 0;
    memset(pt, 0, sizeof(pt));
    bitpos = 0;
    for (i = 0; i < 16; i++)
        put_bits(pt, &bitpos, 0, 1);       /* 16 leading zeros */
    put_bits(pt, &bitpos, 1, 1);
    for (i = 0; i < 16; i++)
        put_bits(pt, &bitpos, 1, 1);
    add_payload(body, &pos, 6, pt, 4);
    feed(h, body, pos);
    check("huge recovery count", h->sei.recovery_frame_cnt, -1);
    free(h);

    /* --- picture timing with an HRD: two delays then pic_struct 5 --- */
    build_sps(&sps, 1);
    memset(pt, 0, sizeof(pt));
    bitpos = 0;
    put_bits(pt, &bitpos, 0x2a, 8);        /* cpb_removal_delay */
    put_bits(pt, &bitpos, 0x15, 8);        /* dpb_output_delay */
    put_bits(pt, &bitpos, 5, 4);           /* pic_struct = top_bottom_top */
    put_bits(pt, &bitpos, 1, 1);           /* clock_timestamp_flag */
    put_bits(pt, &bitpos, 2, 2);           /* the two bits that set ct_type */
    put_bits(pt, &bitpos, 0, 1);           /* nuit_field_based_flag */
    put_bits(pt, &bitpos, 2, 5);           /* counting_type: drop frame range */
    put_bits(pt, &bitpos, 1, 1);           /* full_timestamp_flag */
    put_bits(pt, &bitpos, 0, 1);           /* discontinuity_flag */
    put_bits(pt, &bitpos, 1, 1);           /* cnt_dropped_flag */
    put_bits(pt, &bitpos, 12, 8);          /* n_frames */
    put_bits(pt, &bitpos, 34, 6);          /* seconds_value */
    put_bits(pt, &bitpos, 8, 6);           /* minutes_value */
    put_bits(pt, &bitpos, 13, 5);          /* hours_value */
    put_bits(pt, &bitpos, 0, 24);          /* time_offset */
    put_bits(pt, &bitpos, 1, 1);           /* clock_timestamp_flag */
    put_bits(pt, &bitpos, 1, 2);           /* counting_type */
    put_bits(pt, &bitpos, 0, 1);           /* nuit_field_based_flag */
    put_bits(pt, &bitpos, 0, 5);
    put_bits(pt, &bitpos, 0, 1);           /* full_timestamp_flag */
    put_bits(pt, &bitpos, 0, 1);           /* discontinuity_flag */
    put_bits(pt, &bitpos, 0, 1);           /* cnt_dropped_flag */
    put_bits(pt, &bitpos, 7, 8);           /* n_frames */
    put_bits(pt, &bitpos, 0, 1);           /* seconds_flag */
    put_bits(pt, &bitpos, 0, 24);          /* time_offset */
    i = (bitpos + 7) / 8;
    if (h264_sei_read_picture_timing(&sps, pt, i, &out) == 0) {
        check("timing cpb delay", out.cpb_removal_delay, 0x2a);
        check("timing dpb delay", out.dpb_output_delay, 0x15);
        check("timing pic_struct", out.pic_struct, 5);
        check("timing ct_type bits", out.ct_type, (1 << 2) | (1 << 1));
        check("timing first clock only", out.tc_valid, 1);
        check("timing hours", out.tc_hours, 13);
        check("timing minutes", out.tc_minutes, 8);
        check("timing seconds", out.tc_seconds, 34);
        check("timing frames", out.tc_frames, 12);
        check("timing drop frame", out.tc_drop, 1);
    } else {
        check("timing read without error", -1, 0);
    }

    /* without an HRD there are no delays, and pic_struct starts the payload */
    build_sps(&sps, 0);
    memset(pt, 0, sizeof(pt));
    bitpos = 0;
    put_bits(pt, &bitpos, 0, 4);           /* pic_struct = frame */
    put_bits(pt, &bitpos, 0, 1);           /* clock_timestamp_flag */
    i = (bitpos + 7) / 8;
    if (h264_sei_read_picture_timing(&sps, pt, i, &out) == 0) {
        check("no HRD: cpb delay", out.cpb_removal_delay, -1);
        check("no HRD: pic_struct", out.pic_struct, 0);
    } else {
        check("no HRD read without error", -1, 0);
    }

    /* a reserved pic_struct makes the message untrustable */
    memset(pt, 0, sizeof(pt));
    bitpos = 0;
    put_bits(pt, &bitpos, 14, 4);
    i = (bitpos + 7) / 8;
    check("reserved pic_struct rejected",
          h264_sei_read_picture_timing(&sps, pt, i, &out), -1);

    /* a payload that stops in the middle must be reported as damaged */
    memset(pt, 0, sizeof(pt));
    check("empty timing payload rejected",
          h264_sei_read_picture_timing(&sps, pt, 0, &out), -1);

    printf("sei_check: %d failures\n", fails);
    return fails ? 1 : 0;
}
