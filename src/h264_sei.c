/*
 * Supplemental enhancement information (ITU-T H.264 D.1).
 *
 * Three payloads change what the caller can do with a picture: the
 * unregistered user data carrying the encoder build (the reference decoder
 * keeps x264 compatibility workarounds that switch on it, so matching it keeps
 * old streams bit-exact), picture timing (field order and the CPB/DPB delays)
 * and the recovery point of an open GOP. Everything else is skipped by its
 * declared size, which is what makes walking unknown or damaged messages safe.
 *
 * A picture timing payload is only decodable with the HRD field widths of the
 * SPS, and the SEI can precede that SPS in the access unit, so the raw payload
 * is buffered here and read when the picture it belongs to starts.
 */
#include "h264_internal.h"

#define SEI_PIC_TIMING          1
#define SEI_USER_DATA_UNREG     5
#define SEI_RECOVERY_POINT      6

/* clock_timestamp() entries per pic_struct, D.3.2.3 */
static const uint8_t sei_num_clock_ts_table[9] = { 1, 1, 1, 2, 2, 3, 3, 2, 3 };

static int get_byte(H264BitReader *br)
{
    if (br->pos + 8 > br->size_bits) {
        br->error = 1;
        return -1;
    }
    return (int)h264_br_get_bits(br, 8);
}

static void sei_user_data_unregistered(H264Decoder *h, H264BitReader *br, int size)
{
    char text[128];
    int n = size > 16 ? H264_MIN(size - 16, (int)sizeof(text) - 1) : 0;
    int i, build = 0;

    /* the payload starts with a 16 byte UUID */
    if (h264_br_bits_left(br) < 16 * 8)
        return;
    br->pos += 16 * 8;
    for (i = 0; i < n; i++) {
        int c = get_byte(br);
        if (c < 0)
            break;
        text[i] = (char)c;
    }
    text[i] = 0;
    if (sscanf(text, "x264 - core %d", &build) == 1 && build > 0)
        h->x264_build = build;
    if (build == 1 && !strncmp(text, "x264 - core 0000", 16))
        h->x264_build = 67;      /* early x264 reported a bogus build */
}

static void sei_picture_timing(H264Decoder *h, H264BitReader *br, int size)
{
    int i;

    /* the delays are only readable with an SPS, so keep the bytes and wait */
    if (size <= 0 || size > (int)sizeof(h->sei.pt_payload))
        return;
    for (i = 0; i < size; i++) {
        int c = get_byte(br);
        if (c < 0)
            return;
        h->sei.pt_payload[i] = (uint8_t)c;
    }
    h->sei.pt_size = size;
    h->sei.pt_present = 1;
}

static void sei_recovery_point(H264Decoder *h, H264BitReader *br)
{
    unsigned cnt = h264_br_get_ue(br);

    if (br->error || cnt >= (1u << 16))
        return;                    /* out of range, as the reference treats it */
    h->sei.recovery_frame_cnt = (int)cnt;
    h->sei.exact_match = (int)h264_br_get_bits1(br);
    h->sei.broken_recovery = (int)h264_br_get_bits1(br);
    h264_br_skip(br, 2);           /* changing_slice_group_idc */
}

int h264_sei_read_picture_timing(const H264SPS *sps, const uint8_t *payload,
                                 int size, H264PicTiming *out)
{
    H264BitReader br;
    int i;

    memset(out, 0, sizeof(*out));
    out->cpb_removal_delay = -1;
    out->pic_struct = -1;
    if (size <= 0)
        return -1;
    h264_br_init(&br, payload, size * 8);

    if (sps->nal_hrd_parameters_present_flag ||
        sps->vcl_hrd_parameters_present_flag) {
        out->cpb_removal_delay = (int)h264_br_get_bits(&br, sps->cpb_removal_delay_length);
        out->dpb_output_delay = (int)h264_br_get_bits(&br, sps->dpb_output_delay_length);
    }
    if (!sps->pic_struct_present_flag)
        return br.error ? -1 : 0;

    out->pic_struct = (int)h264_br_get_bits(&br, 4);
    if (out->pic_struct > 8)
        return -1;                 /* reserved, the message cannot be trusted */

    for (i = 0; i < sei_num_clock_ts_table[out->pic_struct]; i++) {
        int counting_type, full, drop, first = !out->tc_valid;
        int hours = 0, minutes = 0, seconds = 0;

        if (!h264_br_get_bits1(&br))
            continue;              /* clock_timestamp_flag */
        out->ct_type |= 1 << (int)h264_br_get_bits(&br, 2);
        h264_br_skip(&br, 1);      /* nuit_field_based_flag */
        counting_type = (int)h264_br_get_bits(&br, 5);
        full = (int)h264_br_get_bits1(&br);
        h264_br_skip(&br, 1);      /* discontinuity_flag */
        drop = (int)h264_br_get_bits1(&br);
        if (first)
            out->tc_frames = (int)h264_br_get_bits(&br, 8);
        else
            h264_br_skip(&br, 8);  /* n_frames */
        if (full) {
            seconds = (int)h264_br_get_bits(&br, 6);
            minutes = (int)h264_br_get_bits(&br, 6);
            hours = (int)h264_br_get_bits(&br, 5);
        } else if (h264_br_get_bits1(&br)) {          /* seconds_flag */
            seconds = (int)h264_br_get_bits(&br, 6);
            if (h264_br_get_bits1(&br)) {             /* minutes_flag */
                minutes = (int)h264_br_get_bits(&br, 6);
                if (h264_br_get_bits1(&br))           /* hours_flag */
                    hours = (int)h264_br_get_bits(&br, 5);
            }
        }
        h264_br_skip(&br, sps->time_offset_length);
        if (first) {
            out->tc_valid = 1;
            out->tc_hours = hours;
            out->tc_minutes = minutes;
            out->tc_seconds = seconds;
            /* a dropped frame only counts for the SMPTE non-real-time types */
            out->tc_drop = drop && counting_type > 1 && counting_type < 7;
        }
    }
    return br.error ? -1 : 0;
}

void h264_sei_reset(H264Decoder *h)
{
    h->sei.pt_present = 0;
    h->sei.pt_size = 0;
    h->sei.recovery_frame_cnt = -1;
    h->sei.exact_match = 0;
    h->sei.broken_recovery = 0;
}

void h264_decode_sei(H264Decoder *h, const uint8_t *nal, int len)
{
    uint8_t *rbsp;
    int size;
    H264BitReader br;

    if (len < 2)
        return;
    rbsp = (uint8_t *)malloc(len + 16);
    if (!rbsp)
        return;
    size = h264_unescape_rbsp(rbsp, len + 16, nal + 1, len - 1);
    if (size <= 0) {
        free(rbsp);
        return;
    }
    h264_br_init(&br, rbsp, size * 8);

    while (!br.error && h264_br_bits_left(&br) > 8) {
        int type = 0, payload = 0, b, start;

        do {
            b = get_byte(&br);
            if (b < 0)
                goto done;
            if (type > 0xff00) {
                br.error = 1;
                goto done;
            }
            type += b;
        } while (b == 255);
        do {
            b = get_byte(&br);
            if (b < 0)
                goto done;
            if (payload > (br.size_bits >> 3)) {
                br.error = 1;
                goto done;
            }
            payload += b;
        } while (b == 255);
        if (type > 0xff00 ||
            (int64_t)payload * 8 > h264_br_bits_left(&br))
            break;

        start = br.pos;
        if (type == SEI_PIC_TIMING)
            sei_picture_timing(h, &br, payload);
        else if (type == SEI_USER_DATA_UNREG)
            sei_user_data_unregistered(h, &br, payload);
        else if (type == SEI_RECOVERY_POINT)
            sei_recovery_point(h, &br);
        /* the declared size, not what the handler read, decides where the next
         * message starts: a handler stops early on damaged input */
        if ((int64_t)start + (int64_t)payload * 8 > br.size_bits) {
            br.error = 1;
        } else {
            br.pos = start + payload * 8;
        }
    }
done:
    free(rbsp);
}
