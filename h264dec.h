/*
 * h264dec - a self-contained H.264/AVC decoder (8-bit 4:2:0 and 10-bit
 * 4:0:0 / 4:2:0 / 4:2:2 / 4:4:4)
 *
 * Public API. The interface deliberately mirrors the avsplusdec API.
 *
 * The decoder is an independent implementation whose algorithms were
 * derived from the H.264 specification (ITU-T Rec. H.264 / ISO/IEC 14496-10)
 * and from the FFmpeg H.264 decoder (LGPL-2.1+) used as a reference
 * implementation. See README.md for details.
 */
#ifndef H264DEC_H
#define H264DEC_H

#include <stdint.h>

/* Return codes */
#define H264_DEC_ERROR       -1   /* decoding error */
#define H264_DEC_DEFAULT      0   /* no output available */
#define H264_DEC_GOT_FRAME    1   /* got a decoded picture */
#define H264_DEC_GOT_HEADER   2   /* got a new sequence header */
#define H264_DEC_END          3   /* end of stream */

/* Stream information */
struct h264_stream_info
{
    int profile;
    int level;
    int width;                 /* display width (cropping applied) */
    int height;                /* display height */
    int coded_width;
    int coded_height;
    int chroma_format;         /* 0 = 4:0:0, 1 = 4:2:0, 2 = 4:2:2, 3 = 4:4:4 */
    int bit_depth;             /* 8 or 10 */
    int frame_rate_num;
    int frame_rate_den;
    int bitrate;
    int progressive_seq;       /* 0 if the sequence uses interlaced coding */
    int sar_width;
    int sar_height;
    int max_num_reorder_frames;
    int max_dec_frame_buffering;
};
typedef struct h264_stream_info h264_stream_info;

/* Input packet: one or more whole access units of an Annex-B byte stream, or of
 * a length-prefixed (AVCC) stream when nal_length_size is set. A picture is
 * finalised at the end of the packet, so the slices of one multi-slice picture
 * may not be split across two calls: feeding one NAL at a time is only safe when
 * every slice is also the first slice of its picture. */
struct h264_packet_t
{
    const uint8_t* data;
    int len;
    int64_t pts;
    int64_t dts;
    void* opaque;
};
typedef struct h264_packet_t h264_packet_t;

/* Decoded picture */
struct h264_picture_t
{
    uint8_t* planes[3];        /* plane pointers, top-left of displayed area */
    int widths[3];             /* displayed width of each plane */
    int lines[3];              /* displayed height of each plane */
    int strides[3];
    int pic_order_count;
    int type;                  /* 1 = I, 2 = P, 3 = B (slice type) */
    int qp;
    int64_t pts;
    int64_t dts;
    int num_planes;            /* 1 for monochrome, 3 when chroma is present */
    int bytes_per_sample;      /* 1 for 8-bit, 2 for 10-bit little-endian */
    int bit_depth;             /* 8 or 10 */
    int b_decode_error;
    int top_field_first;       /* 1 = the picture starts with the top field */
    int interlaced;            /* 1 = coded as fields (or signalled so by SEI) */
    int repeat_pict;           /* extra fields of display time (0,1,2,4) */
    /* Picture metadata from SEI messages; the *_delay and pic_struct fields
     * are -1 when the stream does not carry the message. */
    int pic_struct;            /* D.3.2.3, -1 when pic_struct is not signalled */
    int cpb_removal_delay;     /* -1 when the stream signals no HRD */
    int dpb_output_delay;
    int recovery_frame_cnt;    /* recovery point distance, -1 when absent:
                                  0 on an IDR means a closed GOP entry point */
    int broken_recovery;       /* 1 = the recovery point warns of damaged input */
    int timecode_valid;        /* first clock timestamp of the picture */
    int timecode_hours;
    int timecode_minutes;
    int timecode_seconds;
    int timecode_frames;
    int timecode_drop;         /* 1 = SMPTE drop frame counting */
    void* dec_frame;
    void* opaque;
};
typedef struct h264_picture_t h264_picture_t;

/* Decoder parameters */
struct h264_param_t
{
    int threads;               /* worker threads (0/1 = no extra threads,
                                  larger values clamped to 12; 8-10 is where a
                                  720p-class stream stops paying) */
    int output_order;          /* 0 = display order, 1 = decode order */
    int disable_lf;            /* 1 = skip deblocking filter */
    int skip_non_ref;          /* 1 = do not output non-reference frames */
    int nal_length_size;       /* 0 = Annex-B start codes (default),
                                  otherwise 1/2/4 byte NAL length prefix (AVCC) */
};
typedef struct h264_param_t h264_param_t;

#ifdef H264DECLIB_SHARED
#define H264DEC_EXPORT __attribute__((visibility("default")))
#else
#define H264DEC_EXPORT
#endif

H264DEC_EXPORT void* h264_dec_open(h264_param_t* param);
H264DEC_EXPORT int   h264_dec_send_packet(void* decoder, h264_packet_t* packet);
/* Hand out the next picture in display order, filling the geometry, the frame
 * order flags and the SEI-derived metadata. The caller owns one reference to the
 * picture: keep it until h264_dec_frame_unref, and unref every picture before
 * h264_dec_close, which releases the buffers the decoder owns.
 *
 * The decoder keeps a fixed pool of 36 pictures and unreleased output counts
 * against it, so a caller that holds them all eventually makes the decoder drop
 * input pictures rather than block; cycling a window of a few frames is safe and
 * produces byte-identical output (tests/api_check.c asserts exactly that). */
H264DEC_EXPORT int   h264_dec_recv_frame(void* decoder, h264_stream_info* headerset, h264_picture_t* out_frame);
H264DEC_EXPORT int   h264_dec_flush(void* decoder, h264_stream_info* headerset, h264_picture_t* out_frame);
H264DEC_EXPORT void  h264_dec_frame_unref(void* decoder, h264_picture_t* out_frame);
H264DEC_EXPORT void  h264_dec_close(void* decoder);

#endif /* H264DEC_H */
