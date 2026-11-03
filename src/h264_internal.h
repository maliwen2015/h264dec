/*
 * h264dec internal definitions.
 *
 * Algorithms and constant tables are derived from ITU-T H.264 and from
 * FFmpeg's H.264 decoder (LGPL-2.1+) which serves as the reference.
 */
#ifndef H264_INTERNAL_H
#define H264_INTERNAL_H

#define _WIN32_WINNT 0x0600

#include <stdint.h>
#include <stddef.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>

#ifdef _WIN32
#include <windows.h>
#include <malloc.h>
#else
#include <unistd.h>
#include <pthread.h>
#endif

#include "../h264dec.h"

/* ------------------------------------------------------------------ */
/* misc helpers                                                        */
/* ------------------------------------------------------------------ */
#define H264_MIN(a, b) ((a) < (b) ? (a) : (b))
#define H264_MAX(a, b) ((a) > (b) ? (a) : (b))
#define H264_MIN3(a,b,c) H264_MIN(H264_MIN(a,b),c)
#define H264_MAX3(a,b,c) H264_MAX(H264_MAX(a,b),c)
#define H264_CLIP(a, amin, amax) ((a) < (amin) ? (amin) : (a) > (amax) ? (amax) : (a))
#define H264_ABS(a) ((a) >= 0 ? (a) : -(a))
#define H264_SIGN(a) ((a) > 0 ? 1 : (a) < 0 ? -1 : 0)
#define H264_CLIP_PIXEL(a) H264_CLIP(a, 0, 255)
#define H264_ARRAY_ELEMS(a) (sizeof(a) / sizeof((a)[0]))
#define H264_ALIGN(x, a) (((x) + (a) - 1) & ~((a) - 1))

#ifdef _MSC_VER
#define H264_INLINE static __forceinline
#else
#define H264_INLINE static inline
#endif

/* Storage with one copy per thread. The loop-filter trace state is written by
 * whichever worker happens to hold a picture, so a single shared word there is a
 * data race on a build that only means to print. A compiler without a thread
 * keyword falls back to ordinary static storage: what is lost is that the trace
 * sequence counter mixes threads, nothing in the decoded bits. */
#if defined(_MSC_VER)
#  define H264_THREAD_LOCAL __declspec(thread)
#  define H264_HAVE_THREAD_LOCAL 1
#elif defined(__GNUC__) || defined(__clang__)
#  define H264_THREAD_LOCAL __thread
#  define H264_HAVE_THREAD_LOCAL 1
#else
#  define H264_THREAD_LOCAL
#  define H264_HAVE_THREAD_LOCAL 0
#endif

/* guard bytes in front of the intra4x4 mode rolling buffer (MBAFF reads
 * below slot 0 through mode[6 - left_block[]]) */
#define I4_MODE_PAD 64

/* Two 16-bit halves packed into one word. The shift is unsigned on purpose: a
 * half whose value is above 32767 - any negative motion vector - shifted left 16
 * in `int` overflows the type, which is undefined rather than the wrap every
 * target here performs, and UBSAN reports it once per packed neighbour. The
 * result type stays int32_t so the callers that read it back as two int16_t
 * halves keep exactly the bits they do today. */
#define H264_PACK16TO32(a, b) \
    ((int32_t)(((uint32_t)(a) & 0xffffu) | (((uint32_t)(b) & 0xffffu) << 16)))
#define H264_MV32(p) \
    ((int32_t)(((uint32_t)((const int16_t *)(p))[0] & 0xffff) | \
               (((uint32_t)((const int16_t *)(p))[1] & 0xffff) << 16)))

/* Fill a w x h grid of the 8-wide neighbour caches.  `stride` is in
 * elements (one cache row is 8 elements). */
#define H264_FILL_RECT(dst, w, h, stride, val, T)                 \
    do {                                                          \
        T *_d = (T *)(dst);                                       \
        int _y, _x;                                               \
        for (_y = 0; _y < (h); _y++)                              \
            for (_x = 0; _x < (w); _x++)                          \
                _d[_y * (stride) + _x] = (T)(val);                \
    } while (0)

/* Same rectangle fill addressed in bytes, as FFmpeg's fill_rectangle():
 * `w` and `stride` count elements of `size` bytes each. */
H264_INLINE void h264_fill_rectangle(void *vp, int w, int h, int stride,
                                     uint32_t val, int size)
{
    uint8_t *p = (uint8_t *)vp;
    uint16_t v16 = (uint16_t)val;
    uint32_t v32 = val;
    int x, y;

    w *= size;
    stride *= size;
    for (y = 0; y < h; y++)
        for (x = 0; x < w; x += size) {
            if (size == 1)
                p[y * stride + x] = (uint8_t)val;
            else if (size == 2)
                memcpy(p + y * stride + x, &v16, 2);
            else
                memcpy(p + y * stride + x, &v32, 4);
        }
}

/* block indices of the DC blocks inside the 48-block mb scratch buffer */
#define LUMA_DC_BLOCK_INDEX   48
#define CHROMA_DC_BLOCK_INDEX 49

H264_INLINE uint32_t h264_rn32(const void *p)
{
    uint32_t v;
    memcpy(&v, p, 4);
    return v;
}

H264_INLINE uint64_t h264_rn64(const void *p)
{
    uint64_t v;
    memcpy(&v, p, 8);
    return v;
}

/* unaligned multi-byte loads/stores and fills, as used by the reference */
#define AV_RN32A(p)         h264_rn32(p)
#define AV_RN64A(p)         h264_rn64(p)
#define AV_WN32A(p, v)      do { uint32_t _t = (uint32_t)(v); memcpy((p), &_t, 4); } while (0)
#define AV_WN64A(p, v)      do { uint64_t _t = (uint64_t)(v); memcpy((p), &_t, 8); } while (0)
#define AV_COPY32(d, s)     memcpy((d), &(s), 4)
#define AV_COPY64(d, s)     memcpy((d), &(s), 8)
#define AV_COPY128(d, s)    memcpy((d), &(s), 16)
H264_INLINE void h264_zero(void *p, int n)
{
    memset(p, 0, (size_t)n);
}

#define AV_ZERO32(p)        h264_zero((p), 4)
#define AV_ZERO64(p)        h264_zero((p), 8)
#define AV_ZERO128(p)       h264_zero((p), 16)

/* Environment-gated debug tracing.  The environment is read once, at open, into a
 * plain global: these gates are tested per macroblock, so the accessor has to be a
 * load and a test rather than a call - out of line it measured 1.0% of a one-slot
 * decode. h264_dbg_init() is idempotent and atomic, so two decoders opened at once
 * both see the values filled, with the same ones. */
typedef struct H264Dbg {
    int mv, mc, pred, dm, mb, lf, bits;   /* trace enables */
    int coef;                             /* dump dequantized coefficients too */
    int ps;                               /* trace parameter set references */
    int mbx, mby;                         /* traced macroblock position */
    int fn;                               /* traced frame_num, -1 matches all */
    int lag;                              /* stall progress reports this many
                                             "us" to widen any wait window */
} H264Dbg;

extern H264Dbg h264_dbg_cfg;
void h264_dbg_init(void);
H264_INLINE const H264Dbg *h264_dbg(void) { return &h264_dbg_cfg; }
/* true when the trace for flag `f` is on and `sl` is the traced position */
#define H264_DBG_HIT(f, sl) (h264_dbg()->f && h264_dbg()->mbx == (sl)->mb_x && \
                             h264_dbg()->mby == (sl)->mb_y)

H264_INLINE int h264_mid_pred(int a, int b, int c)
{
    if (a > b) {
        if (b > c)
            return b;
        else if (a > c)
            return c;
        else
            return a;
    } else {
        if (a > c)
            return a;
        else if (b > c)
            return c;
        else
            return b;
    }
}

#ifdef _WIN32
typedef CRITICAL_SECTION h264_mutex;
#define h264_mutex_init(m)     InitializeCriticalSection(m)
#define h264_mutex_destroy(m)  DeleteCriticalSection(m)
#define h264_mutex_lock(m)     EnterCriticalSection(m)
#define h264_mutex_unlock(m)   LeaveCriticalSection(m)
typedef CONDITION_VARIABLE h264_cond;
#define h264_cond_init(c)      InitializeConditionVariable(c)
#define h264_cond_destroy(c)   ((void)0)
#define h264_cond_wait(c, m)   SleepConditionVariableCS((c), (m), INFINITE)
#define h264_cond_signal(c)    WakeConditionVariable(c)
#define h264_cond_broadcast(c) WakeAllConditionVariable(c)
#define H264_THREAD_RET        unsigned __stdcall
typedef HANDLE h264_thread;
#else
typedef pthread_mutex_t h264_mutex;
#define h264_mutex_init(m)     pthread_mutex_init(m, NULL)
#define h264_mutex_destroy(m)  pthread_mutex_destroy(m)
#define h264_mutex_lock(m)     pthread_mutex_lock(m)
#define h264_mutex_unlock(m)   pthread_mutex_unlock(m)
typedef pthread_cond_t h264_cond;
#define h264_cond_init(c)      pthread_cond_init(c, NULL)
#define h264_cond_destroy(c)   pthread_cond_destroy(c)
#define h264_cond_wait(c, m)   pthread_cond_wait(c, m)
#define h264_cond_signal(c)    pthread_cond_signal(c)
#define h264_cond_broadcast(c) pthread_cond_broadcast(c)
#define H264_THREAD_RET        void *
typedef pthread_t h264_thread;
#endif

/* The lock and channel a picture's row progress is waited on. One per picture,
 * not one for the pool: FFmpeg's ff_thread_report_progress() takes the mutex of
 * the thread that *owns* the picture being published, so a publish contends only
 * with the threads awaiting that one picture. h264_progress_report() does the
 * same. It is a pointer so that an await holding a const H264Frame * can still
 * lock what it points at. */
typedef struct H264FrameSync {
    h264_mutex m;
    h264_cond c;
} H264FrameSync;

#ifdef _WIN32
#define h264_atomic_inc(p)     InterlockedIncrement((volatile LONG *)(p))
#define h264_atomic_dec(p)     InterlockedDecrement((volatile LONG *)(p))
#define h264_atomic_set(p, v)  InterlockedExchange((volatile LONG *)(p), (LONG)(v))
#define h264_atomic_add(p, v)  InterlockedExchangeAdd((volatile LONG *)(p), (LONG)(v))
#define h264_atomic_get(p)     InterlockedCompareExchange((volatile LONG *)(p), 0, 0)
#else
#define h264_atomic_inc(p)     __sync_add_and_fetch((p), 1)
#define h264_atomic_dec(p)     __sync_sub_and_fetch((p), 1)
#define h264_atomic_set(p, v)  __sync_lock_test_and_set((p), (v))
#define h264_atomic_add(p, v)  __sync_fetch_and_add((p), (v))
#define h264_atomic_get(p)     __sync_add_and_fetch((p), 0)
#endif

/* ------------------------------------------------------------------ */
/* bit reader                                                          */
/* ------------------------------------------------------------------ */
typedef struct H264BitReader {
    const uint8_t *buffer;   /* padded with at least 8 zero bytes      */
    int size_bits;
    int pos;                 /* bit position                           */
    int error;               /* set on overread                        */
} H264BitReader;

H264_INLINE void h264_br_init(H264BitReader *br, const uint8_t *buf, int size_bits)
{
    br->buffer   = buf;
    br->size_bits = size_bits;
    br->pos      = 0;
    br->error    = 0;
}

H264_INLINE int h264_br_bits_left(const H264BitReader *br)
{
    return br->size_bits - br->pos;
}

static inline uint64_t h264_load_be64(const uint8_t *p)
{
    uint64_t v;
    memcpy(&v, p, 8);
#ifdef _WIN32
    v = _byteswap_uint64(v);
#else
    v = __builtin_bswap64(v);
#endif
    return v;
}

static inline uint32_t h264_load_be32(const uint8_t *p)
{
    uint32_t v;
    memcpy(&v, p, 4);
#ifdef _WIN32
    v = _byteswap_ulong(v);
#else
    v = __builtin_bswap32(v);
#endif
    return v;
}

H264_INLINE unsigned h264_br_get_bits(H264BitReader *br, int n)
{
    unsigned v;
    int pos;
    if (n <= 0)
        return 0;
    pos = br->pos;
    if (n <= 32 && pos + 32 <= br->size_bits) {
        uint64_t w = h264_load_be64(br->buffer + (pos >> 3));
        v = (unsigned)((w << (pos & 7)) >> (64 - n));
        br->pos = pos + n;
        return v;
    }
    if (pos + n > br->size_bits) {
        int avail = br->size_bits - pos;
        br->error = 1;
        v = avail > 0 ? h264_br_get_bits(br, avail) : 0;
        br->pos = br->size_bits;
        /* nothing was available to pad with, and n can be 32: shifting by
         * the whole width would be undefined */
        if (avail <= 0)
            return 0;
        return v << (n - avail);
    }
    {
        int i;
        v = 0;
        for (i = 0; i < n; i++) {
            int p = pos + i;
            v = (v << 1) | ((br->buffer[p >> 3] >> (7 - (p & 7))) & 1);
        }
    }
    br->pos = pos + n;
    return v;
}

H264_INLINE unsigned h264_br_get_bits1(H264BitReader *br)
{
    return h264_br_get_bits(br, 1);
}

H264_INLINE void h264_br_skip(H264BitReader *br, int n)
{
    if (n < 0)
        return;
    if (br->pos + n > br->size_bits) {
        br->error = 1;
        br->pos = br->size_bits;
        return;
    }
    br->pos += n;
}

H264_INLINE void h264_br_align(H264BitReader *br)
{
    int rem = br->pos & 7;
    if (rem)
        h264_br_skip(br, 8 - rem);
}

H264_INLINE int h264_br_byte_aligned(const H264BitReader *br)
{
    return !(br->pos & 7);
}

H264_INLINE unsigned h264_br_get_ue(H264BitReader *br)
{
    int leading = 0;
    while (leading < 32 && !h264_br_get_bits1(br))
        leading++;
    if (leading >= 32) {
        br->error = 1;
        return 0;
    }
    return ((1u << leading) - 1) + h264_br_get_bits(br, leading);
}

H264_INLINE int h264_br_get_se(H264BitReader *br)
{
    unsigned v = h264_br_get_ue(br);
    return (v & 1) ? (int)((v + 1) >> 1) : -(int)(v >> 1);
}

H264_INLINE int h264_br_more_rbsp_data(const H264BitReader *br)
{
    int pos = br->pos;
    int last = br->size_bits - 1;
    if (pos >= br->size_bits)
        return 0;
    while (last > pos && !((br->buffer[last >> 3] >> (7 - (last & 7))) & 1))
        last--;
    return pos < last;
}

/* ------------------------------------------------------------------ */
/* H.264 constants / macroblock type flags (FFmpeg-compatible)         */
/* ------------------------------------------------------------------ */
#define H264_MAX_SPS_COUNT      32
#define H264_MAX_PPS_COUNT      256
#define H264_MAX_REF_FRAMES     16
#define H264_MAX_PICTURE_COUNT  36
#define H264_MAX_MMCO_COUNT     66
#define H264_MAX_SLICES         64
#define H264_MAX_LIST_SIZE      48

#define QP_MAX_NUM              (51 + 6 * 6)

#define MB_TYPE_INTRA4x4   (1 <<  0)
#define MB_TYPE_INTRA16x16 (1 <<  1)
#define MB_TYPE_INTRA_PCM  (1 <<  2)
#define MB_TYPE_16x16      (1 <<  3)
#define MB_TYPE_16x8       (1 <<  4)
#define MB_TYPE_8x16       (1 <<  5)
#define MB_TYPE_8x8        (1 <<  6)
#define MB_TYPE_INTERLACED (1 <<  7)
#define MB_TYPE_DIRECT2    (1 <<  8)
#define MB_TYPE_CODEC_SPECIFIC (1 << 9)   /* MB_TYPE_REF0 in FFmpeg */
#define MB_TYPE_REF0       MB_TYPE_CODEC_SPECIFIC
#define MB_TYPE_CBP        (1 << 10)
#define MB_TYPE_QUANT      (1 << 11)
#define MB_TYPE_P0L0       (1 << 12)
#define MB_TYPE_P1L0       (1 << 13)
#define MB_TYPE_P0L1       (1 << 14)
#define MB_TYPE_P1L1       (1 << 15)
#define MB_TYPE_L0         (MB_TYPE_P0L0 | MB_TYPE_P1L0)
#define MB_TYPE_L1         (MB_TYPE_P0L1 | MB_TYPE_P1L1)
#define MB_TYPE_L0L1       (MB_TYPE_L0   | MB_TYPE_L1)
#define MB_TYPE_SKIP       (1 << 17)
#define MB_TYPE_8x8DCT     0x01000000

#define IS_INTRA4x4(a)     ((a) & MB_TYPE_INTRA4x4)
#define IS_INTRA16x16(a)   ((a) & MB_TYPE_INTRA16x16)
#define IS_INTRA_PCM(a)    ((a) & MB_TYPE_INTRA_PCM)
#define IS_INTRA(a)        ((a) & 7)
#define IS_INTERLACED(a)   ((a) & MB_TYPE_INTERLACED)
#define IS_DIRECT(a)       ((a) & MB_TYPE_DIRECT2)
#define IS_16X16(a)        ((a) & MB_TYPE_16x16)
#define IS_16X8(a)         ((a) & MB_TYPE_16x8)
#define IS_8X16(a)         ((a) & MB_TYPE_8x16)
#define IS_8X8(a)          ((a) & MB_TYPE_8x8)
#define IS_8x8DCT(a)       ((a) & MB_TYPE_8x8DCT)
#define IS_SKIP(a)         ((a) & MB_TYPE_SKIP)
#define IS_REF0(a)         ((a) & MB_TYPE_REF0)
#define IS_INTER(a)        ((a) & (MB_TYPE_16x16 | MB_TYPE_16x8 | MB_TYPE_8x16 | MB_TYPE_8x8))
#define USES_LIST(a, list) ((a) & ((MB_TYPE_P0L0 | MB_TYPE_P1L0) << (2 * (list))))
#define IS_DIR(a, part, list) ((a) & (MB_TYPE_P0L0 << ((part) + 2 * (list))))

#define IS_SUB_8X8(a)      ((a) & MB_TYPE_16x16)
#define IS_SUB_8X4(a)      ((a) & MB_TYPE_16x8)
#define IS_SUB_4X8(a)      ((a) & MB_TYPE_8x16)
#define IS_SUB_4X4(a)      ((a) & MB_TYPE_8x8)
#define MB_FIELD(sl)       ((sl)->mb_field_decoding_flag)
#define MB_MBAFF(sl)       ((sl)->mb_mbaff)

/* slice types (H.264 slice_type values; SP/SI follow the spec numbering) */
enum {
    H264_SLICE_P = 0,
    H264_SLICE_B = 1,
    H264_SLICE_I = 2,
    H264_SLICE_SP = 3,
    H264_SLICE_SI = 4
};

enum { PICT_TOP_FIELD = 1, PICT_BOTTOM_FIELD = 2, PICT_FRAME = 3 };

enum {
    LIST_NOT_USED = -1,
    PART_NOT_AVAILABLE = -2
};

/* intra prediction modes */
enum {
    VERT_PRED = 0,
    HOR_PRED,
    DC_PRED,
    DIAG_DOWN_LEFT_PRED,
    DIAG_DOWN_RIGHT_PRED,
    VERT_RIGHT_PRED,
    HOR_DOWN_PRED,
    HOR_UP_PRED,
    VERT_LEFT_PRED
};
enum { DC_128_PRED8x8 = 4 };

/* dequant coeff init tables */
extern const uint8_t h264_dequant4_coeff_init[6][3];
extern const uint8_t h264_dequant8_coeff_init_scan[16];
extern const uint8_t h264_dequant8_coeff_init[6][6];
extern const uint8_t h264_quant_rem6[QP_MAX_NUM + 1];
extern const uint8_t h264_quant_div6[QP_MAX_NUM + 1];
extern const uint8_t h264_chroma_qp[7][QP_MAX_NUM + 1];
extern const uint8_t h264_golomb_to_intra4x4_cbp[48];
extern const uint8_t h264_golomb_to_inter_cbp[48];
extern const uint8_t h264_golomb_to_inter_cbp_gray[16];
extern const uint8_t h264_golomb_to_intra4x4_cbp_gray[16];
extern const uint8_t h264_chroma_dc_scan[4];
extern const uint8_t h264_chroma422_dc_scan[8];
extern const uint8_t h264_zigzag_scan[17];
extern const uint8_t h264_zigzag_direct[64];
extern const uint8_t h264_field_scan[17];
extern const uint8_t h264_field_scan8x8[65];
extern const uint8_t h264_field_scan8x8_cavlc[65];
extern const uint8_t h264_zigzag_scan8x8_cavlc[65];

extern const uint8_t h264_scan8[16 * 3 + 3];

/* macroblock type tables */
typedef struct H264IMbInfo {
    uint8_t type;
    int8_t pred_mode;
    int8_t cbp;
} H264IMbInfo;
typedef struct H264PMbInfo {
    uint16_t type;
    uint8_t partition_count;
} H264PMbInfo;

extern const H264IMbInfo h264_i_mb_type_info[26];
extern const H264PMbInfo h264_p_mb_type_info[5];
extern const H264PMbInfo h264_p_sub_mb_type_info[4];
extern const H264PMbInfo h264_b_mb_type_info[23];
extern const H264PMbInfo h264_b_sub_mb_type_info[13];

/* CABAC tables and engine (table data in h264_cabac_data.c) */
extern const uint8_t h264_cabac_tables[512 + 4 * 2 * 64 + 4 * 64 + 63];
#define H264_NORM_SHIFT_OFFSET 0
#define H264_LPS_RANGE_OFFSET 512
#define H264_MLPS_STATE_OFFSET 1024
#define H264_LAST_COEFF_FLAG_OFFSET_8x8_OFFSET 1280
extern const int8_t h264_cabac_context_init_I[1024][2];
extern const int8_t h264_cabac_context_init_PB[3][1024][2];

typedef struct H264CABAC {
    int low;
    int range;
    const uint8_t *bytestream_start;
    const uint8_t *bytestream;
    const uint8_t *bytestream_end;
} H264CABAC;

#define CABAC_BITS 16
#define CABAC_MASK ((1 << CABAC_BITS) - 1)

/* ------------------------------------------------------------------ */
/* Parameter sets                                                       */
/* ------------------------------------------------------------------ */
typedef struct H264SPS {
    int refcount;
    int valid;
    int sps_id;
    int profile_idc;
    int constraint_set_flags;
    int level_idc;

    int chroma_format_idc;
    int bit_depth_luma;
    int bit_depth_chroma;
    int transform_bypass;

    int log2_max_frame_num;
    int poc_type;
    int log2_max_poc_lsb;
    int delta_pic_order_always_zero_flag;
    int offset_for_non_ref_pic;
    int offset_for_top_to_bottom_field;
    int poc_cycle_length;
    int offset_for_ref_frame[256];

    int ref_frame_count;
    int gaps_in_frame_num_allowed_flag;
    int mb_width;
    int mb_height;
    int frame_mbs_only_flag;
    int mb_aff;
    int direct_8x8_inference_flag;
    int crop;
    int crop_left;
    int crop_right;
    int crop_top;
    int crop_bottom;

    int scaling_matrix_present;
    uint8_t scaling_matrix4[6][16];
    uint8_t scaling_matrix8[6][64];

    int vui_parameters_present_flag;
    int sar_num, sar_den;
    int timing_info_present_flag;
    uint32_t num_units_in_tick;
    uint32_t time_scale;
    int fixed_frame_rate_flag;
    int pic_struct_present_flag;
    int nal_hrd_parameters_present_flag;
    int vcl_hrd_parameters_present_flag;
    /* field widths the picture timing SEI reads its delays with */
    int cpb_removal_delay_length;
    int dpb_output_delay_length;
    int time_offset_length;
    int bitstream_restriction_flag;
    int num_reorder_frames;
    int max_dec_frame_buffering;
} H264SPS;

typedef struct H264PPS {
    int refcount;
    int valid;
    int pps_id;
    int sps_id;
    H264SPS *sps;

    int cabac;
    int pic_order_present;
    int slice_group_count;
    int ref_count[2];
    int weighted_pred;
    int weighted_bipred_idc;
    int init_qp;
    int init_qs;
    int chroma_qp_index_offset[2];
    int deblocking_filter_parameters_present;
    int constrained_intra_pred;
    int redundant_pic_cnt_present;
    int transform_8x8_mode;
    int chroma_qp_diff;

    uint8_t scaling_matrix4[6][16];
    uint8_t scaling_matrix8[6][64];

    uint8_t chroma_qp_table[2][QP_MAX_NUM + 1];
    uint32_t dequant4_coeff[6][QP_MAX_NUM + 1][16];
    uint32_t dequant8_coeff[6][QP_MAX_NUM + 1][64];
} H264PPS;

/* ------------------------------------------------------------------ */
/* Supplemental enhancement information                                 */
/* ------------------------------------------------------------------ */
/* Picture timing (D.2.5): the delays need the HRD field widths from the SPS,
 * so the payload is buffered until the picture it belongs to has an active SPS. */
typedef struct H264PicTiming {
    int cpb_removal_delay;    /* -1 when the stream signals no HRD */
    int dpb_output_delay;
    int pic_struct;           /* -1 when pic_struct_present_flag is 0 */
    int ct_type;              /* bit set per counting_type seen */
    int tc_valid;             /* the first clock timestamp was flagged */
    int tc_hours, tc_minutes, tc_seconds, tc_frames;
    int tc_drop;              /* cnt_dropped_flag */
} H264PicTiming;

typedef struct H264SEIPending {
    int pt_present;
    int pt_size;
    uint8_t pt_payload[256];
    int recovery_frame_cnt;   /* -1 when no recovery point message */
    int exact_match;
    int broken_recovery;
} H264SEIPending;

/* ------------------------------------------------------------------ */
/* Picture                                                              */
/* ------------------------------------------------------------------ */
typedef struct H264Frame {
    /* pixel planes (coded origin, including padding) */
    uint8_t *data[3];
    uint8_t *buf_base[3];
    int linesize[3];
    /* 4x4 block offsets from the macroblock origin, [0] frame strides and
     * [1] the doubled field-picture strides; they depend only on the picture
     * and are rebuilt by alloc_frame_buffers */
    int block_offset[2][48];
    int bit_depth;              /* 8 or 10; sample size is bit_depth > 8 ? 2 : 1 */
    int chroma_format_idc;      /* 0 = 4:0:0, 1 = 4:2:0, 2 = 4:2:2, 3 = 4:4:4 */
    int chroma_shift_x, chroma_shift_y;
    int chroma_width, chroma_height;
    int width, height;          /* coded dimensions */
    int disp_width, disp_height;
    int crop_left, crop_top, crop_right, crop_bottom;

    int mb_width, mb_height, mb_stride, b_stride;

    /* per-macroblock data with FFmpeg-compatible borders */
    uint8_t *meta_base;
    int meta_count;
    uint32_t *mb_type;
    int8_t *qscale_table;
    uint8_t (*nnz)[48];
    uint16_t *cbp_table;
    uint8_t *chroma_pred_mode;
    uint16_t *slice_table;
    uint8_t *list_counts;
    uint8_t *direct_table;

    int16_t (*motion_val_base[2])[2];
    int16_t (*motion_val[2])[2];
    int8_t *ref_index[2];
    uint8_t (*mvd[2])[8][2];

    uint8_t *mb_skip;                /* per MB */

    int8_t *mb_deblock_idc;          /* per MB */
    int8_t *mb_alpha;                /* per MB, slice_alpha_c0_offset */
    int8_t *mb_beta;                 /* per MB, slice_beta_offset */
    int (*ref2frm)[2][64];           /* per slice ref -> frame id mapping */
    int cabac;
    int intra_only;

    /* DPB / output state */
    int poc;
    int field_poc[2];
    /* Collocated reference lists, per field: used by B direct prediction. */
    int ref_list_count[2][2];              /* [field][list] */
    int ref_poc[2][2][H264_MAX_LIST_SIZE]; /* [field][list][ref] */
    int frame_num;
    int pic_id;
    int long_ref;
    int in_short_ref, in_long_ref;
    int reference;      /* field/frame reference bitmask (1/2/3) */
    int field_picture;
    int mbaff;
    int picture_structure;
    /* picture metadata, from SEI when the stream carries it */
    int pt_present;
    H264PicTiming pt;
    int recovery_frame_cnt;    /* -1 when no recovery point message */
    int exact_match;
    int broken_recovery;
    int sei_applied;           /* the four fields above are filled in */
    int interlaced;            /* coded as fields, see h264_picture_sei_props */
    int repeat_pict;
    int top_field_first;
    int output_pending;
    int output_sent;

    int qp;             /* first slice QP */
    int type;           /* H264_SLICE_* of first slice */
    int64_t pts, dts;
    int b_decode_error;

    uint8_t chroma_qp_table[2][QP_MAX_NUM + 1];
    int chroma_qp_diff;
    int disable_lf;

    int ref_count;      /* ownership count (DPB, current decoder, API) */
    int decode_done;    /* number of slices still to decode (atomic later) */
    int deblock_done;
    int deblock_row[2];   /* next MB row per field parity to deblock */
    struct H264Slice *filter_sl;   /* the filter pass's caches, one per picture */
    int recon[2];         /* MB rows the decode pass has finished, -1 = none */
    int filter_split;     /* rows are filtered by whoever is free, see below */
    int filter_done;      /* the filter chain has reached the last row */
    unsigned char filter_queued, filter_claimed;
    int progress[2];      /* last deblocked line per field parity, -1 = none */
    H264FrameSync *sync;  /* protects and wakes those two counters */
    int slice_count;
    int have_top_field, have_bottom_field;

    int64_t seq_epoch;  /* distinguishes IDR sequences for output reorder */
} H264Frame;

/* Reference list entry (points into a frame) */
typedef struct H264Ref {
    uint8_t *data[3];
    int linesize[3];
    int reference;      /* picture structure of the referenced picture */
    int poc;
    int pic_id;
    const H264Frame *parent;
} H264Ref;

/* MMCO */
typedef struct H264MMCO {
    uint8_t opcode;
    int short_pic_num;
    int long_arg;
} H264MMCO;

/* Weighted prediction tables */
typedef struct H264PredWeightTable {
    int use_weight;
    int use_weight_chroma;
    int luma_log2_weight_denom;
    int chroma_log2_weight_denom;
    int luma_weight_flag[2];
    int chroma_weight_flag[2];
    int luma_weight[48][2][2];
    int chroma_weight[48][2][2][2];
    int implicit_weight[48][48][2];
} H264PredWeightTable;

/* POC state */
typedef struct H264POCContext {
    int frame_num;
    int prev_frame_num;
    int prev_frame_num_offset;
    int prev_poc_msb;
    int prev_poc_lsb;
    int poc_lsb;
    int poc_msb;
    int delta_poc_bottom;
    int delta_poc[2];
    int frame_num_offset;
} H264POCContext;

/* ------------------------------------------------------------------ */
/* Slice context                                                       */
/* ------------------------------------------------------------------ */
typedef struct H264Decoder H264Decoder;

typedef struct H264Slice {
    H264Decoder *h;
    H264BitReader gb;

    int slice_num;
    int slice_type;
    int slice_type_nos;
    int slice_type_fixed;

    int qscale;
    int chroma_qp[2];
    int qp_thresh;
    int last_qscale_diff;

    int deblocking_filter;
    int slice_alpha_c0_offset;
    int slice_beta_offset;

    H264PredWeightTable pwt;

    int prev_mb_skipped;
    int next_mb_skipped;

    int chroma_pred_mode;
    int intra16x16_pred_mode;
    int8_t intra4x4_pred_mode_cache[48 + 8 * 8];
    int8_t *intra4x4_pred_mode;      /* 2 MB row rolling buffer, 8 bytes per MB */

    int topleft_mb_xy, top_mb_xy, topright_mb_xy;
    int left_mb_xy[2];
    int topleft_type, top_type, topright_type;
    int left_type[2];
    const uint8_t *left_block;
    int topleft_partition;
    unsigned int topleft_samples_available;
    unsigned int top_samples_available;
    unsigned int topright_samples_available;
    unsigned int left_samples_available;

    int mb_x, mb_y, mb_xy;
    int resync_mb_x, resync_mb_y;
    unsigned int first_mb_addr;
    int mb_skip_run;

    int picture_structure;
    int mb_field_decoding_flag;
    int mb_mbaff;

    int redundant_pic_count;
    int neighbor_transform_size;

    int direct_spatial_mv_pred;
    int col_parity;
    int col_fieldoff;

    int cbp;
    int top_cbp;
    int left_cbp;

    int dist_scale_factor[32];
    int dist_scale_factor_field[2][32];
    int map_col_to_list0[2][16 + 32];
    int map_col_to_list0_field[2][2][16 + 32];

    int ref_count[2];
    int list_count;
    H264Ref ref_list[2][H264_MAX_LIST_SIZE];

    struct {
        uint8_t op;
        uint32_t val;
    } ref_modifications[2][32];
    int nb_ref_modifications[2];

    unsigned int pps_id;
    const uint8_t *intra_pcm_ptr;

    uint8_t *edge_emu_buffer;
    uint8_t *bipred_scratchpad;
    int edge_emu_alloc, bipred_alloc;
    uint8_t *rbsp;
    int rbsp_size;
    int mb_linesize, mb_uvlinesize;
    int pixel_shift;              /* 1 for 10-bit, 0 for 8-bit */

    uint8_t non_zero_count_cache[15 * 8];
    int16_t mv_cache[2][5 * 8][2];
    int8_t ref_cache[2][5 * 8];
    uint8_t mvd_cache[2][5 * 8][2];
    uint8_t direct_cache[5 * 8];

    uint16_t sub_mb_type[4];

    int16_t mb[16 * 48 * 2];
    int16_t mb_luma_dc[3][16 * 2];   /* p0 luma DC; p1,p2 keep the CABAC 3-plane layout */

    uint8_t cabac_state[1024];
    H264CABAC cabac;
    int cabac_init_idc;

    H264MMCO mmco[H264_MAX_MMCO_COUNT];
    int nb_mmco;
    int explicit_ref_marking;

    int frame_num;
    int idr_pic_id;
    int poc_lsb;
    int delta_poc_bottom;
    int delta_poc[2];
    int curr_pic_num;
    int max_pic_num;

    int next_slice_idx;
    int nal_ref_idc;
    int nal_type;
    int mb_count;
    H264SPS *sps;
    H264PPS *pps;

    /* slice threading */
    int task_index;
    int done;
} H264Slice;

/* ------------------------------------------------------------------ */
/* Frame-level threading                                               */
/* ------------------------------------------------------------------ */
typedef struct H264Worker H264Worker;

/* In flight pictures pin their references in the 36 entry pool, so the payoff
 * per extra slot fades as the reconstruction runs out of memory bandwidth.
 * Measured with the pictures discarded (example_decoder.c takes "-" for that),
 * on the build with the loop filter on the shared queue: the 60s 720p clip does
 * 582 fps at one slot, 1992 at six, 2268 at eight, 2450 at ten and 2518 at
 * twelve; 1080p goes 268, 928, 1028, 1128 and 1159 over the same rungs, 4.3x
 * both ways. Six used to be the ceiling because eight was worse than six - that
 * was the pool-wide lock on the report path, not the picture pool. */
#define H264_MAX_JOB_SLOTS 12

/* one picture handed to a worker: the slices to decode plus the picture they
 * write into. The header phase (reference lists, MMCO, slice numbering) is
 * always done by the thread that owns the decoder, so a job only ever runs
 * the pixel work. */
enum {
    H264_JOB_FREE = 0,
    H264_JOB_OWNED,       /* reserved by the decoder owner, being filled in */
    H264_JOB_QUEUED,
    H264_JOB_RUNNING
};

typedef struct H264Job {
    H264Frame *f;
    H264Slice *slices[H264_MAX_SLICES];
    int nb_slices;
    int structure;
    int64_t seq;
    int state;
    int result;
} H264Job;

/* Rows the loop filter can take. A picture goes on the list when its decode pass
 * has finished a row that the filter has not caught up with, and any thread that
 * would otherwise block - waiting for a reference row, waiting for a job, waiting
 * for its own picture to be filtered - takes the list instead of sleeping. The
 * filter's caches live with the picture for exactly that reason, and one picture
 * is never filtered by two threads at once: the claim flag says who has it.
 * A member of the list holds a reference on its picture.
 *
 * claim_c is slept on under m by the thread that owns a picture whose claim is in
 * another thread's hands, and is broadcast under m whenever a claim is put down.
 * The picture's own decode pass must never wait for anything but a claim it cannot
 * take itself: if it did, the list would hold work with every thread asleep on it. */
typedef struct H264FilterQueue {
    h264_mutex m;
    h264_cond claim_c;
    H264Frame *items[H264_MAX_PICTURE_COUNT];
    int head, count;
} H264FilterQueue;

/* ------------------------------------------------------------------ */
/* Decoder                                                             */
/* ------------------------------------------------------------------ */
struct H264Decoder {
    h264_param_t param;
    h264_stream_info info;

    H264SPS *sps_list[H264_MAX_SPS_COUNT];
    H264PPS *pps_list[H264_MAX_PPS_COUNT];

    H264Frame *frame_pool[H264_MAX_PICTURE_COUNT];
    int frame_pool_count;

    H264Frame *cur_pic;        /* picture currently being decoded */
    H264Frame *last_pic;       /* previous picture (for fields) */

    H264Frame *short_ref[32];
    int short_ref_count;
    H264Frame *long_ref[32];
    int long_ref_count;

    H264Frame *delayed[H264_MAX_PICTURE_COUNT];
    int delayed_count;
    H264Frame *ready[H264_MAX_PICTURE_COUNT];
    int ready_count;
    H264Ref default_ref[2];
    int noref_gray, non_gray;

    H264POCContext poc;
    int poc_offset;
    int frame_recovered;

    int picture_structure;
    int first_field;
    int droppable;
    int picture_idr;
    int picture_intra_only;
    int missing_fields;
    int mmco_reset;
    int explicit_ref_marking;

    H264MMCO mmco[H264_MAX_MMCO_COUNT];
    int nb_mmco;

    H264Slice *slices[H264_MAX_SLICES];
    int nb_slices;
    int current_slice;
    int cur_first_mb;

    /* frame-level threading: one worker per job slot, each with its own
     * decoder copy for the pixel work */
    h264_mutex *mtx;         /* points at mutex below, shared with the clones */
    h264_cond *cv;
    H264Worker *workers;
    int nb_threads;
    int threaded;
    int64_t commit_next;
    int64_t job_seq;

    int mb_width, mb_height, mb_stride, b_stride;
    int width, height;         /* coded */
    int disp_width, disp_height;
    int chroma_y_shift;
    int context_initialized;
    /* Scan tables, re-derived whenever the active SPS changes: with
     * transform_bypass the q0 variants are untransposed (lossless). */
    uint8_t zigzag_scan[16];
    uint8_t field_scan[16];
    uint8_t zigzag_scan8x8[64];
    uint8_t field_scan8x8[64];
    uint8_t zigzag_scan8x8_cavlc[64];
    uint8_t field_scan8x8_cavlc[64];
    uint8_t zigzag_scan_q0[16];
    uint8_t field_scan_q0[16];
    uint8_t zigzag_scan8x8_q0[64];
    uint8_t field_scan8x8_q0[64];
    uint8_t zigzag_scan8x8_cavlc_q0[64];
    uint8_t field_scan8x8_cavlc_q0[64];
    /* x264 build from the SEI; -1 when unknown (matches the reference's
     * default, which the direct-mode workarounds compare against) */
    int x264_build;
    /* SEI messages of the access unit being parsed, applied to its picture */
    H264SEIPending sei;
    int prev_interlaced_frame;
    H264SPS *active_sps;
    H264PPS *active_pps;

    uint32_t *mb_type_base_storage; /* only used for frame allocations */
    uint32_t *mb2b_xy;
    uint32_t *mb2br_xy;
    int8_t *intra4x4_pred_mode;     /* rolling 2 MB row buffer, 8 bytes per MB */
    int64_t seq_epoch;
    int next_outputed_poc;
    int max_reorder;

    uint8_t *nal_buf;          /* rbsp buffer */
    int nal_buf_alloc;

    int got_header;            /* stream info changed -> report GOT_HEADER */
    int saw_idr;
    int eof;
    int error;
    int au_error;        /* a picture failed on a worker since the last call */

    h264_mutex mutex;
    h264_cond cond;
    H264FilterQueue filter_q;    /* embedded; the clones reach it through fp */
    H264FilterQueue *fp;
    int pool_stop;

    /* output queue is 'delayed' */
    int64_t last_pts, last_dts;
    int64_t packet_pts, packet_dts;
};

/* ------------------------------------------------------------------ */
/* functions                                                           */
/* ------------------------------------------------------------------ */
/* tables / ps */
void h264_init_scan_tables(H264Decoder *h);
H264SPS *h264_sps_ref(H264SPS *sps, const char *why);
void h264_sps_unref(H264SPS **sps, const char *why);
H264PPS *h264_pps_ref(H264PPS *pps, const char *why);
void h264_pps_unref(H264PPS **pps, const char *why);
int h264_decode_sps(H264Decoder *h, const uint8_t *rbsp, int size_bits, int nal_type);
int h264_decode_pps(H264Decoder *h, const uint8_t *rbsp, int size_bits);
void h264_pps_init_dequant(H264PPS *pps);

H264_INLINE int h264_get_chroma_qp(const H264PPS *pps, int t, int qscale)
{
    return pps->chroma_qp_table[t][qscale];
}
H264_INLINE int h264_frame_get_chroma_qp(const H264Frame *f, int t, int qscale)
{
    return f->chroma_qp_table[t][qscale];
}

/* nal */
typedef struct H264NAL {
    int type;
    int ref_idc;
    int forbidden;
    const uint8_t *data;      /* EBSP payload after header */
    int size;
    int header_bytes;
} H264NAL;
int h264_split_annexb(const uint8_t *data, int size, int *pos,
                      const uint8_t **nal, int *nal_len);
int h264_split_avcc(const uint8_t *data, int size, int *pos,
                    const uint8_t **nal, int *nal_len, int len_size);
int h264_unescape_rbsp(uint8_t *dst, int dst_size, const uint8_t *src, int src_len);
void h264_decode_sei(H264Decoder *h, const uint8_t *nal, int len);
void h264_sei_reset(H264Decoder *h);
int h264_sei_read_picture_timing(const H264SPS *sps, const uint8_t *payload,
                                 int size, H264PicTiming *out);

/* frame */
H264Frame *h264_alloc_frame(H264Decoder *h);
H264Frame *h264_frame_ref(H264Frame *f);
void h264_frame_unref(H264Decoder *h, H264Frame *f);
void h264_pool_free(H264Decoder *h);
void h264_free_frame_buffers(H264Frame *f);
int h264_frame_start(H264Decoder *h, const H264Slice *sl);
void h264_flush_frames(H264Decoder *h);
void h264_queue_output(H264Decoder *h, H264Frame *f);

/* dpb / refs */
void h264_clear_refs(H264Decoder *h);
void h264_mmco_reset(H264Decoder *h);
int h264_execute_ref_pic_marking(H264Decoder *h);
void h264_apply_ref_pic_marking(H264Decoder *h, H264Slice *sl, int idr);
int h264_init_poc(H264Decoder *h, int field_poc[2], int *poc, int picture_structure, int nal_ref_idc);
int h264_build_ref_list(H264Decoder *h, H264Slice *sl);
int h264_select_output_frame(H264Decoder *h);

/* slice */
int h264_decode_slice_header(H264Decoder *h, H264Slice *sl, const H264NAL *nal,
                             int nal_ref_idc, int nal_type);
int h264_decode_slice_data(H264Decoder *h, H264Slice *sl);
int h264_slice_init(H264Decoder *h, H264Slice *sl);
void h264_slice_release(H264Slice *sl);
int h264_decode_mb_cavlc(H264Decoder *h, H264Slice *sl);
int h264_decode_mb_cabac(H264Decoder *h, H264Slice *sl);
void h264_init_cabac_states(H264Decoder *h, H264Slice *sl);
int h264_field_start(H264Decoder *h, H264Slice *sl);
int h264_picture_end(H264Decoder *h);
void h264_picture_headers_end(H264Decoder *h);
void h264_picture_pixels(H264Decoder *h, H264Frame *f, int structure);
void h264_picture_output(H264Decoder *h, H264Frame *f, int structure);
void h264_cavlc_init(void);
int h264_activate_ps(H264Decoder *h, H264Slice *sl);
void h264_implicit_weight_table(H264Decoder *h, H264Slice *sl, int field);

/* mb / reconstruction */
void h264_hl_decode_mb(H264Decoder *h, H264Slice *sl);
void h264_decode_mb_skip(H264Decoder *h, H264Slice *sl);
void h264_write_back_motion(H264Decoder *h, H264Slice *sl, uint32_t mb_type);
void h264_write_back_non_zero_count(H264Decoder *h, H264Slice *sl);
void h264_write_back_intra_pred_mode(H264Decoder *h, H264Slice *sl);
void h264_direct_dist_scale_factor(H264Decoder *h, H264Slice *sl);
void h264_direct_ref_list_init(H264Decoder *h, H264Slice *sl);
void h264_pred_direct_motion(H264Decoder *h, H264Slice *sl, uint32_t *mb_type);
int h264_decode_mb_intra_pcm(H264Decoder *h, H264Slice *sl);

/* deblock */
void h264_deblock_upto(H264Decoder *h, H264Frame *f, int structure, int until);
void h264_filter_mb(H264Decoder *h, H264Slice *sl, H264Frame *f, int mb_x, int mb_y,
                    uint8_t *img_y, uint8_t *img_cb, uint8_t *img_cr,
                    int linesize, int uvlinesize);
/* 0 when the index the whole-macroblock loop-filter skip uses still matches the
 * alpha and tc0 tables; see LF_FILTER_INERT_INDEX in h264_deblock.c */
int h264_deblock_check_inert_index(void);

/* dsp: idct, intra, mc, pixel ops, edge emulation */
void h264_idct_add(uint8_t *dst, int16_t *block, int stride);
void h264_idct_dc_add(uint8_t *dst, int16_t *block, int stride);
void h264_idct8_add(uint8_t *dst, int16_t *block, int stride);
void h264_add_pixels4(uint8_t *dst, int16_t *block, int stride);
void h264_add_pixels8(uint8_t *dst, int16_t *block, int stride);
void h264_idct8_dc_add(uint8_t *dst, int16_t *block, int stride);
void h264_idct_add16(uint8_t *dst, const int *block_offset, int16_t *block, int stride, const uint8_t nnzc[5 * 8]);
void h264_idct_add16intra(uint8_t *dst, const int *block_offset, int16_t *block, int stride, const uint8_t nnzc[5 * 8]);
void h264_idct8_add4(uint8_t *dst, const int *block_offset, int16_t *block, int stride, const uint8_t nnzc[5 * 8]);
void h264_idct_add8(uint8_t **dest, const int *block_offset, int16_t *block, int stride, const uint8_t nnzc[15 * 8]);
void h264_luma_dc_dequant_idct(int16_t *output, int16_t *input, int qmul);
void h264_chroma_dc_dequant_idct(int16_t *block, int qmul);

void h264_pred4x4_dispatch(int mode, uint8_t *src, const uint8_t *topright, int stride);
void h264_pred8x8l_dispatch(int mode, uint8_t *src, int has_topleft, int has_topright, int stride);
void h264_pred8x8_dispatch(int mode, uint8_t *src, int stride);
void h264_pred16x16_dispatch(int mode, uint8_t *src, int stride);
/* lossless (transform_bypass) fused prediction + residual add, mode is
 * always V_PRED(0) or H_PRED(1) */
void h264_pred4x4_add(int mode, uint8_t *pix, int16_t *block, int stride);
void h264_pred8x8l_add(int mode, uint8_t *pix, int16_t *block, int stride);
void h264_pred8x8l_filter_add(int mode, uint8_t *src, int16_t *block,
                               int has_topleft, int has_topright, int stride);
void h264_pred16x16_add(int mode, uint8_t *pix, const int *block_offset,
                        int16_t *block, int stride);
void h264_pred8x8_add(int mode, uint8_t *pix, const int *block_offset,
                      int16_t *block, int stride);
int h264_check_intra4x4_pred_mode(int8_t *pred_mode_cache, int top_samples_available, int left_samples_available);
int h264_check_intra_pred_mode(int top_samples_available, int left_samples_available, int mode, int is_chroma);

/* 10-bit scalar pixel path: 16-bit samples, int32 coefficients, strides in
 * samples for the intra/idct entry points and in bytes for MC/deblock. */
void h264_idct_add_16(uint16_t *dst, int32_t *block, int stride);
void h264_idct_dc_add_16(uint16_t *dst, int32_t *block, int stride);
void h264_idct8_add_16(uint16_t *dst, int32_t *block, int stride);
void h264_idct8_dc_add_16(uint16_t *dst, int32_t *block, int stride);
void h264_add_pixels4_16(uint16_t *dst, int32_t *block, int stride);
void h264_add_pixels8_16(uint16_t *dst, int32_t *block, int stride);
void h264_idct_add16_16(uint16_t *dst, const int *block_offset, int32_t *block, int stride, const uint8_t nnzc[5 * 8]);
void h264_idct_add16intra_16(uint16_t *dst, const int *block_offset, int32_t *block, int stride, const uint8_t nnzc[5 * 8]);
void h264_idct8_add4_16(uint16_t *dst, const int *block_offset, int32_t *block, int stride, const uint8_t nnzc[5 * 8]);
void h264_idct_add8_16(uint16_t **dest, const int *block_offset, int32_t *block, int stride, const uint8_t nnzc[15 * 8]);
void h264_luma_dc_dequant_idct_16(int32_t *output, int32_t *input, int qmul);
void h264_chroma_dc_dequant_idct_16(int32_t *block, int qmul);
void h264_chroma422_dc_dequant_idct_16(int32_t *block, int qmul);
void h264_idct_add8_422_16(uint16_t **dest, const int *block_offset, int32_t *block, int stride, const uint8_t nnzc[15 * 8]);

void h264_pred4x4_dispatch_16(int mode, uint16_t *src, const uint16_t *topright, int stride);
void h264_pred8x8l_dispatch_16(int mode, uint16_t *src, int has_topleft, int has_topright, int stride);
void h264_pred8x8_dispatch_16(int mode, uint16_t *src, int stride);
void h264_pred8x16_dispatch_16(int mode, uint16_t *src, int stride);
void h264_pred16x16_dispatch_16(int mode, uint16_t *src, int stride);
void h264_pred4x4_add_16(int mode, uint16_t *pix, int32_t *block, int stride);
void h264_pred8x8l_add_16(int mode, uint16_t *pix, int32_t *block, int stride);
void h264_pred8x8l_filter_add_16(int mode, uint16_t *src, int32_t *block,
                                 int has_topleft, int has_topright, int stride);
void h264_pred16x16_add_16(int mode, uint16_t *pix, const int *block_offset,
                           int32_t *block, int stride);
void h264_pred8x8_add_16(int mode, uint16_t *pix, const int *block_offset,
                         int32_t *block, int stride);

void h264_emulated_edge_mc_16(uint8_t *buf, const uint8_t *src, int src_stride,
                              int w, int h, int x, int y, int pic_w, int pic_h);
void h264_qpel_mc_16(uint8_t *dst, const uint8_t *src, int stride,
                     int w, int h, int x, int y, int avg);
void h264_chroma_mc_16(uint8_t *dst, const uint8_t *src, int stride,
                       int w, int h, int x, int y, int avg);
void h264_hpel_mc_16(uint8_t *dst, const uint8_t *src, int stride,
                     int w, int h, int avg);
void h264_weight_pixels_16(uint8_t *dst, int stride, int w, int h,
                           int log2_denom, int weight, int offset);
void h264_biweight_pixels_16(uint8_t *dst, const uint8_t *src, int stride,
                             int w, int h, int log2_denom, int weightd,
                             int weights, int offset);
void h264_loop_filter_luma_16_c(uint8_t *pix, int xstride, int ystride, int inner_iters,
                                int alpha, int beta, const int8_t *tc0);
void h264_loop_filter_chroma_16_c(uint8_t *pix, int xstride, int ystride, int inner_iters,
                                  int alpha, int beta, const int8_t *tc0);
void h264_loop_filter_luma_intra_16_c(uint8_t *pix, int xstride, int ystride, int inner_iters,
                                      int alpha, int beta);
void h264_loop_filter_chroma_intra_16_c(uint8_t *pix, int xstride, int ystride, int inner_iters,
                                        int alpha, int beta);

void h264_emulated_edge_mc(uint8_t *buf, const uint8_t *src, int src_stride,
                           int w, int h, int x, int y, int pic_w, int pic_h);
void h264_qpel_mc(uint8_t *dst, const uint8_t *src, int stride,
                  int w, int h, int x, int y, int avg);
void h264_chroma_mc(uint8_t *dst, const uint8_t *src, int stride,
                    int w, int h, int x, int y, int avg);
/* SSE2 variants used on x86 whenever the compiler targets SSE2; both require
 * 0 < x + y and handle w/h of 2, 4, 8 or 16. */
void h264_idct_add_c(uint8_t *dst, int16_t *block, int stride);
void h264_idct_dc_add_c(uint8_t *dst, int16_t *block, int stride);
void h264_idct8_dc_add_c(uint8_t *dst, int16_t *block, int stride);
void h264_idct_add_sse2(uint8_t *dst, int16_t *block, int stride);
void h264_idct_dc_add_sse2(uint8_t *dst, int16_t *block, int stride);
void h264_idct8_dc_add_sse2(uint8_t *dst, int16_t *block, int stride);
void h264_qpel_mc_sse2(uint8_t *dst, const uint8_t *src, int stride,
                       int w, int h, int x, int y, int avg);
void h264_qpel_mc_avx2(uint8_t *dst, const uint8_t *src, int stride,
                       int w, int h, int x, int y, int avg);
void h264_loop_filter_luma_avx2(uint8_t *pix, int xstride, int ystride,
                                int inner_iters, int alpha, int beta,
                                const int8_t *tc0);
void h264_hpel_mc_sse2(uint8_t *dst, const uint8_t *src, int stride,
                       int w, int h, int avg);
/* in-loop deblocking pixel filters, one int16 lane per edge position; the C
 * versions are exported so that tests/dsp_check.c can compare the two */
void h264_loop_filter_luma_c(uint8_t *pix, int xstride, int ystride, int inner_iters,
                             int alpha, int beta, const int8_t *tc0);
void h264_loop_filter_chroma_c(uint8_t *pix, int xstride, int ystride, int inner_iters,
                               int alpha, int beta, const int8_t *tc0);
void h264_loop_filter_luma_intra_c(uint8_t *pix, int xstride, int ystride, int inner_iters,
                                   int alpha, int beta);
void h264_loop_filter_chroma_intra_c(uint8_t *pix, int xstride, int ystride, int inner_iters,
                                     int alpha, int beta);
void h264_loop_filter_luma_sse2(uint8_t *pix, int xstride, int ystride, int inner_iters,
                                int alpha, int beta, const int8_t *tc0);
void h264_loop_filter_chroma_sse2(uint8_t *pix, int xstride, int ystride, int inner_iters,
                                  int alpha, int beta, const int8_t *tc0);
void h264_loop_filter_luma_intra_sse2(uint8_t *pix, int xstride, int ystride, int inner_iters,
                                      int alpha, int beta);
void h264_loop_filter_chroma_intra_sse2(uint8_t *pix, int xstride, int ystride, int inner_iters,
                                        int alpha, int beta);
void h264_chroma_mc_sse2(uint8_t *dst, const uint8_t *src, int stride,
                         int w, int h, int x, int y, int avg);
/* the scalar reference versions, kept callable so that tests/dsp_check.c can
 * compare the two implementations sample by sample */
void h264_qpel_mc_c(uint8_t *dst, const uint8_t *src, int stride,
                    int w, int h, int x, int y, int avg);
void h264_chroma_mc_c(uint8_t *dst, const uint8_t *src, int stride,
                      int w, int h, int x, int y, int avg);
void h264_weight_pixels(uint8_t *dst, int stride, int w, int h,
                        int log2_denom, int weight, int offset);
void h264_biweight_pixels(uint8_t *dst, const uint8_t *src, int stride,
                          int w, int h, int log2_denom, int weightd,
                          int weights, int offset);
void h264_weight_pixels_c(uint8_t *dst, int stride, int w, int h,
                          int log2_denom, int weight, int offset);
void h264_biweight_pixels_c(uint8_t *dst, const uint8_t *src, int stride,
                            int w, int h, int log2_denom, int weightd,
                            int weights, int offset);
void h264_weight_pixels_sse2(uint8_t *dst, int stride, int w, int h,
                             int log2_denom, int weight, int offset);
void h264_biweight_pixels_sse2(uint8_t *dst, const uint8_t *src, int stride,
                               int w, int h, int log2_denom, int weightd,
                               int weights, int offset);
void h264_avg_pixels_sse2(uint8_t *dst, const uint8_t *src, int stride,
                          int w, int h);

/* mvpred (inline header) */
void h264_fill_decode_neighbors(H264Decoder *h, H264Slice *sl, uint32_t mb_type);
void h264_fill_decode_caches(H264Decoder *h, H264Slice *sl, uint32_t mb_type);
void h264_pred_motion(H264Decoder *h, H264Slice *sl, int n, int part_width,
                      int list, int ref, int *mx, int *my);
void h264_pred_16x8_motion(H264Decoder *h, H264Slice *sl, int n, int list, int ref,
                           int *mx, int *my);
void h264_pred_8x16_motion(H264Decoder *h, H264Slice *sl, int n, int list, int ref,
                           int *mx, int *my);

/* cabac engine */
int h264_cabac_init(H264Slice *sl, const uint8_t *buf, int size);
int h264_cabac_get(H264Slice *sl, uint8_t *state);
int h264_cabac_get_bypass(H264Slice *sl);
int h264_cabac_terminate(H264Slice *sl);
int h264_cabac_get_bypass_sign(H264Slice *sl, int val);

/* frame / output */
void h264_build_ref2frm(H264Decoder *h, H264Slice *sl);
void h264_queue_output(H264Decoder *h, H264Frame *f);

/* CPU feature detection, set once at load */
extern int h264_have_sse2;
extern int h264_have_avx2;
extern int h264_have_neon;

/* ------------------------------------------------------------------ */
/* SIMD ISA selection                                                   */
/* ------------------------------------------------------------------ */
/* Exactly one vector ISA per target: SSE2 (plus an AVX2 qpel kernel) on x86,
 * NEON on aarch64 and on any ARM core compiled with NEON enabled. Which one is
 * reachable is decided here at compile time, and whether it is used is decided
 * at load time by the flags above - both stay 0 when the CPU lacks the ISA or
 * when H264_DISABLE_SIMD=1, and then every dispatcher falls through to the
 * scalar C kernel. A target that is neither (ARM without NEON, RISC-V, ...)
 * compiles the vector files as empty translation units and has only the C path.
 *
 * The *_simd names exist so that tests/dsp_check.c can compare the scalar
 * reference against the vector kernel of whatever ISA this target actually
 * has, instead of hard-coding an x86 symbol it cannot link against. */
#if defined(__i386__) || defined(__x86_64__)
#  define H264_SIMD_X86 1
#  define H264_SIMD_NAME "sse2"
#elif defined(__ARM_NEON) || defined(__ARM_NEON__)
#  define H264_SIMD_ARM 1
#  define H264_SIMD_NAME "neon"
#endif

#if defined(H264_SIMD_X86) || defined(H264_SIMD_ARM)
#  define H264_SIMD 1
#endif

/* the one flag a dispatcher asks about: which vector ISA this target was built
 * for is already decided above, the flag only says whether the CPU may use it */
#if defined(H264_SIMD_ARM)
#  define H264_HAVE_SIMD h264_have_neon
#elif defined(H264_SIMD_X86)
#  define H264_HAVE_SIMD h264_have_sse2
#endif

/* the NEON counterparts of the SSE2 kernels, same signatures, same results */
void h264_qpel_mc_neon(uint8_t *dst, const uint8_t *src, int stride,
                       int w, int h, int x, int y, int avg);
void h264_chroma_mc_neon(uint8_t *dst, const uint8_t *src, int stride,
                         int w, int h, int x, int y, int avg);
void h264_hpel_mc_neon(uint8_t *dst, const uint8_t *src, int stride,
                       int w, int h, int avg);
void h264_avg_pixels_neon(uint8_t *dst, const uint8_t *src, int stride,
                          int w, int h);
void h264_weight_pixels_neon(uint8_t *dst, int stride, int w, int h,
                             int log2_denom, int weight, int offset);
void h264_biweight_pixels_neon(uint8_t *dst, const uint8_t *src, int stride,
                               int w, int h, int log2_denom, int weightd,
                               int weights, int offset);
void h264_idct_add_neon(uint8_t *dst, int16_t *block, int stride);
void h264_idct_dc_add_neon(uint8_t *dst, int16_t *block, int stride);
void h264_idct8_dc_add_neon(uint8_t *dst, int16_t *block, int stride);
void h264_loop_filter_luma_neon(uint8_t *pix, int xstride, int ystride,
                                int inner_iters, int alpha, int beta,
                                const int8_t *tc0);
void h264_loop_filter_chroma_neon(uint8_t *pix, int xstride, int ystride,
                                  int inner_iters, int alpha, int beta,
                                  const int8_t *tc0);
void h264_loop_filter_luma_intra_neon(uint8_t *pix, int xstride, int ystride,
                                      int inner_iters, int alpha, int beta);
void h264_loop_filter_chroma_intra_neon(uint8_t *pix, int xstride, int ystride,
                                        int inner_iters, int alpha, int beta);

#if defined(H264_SIMD_ARM)
#  define h264_qpel_mc_simd            h264_qpel_mc_neon
#  define h264_chroma_mc_simd          h264_chroma_mc_neon
#  define h264_hpel_mc_simd            h264_hpel_mc_neon
#  define h264_avg_pixels_simd         h264_avg_pixels_neon
#  define h264_weight_pixels_simd      h264_weight_pixels_neon
#  define h264_biweight_pixels_simd    h264_biweight_pixels_neon
#  define h264_idct_add_simd           h264_idct_add_neon
#  define h264_idct_dc_add_simd        h264_idct_dc_add_neon
#  define h264_idct8_dc_add_simd       h264_idct8_dc_add_neon
#  define h264_loop_filter_luma_simd   h264_loop_filter_luma_neon
#  define h264_loop_filter_chroma_simd h264_loop_filter_chroma_neon
#  define h264_loop_filter_luma_intra_simd   h264_loop_filter_luma_intra_neon
#  define h264_loop_filter_chroma_intra_simd h264_loop_filter_chroma_intra_neon
#elif defined(H264_SIMD_X86)
#  define h264_qpel_mc_simd            h264_qpel_mc_sse2
#  define h264_chroma_mc_simd          h264_chroma_mc_sse2
#  define h264_hpel_mc_simd            h264_hpel_mc_sse2
#  define h264_avg_pixels_simd         h264_avg_pixels_sse2
#  define h264_weight_pixels_simd      h264_weight_pixels_sse2
#  define h264_biweight_pixels_simd    h264_biweight_pixels_sse2
#  define h264_idct_add_simd           h264_idct_add_sse2
#  define h264_idct_dc_add_simd        h264_idct_dc_add_sse2
#  define h264_idct8_dc_add_simd       h264_idct8_dc_add_sse2
#  define h264_loop_filter_luma_simd   h264_loop_filter_luma_sse2
#  define h264_loop_filter_chroma_simd h264_loop_filter_chroma_sse2
#  define h264_loop_filter_luma_intra_simd   h264_loop_filter_luma_intra_sse2
#  define h264_loop_filter_chroma_intra_simd h264_loop_filter_chroma_intra_sse2
#endif

/* frame-level threading */
void h264_slice_bind_scratch(H264Decoder *h, H264Slice *sl);
int h264_thread_init(H264Decoder *h, int threads);
void h264_thread_close(H264Decoder *h);
void h264_decoder_lock(H264Decoder *h);
void h264_decoder_unlock(H264Decoder *h);
/* acquire/submit/wait_all require the decoder lock */
H264Job *h264_job_acquire(H264Decoder *h);
void h264_job_submit(H264Decoder *h, H264Job *job);
void h264_job_wait_last(H264Decoder *h);
void h264_job_wait_all(H264Decoder *h);
void h264_progress_report(H264Decoder *h, H264Frame *f, int line, int dir);
void h264_progress_await(H264Decoder *h, const H264Frame *f, int line, int dir);
/* the decode pass says how far it has got, and anyone blocked takes the filter
 * rows that this makes available instead of sleeping */
void h264_recon_report(H264Decoder *h, H264Frame *f, int row, int dir);
int h264_filter_help(H264Decoder *h);
void h264_filter_wait_done(H264Decoder *h, H264Frame *f);
/* pixel work of one picture, run on a worker clone */
int h264_run_job(H264Decoder *cl, H264Job *job);
/* bookkeeping that has to happen in decode order, run by the decoder owner */
void h264_commit_job(H264Decoder *h, H264Job *job);
void h264_slice_bind_scratch(H264Decoder *h, H264Slice *sl);
#endif /* H264_INTERNAL_H */
