/*
 * H.264 10-bit 4:2:0 motion compensation, weighted prediction and edge
 * emulation. The arithmetic is the 8-bit scalar path in h264_mc.c with 16-bit
 * samples and a 0..1023 clip; stride arguments stay in bytes, as everywhere
 * else in the frame code, and are shifted to samples at function entry.
 */
#include "h264_internal.h"

static inline uint16_t clip10(int v)
{
    return (uint16_t)(v < 0 ? 0 : v > 1023 ? 1023 : v);
}

static inline int rnd_avg10(int a, int b)
{
    return (a + b + 1) >> 1;
}

#if defined(__x86_64__)
#include <emmintrin.h>

/* 8-bit samples were the natural width for the SSE2 kernels; a 16-bit sample
 * needs only a straight load, and pavgw is exactly (a+b+1)>>1. */
static inline void copy_avg_row_16(uint16_t *dst, const uint16_t *src,
                                   int w, int avg)
{
    int i = 0;
    if (w >= 8) {
        for (; i + 8 <= w; i += 8) {
            __m128i v = _mm_loadu_si128((const __m128i *)(src + i));
            if (avg)
                v = _mm_avg_epu16(_mm_loadu_si128((const __m128i *)(dst + i)), v);
            _mm_storeu_si128((__m128i *)(dst + i), v);
        }
        for (; i < w; i++)
            dst[i] = avg ? (uint16_t)rnd_avg10(dst[i], src[i]) : src[i];
    } else if (w >= 4) {
        __m128i v = _mm_loadl_epi64((const __m128i *)src);
        if (avg)
            v = _mm_avg_epu16(_mm_loadl_epi64((const __m128i *)dst), v);
        _mm_storel_epi64((__m128i *)dst, v);
    } else {
        for (; i < w; i++)
            dst[i] = avg ? (uint16_t)rnd_avg10(dst[i], src[i]) : src[i];
    }
}
#else
static inline void copy_avg_row_16(uint16_t *dst, const uint16_t *src,
                                   int w, int avg)
{
    int i;
    for (i = 0; i < w; i++)
        dst[i] = avg ? (uint16_t)rnd_avg10(dst[i], src[i]) : src[i];
}
#endif

/* ------------------------------------------------------------------ */
/* Edge emulation                                                      */
/* ------------------------------------------------------------------ */
void h264_emulated_edge_mc_16(uint8_t *buf, const uint8_t *src, int stride,
                              int block_w, int block_h, int src_x, int src_y,
                              int pic_w, int pic_h)
{
    uint16_t *b = (uint16_t *)buf;
    const uint16_t *s = (const uint16_t *)src;
    int st = stride >> 1;
    int x, y;
    int start_y, start_x, end_y, end_x;

    if (!pic_w || !pic_h)
        return;

    if (src_y >= pic_h) {
        s -= src_y * st;
        s += (pic_h - 1) * st;
        src_y = pic_h - 1;
    } else if (src_y <= -block_h) {
        s -= src_y * st;
        s += (1 - block_h) * st;
        src_y = 1 - block_h;
    }
    if (src_x >= pic_w) {
        s -= (1 + src_x - pic_w);
        src_x = pic_w - 1;
    } else if (src_x <= -block_w) {
        s += (1 - block_w - src_x);
        src_x = 1 - block_w;
    }

    start_y = H264_MAX(0, -src_y);
    start_x = H264_MAX(0, -src_x);
    end_y = H264_MIN(block_h, pic_h - src_y);
    end_x = H264_MIN(block_w, pic_w - src_x);

    if (start_y >= end_y || start_x >= end_x)
        return;

    {
        int w = end_x - start_x;
        s += start_y * st + start_x;
        b += start_x;

        for (y = 0; y < start_y; y++) {
            memcpy(b, s, (size_t)w * 2);
            b += st;
        }
        for (; y < end_y; y++) {
            memcpy(b, s, (size_t)w * 2);
            s += st;
            b += st;
        }
        s -= st;
        for (; y < block_h; y++) {
            memcpy(b, s, (size_t)w * 2);
            b += st;
        }
        b -= block_h * st + start_x;
        while (block_h--) {
            for (x = 0; x < start_x; x++)
                b[x] = b[start_x];
            for (x = end_x; x < block_w; x++)
                b[x] = b[end_x - 1];
            b += st;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Quarter-pel interpolation                                           */
/* ------------------------------------------------------------------ */
static inline int hp_h_raw(const uint16_t *p)
{
    return (p[0] + p[1]) * 20 - (p[-1] + p[2]) * 5 + (p[-2] + p[3]);
}

static inline int hp_v_raw(const uint16_t *p, int stride)
{
    return (p[0] + p[stride]) * 20 - (p[-stride] + p[2 * stride]) * 5 +
           (p[-2 * stride] + p[3 * stride]);
}

static inline int hp_h(const uint16_t *p)
{
    return clip10((hp_h_raw(p) + 16) >> 5);
}

static inline int hp_v(const uint16_t *p, int stride)
{
    return clip10((hp_v_raw(p, stride) + 16) >> 5);
}

static inline int hp_hv(const uint16_t *p, int stride)
{
    int bm2 = hp_h_raw(p - 2 * stride);
    int bm1 = hp_h_raw(p - 1 * stride);
    int b0  = hp_h_raw(p);
    int b1  = hp_h_raw(p + 1 * stride);
    int b2  = hp_h_raw(p + 2 * stride);
    int b3  = hp_h_raw(p + 3 * stride);
    int v = (b0 + b1) * 20 - (bm1 + b2) * 5 + (bm2 + b3);
    return clip10((v + 512) >> 10);
}

void h264_qpel_mc_16(uint8_t *dst8, const uint8_t *src8, int stride,
                     int w, int h, int x, int y, int avg)
{
    uint16_t *dst = (uint16_t *)dst8;
    const uint16_t *src = (const uint16_t *)src8;
    int st = stride >> 1;
    int i, j;

    if (x == 0 && y == 0) {
        for (j = 0; j < h; j++) {
            copy_avg_row_16(dst, src, w, avg);
            dst += st;
            src += st;
        }
        return;
    }

    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i++) {
            const uint16_t *p = src + i;
            int v;
            switch (x + 4 * y) {
            case 1:  v = rnd_avg10(p[0], hp_h(p)); break;
            case 2:  v = hp_h(p); break;
            case 3:  v = rnd_avg10(p[1], hp_h(p)); break;
            case 4:  v = rnd_avg10(p[0], hp_v(p, st)); break;
            case 5:  v = rnd_avg10(hp_h(p), hp_v(p, st)); break;
            case 6:  v = rnd_avg10(hp_h(p), hp_hv(p, st)); break;
            case 7:  v = rnd_avg10(hp_h(p), hp_v(p + 1, st)); break;
            case 8:  v = hp_v(p, st); break;
            case 9:  v = rnd_avg10(hp_v(p, st), hp_hv(p, st)); break;
            case 10: v = hp_hv(p, st); break;
            case 11: v = rnd_avg10(hp_v(p + 1, st), hp_hv(p, st)); break;
            case 12: v = rnd_avg10(p[st], hp_v(p, st)); break;
            case 13: v = rnd_avg10(hp_h(p + st), hp_v(p, st)); break;
            case 14: v = rnd_avg10(hp_h(p + st), hp_hv(p, st)); break;
            case 15: v = rnd_avg10(hp_h(p + st), hp_v(p + 1, st)); break;
            default: v = p[0]; break;
            }
            dst[i] = avg ? (uint16_t)rnd_avg10(dst[i], v) : (uint16_t)v;
        }
        dst += st;
        src += st;
    }
}

/* ------------------------------------------------------------------ */
/* Chroma interpolation (bilinear)                                     */
/* ------------------------------------------------------------------ */
void h264_chroma_mc_16(uint8_t *dst8, const uint8_t *src8, int stride,
                       int w, int h, int x, int y, int avg)
{
    uint16_t *dst = (uint16_t *)dst8;
    const uint16_t *src = (const uint16_t *)src8;
    int st = stride >> 1;
    const int A = (8 - x) * (8 - y);
    const int B = x * (8 - y);
    const int C = (8 - x) * y;
    const int D = x * y;
    int i, j;

    if (!x && !y) {
        for (j = 0; j < h; j++) {
            copy_avg_row_16(dst, src, w, avg);
            dst += st;
            src += st;
        }
        return;
    }

    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i++) {
            int v = (A * src[i] + B * src[i + 1] +
                     C * src[st + i] + D * src[st + i + 1] + 32) >> 6;
            dst[i] = avg ? (uint16_t)rnd_avg10(dst[i], v) : (uint16_t)v;
        }
        dst += st;
        src += st;
    }
}

/* ------------------------------------------------------------------ */
/* Full-pel copy and average                                           */
/* ------------------------------------------------------------------ */
void h264_hpel_mc_16(uint8_t *dst8, const uint8_t *src8, int stride,
                     int w, int h, int avg)
{
    h264_qpel_mc_16(dst8, src8, stride, w, h, 0, 0, avg);
}

/* ------------------------------------------------------------------ */
/* Weighted prediction                                                 */
/* ------------------------------------------------------------------ */
void h264_weight_pixels_16(uint8_t *dst8, int stride, int w, int h,
                           int log2_denom, int weight, int offset)
{
    uint16_t *dst = (uint16_t *)dst8;
    int st = stride >> 1;
    int i, j;

    if (weight == (1 << log2_denom) && offset == 0)
        return;
    offset = (unsigned)offset << (log2_denom + 2);
    if (log2_denom)
        offset += 1 << (log2_denom - 1);
    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i++)
            dst[i] = clip10((dst[i] * weight + offset) >> log2_denom);
        dst += st;
    }
}

void h264_biweight_pixels_16(uint8_t *dst8, const uint8_t *src8, int stride,
                             int w, int h, int log2_denom, int weightd,
                             int weights, int offset)
{
    uint16_t *dst = (uint16_t *)dst8;
    const uint16_t *src = (const uint16_t *)src8;
    int st = stride >> 1;
    int i, j;

    if (weightd == weights && weightd == (1 << log2_denom) && offset == 0) {
        for (j = 0; j < h; j++) {
            copy_avg_row_16(dst, src, w, 1);
            dst += st;
            src += st;
        }
        return;
    }
    offset = (unsigned)offset << 2;
    offset = (unsigned)((offset + 1) | 1) << log2_denom;
    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i++)
            dst[i] = clip10((src[i] * weights + dst[i] * weightd + offset) >>
                            (log2_denom + 1));
        dst += st;
        src += st;
    }
}
