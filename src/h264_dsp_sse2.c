/*
 * SSE2 kernels for the three costs that measured as worth it: motion
 * compensation, the inverse transforms and the in-loop deblocking filter
 * (8-bit 4:2:0).
 *
 * Each is a translation of the scalar reference in h264_mc.c, h264_idct.c or
 * h264_deblock.c - the 6-tap half-pel filter, the bilinear chroma filter, the
 * integer transforms and the edge filters. The file is compiled for SSE2 on
 * its own and reached only when h264_have_sse2 says the running CPU has it;
 * other targets keep the scalar path.
 *
 * Every kernel works on 8 samples at a time. Blocks are 2, 4, 8 or 16 samples
 * wide: a narrow block still loads 8 source samples (the picture planes carry
 * a 16-pixel margin and the edge-emulation buffer is oversized) but stores
 * only its own bytes, so no scalar tail loop is needed.
 */
#include "h264_internal.h"

#if defined(__i386__) || defined(__x86_64__)

/* Compiled for SSE2 whatever the rest of the library is built with, so one
 * binary runs on a plain x86-64 CPU; h264_have_sse2 decides at run time
 * whether these are reachable. */
#pragma GCC push_options
#pragma GCC target("sse2")

#include <emmintrin.h>

#define ZERO   _mm_setzero_si128()
#define C20    _mm_set1_epi16(20)
#define C5     _mm_set1_epi16(5)
#define C16    _mm_set1_epi16(16)
#define ONE    _mm_set1_epi16(1)
#define MASK_FF _mm_set1_epi16(0xFF)
#define U8(p)  _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i *)(p)), ZERO)

/* 8 unrounded horizontal 6-tap results for output columns p[0..7] */
static inline __m128i hfilter8_u(const uint8_t *p)
{
    __m128i acc = _mm_mullo_epi16(_mm_add_epi16(U8(p), U8(p + 1)), C20);
    acc = _mm_sub_epi16(acc, _mm_mullo_epi16(_mm_add_epi16(U8(p - 1), U8(p + 2)), C5));
    return _mm_add_epi16(acc, _mm_add_epi16(U8(p - 2), U8(p + 3)));
}

/* 8 unrounded vertical 6-tap results for output columns p[0..7] */
static inline __m128i vfilter8_u(const uint8_t *p, int stride)
{
    __m128i acc = _mm_mullo_epi16(_mm_add_epi16(U8(p), U8(p + stride)), C20);
    acc = _mm_sub_epi16(acc, _mm_mullo_epi16(_mm_add_epi16(U8(p - stride),
                                                           U8(p + 2 * stride)), C5));
    return _mm_add_epi16(acc, _mm_add_epi16(U8(p - 2 * stride), U8(p + 3 * stride)));
}

#define RND(v)    _mm_srai_epi16(_mm_add_epi16(v, C16), 5)
/* clip to a sample range, then average: the scalar path clips every half-pel
 * result before it combines two of them, so a single clamp at the end would
 * give different values */
#define CLIP8(v)  _mm_min_epi16(_mm_max_epi16(v, ZERO), _mm_set1_epi16(255))
#define AVG16(a, b) \
    _mm_srli_epi16(_mm_add_epi16(_mm_add_epi16(CLIP8(a), CLIP8(b)), ONE), 1)

/* round to byte range, optionally average onto dst, store the first w lanes */
static inline void put8(uint8_t *dst, __m128i v, int w, int avg)
{
    __m128i b;
    if (avg) {
        /* the scalar path clips every half-pel sample to 0..255 before it
         * averages, so the clamp has to happen here as well */
        __m128i d = U8(dst);
        v = _mm_min_epi16(_mm_max_epi16(v, ZERO), _mm_set1_epi16(255));
        v = _mm_srli_epi16(_mm_add_epi16(_mm_add_epi16(v, d), ONE), 1);
    }
    b = _mm_packus_epi16(v, v);
    if (w >= 8)
        _mm_storel_epi64((__m128i *)dst, b);
    else if (w >= 4)
        *(int32_t *)dst = _mm_cvtsi128_si32(b);
    else
        *(int16_t *)dst = (int16_t)_mm_cvtsi128_si32(b);
}

static void qpel_h_sse2(uint8_t *dst, const uint8_t *src, int stride,
                        int w, int h, int x, int avg)
{
    int j, i;
    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i += 8) {
            const uint8_t *p = src + i;
            __m128i hp = RND(hfilter8_u(p));
            __m128i v = (x == 2) ? hp : AVG16(U8(p + (x >> 1)), hp);
            put8(dst + i, v, w - i, avg);
        }
        dst += stride;
        src += stride;
    }
}

static void qpel_v_sse2(uint8_t *dst, const uint8_t *src, int stride,
                        int w, int h, int y, int avg)
{
    int j, i;
    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i += 8) {
            const uint8_t *p = src + i;
            __m128i hp = RND(vfilter8_u(p, stride));
            __m128i v = (y == 2) ? hp
                                 : AVG16(U8(p + (y == 3 ? stride : 0)), hp);
            put8(dst + i, v, w - i, avg);
        }
        dst += stride;
        src += stride;
    }
}

/* Horizontal pass results are up to ~11000, and the vertical combination of
 * six of them needs ~440000, so the second pass runs on int32 lanes. The
 * multiplies by 20 and 5 become shift/add pairs, keeping this SSE2-only.
 * U4 sign-extends four int16 samples into four int32 lanes. */
#define U4(row, off)                                                    \
    _mm_unpacklo_epi16(_mm_loadl_epi64((const __m128i *)((row) + (off))), \
                       _mm_srai_epi16(_mm_loadl_epi64((const __m128i *)((row) + (off))), 15))
#define TWENTY(x) _mm_add_epi32(_mm_slli_epi32(x, 4), _mm_slli_epi32(x, 2))
#define FIVE(x)   _mm_add_epi32(_mm_slli_epi32(x, 2), x)

static inline __m128i vsum32_4(const int16_t *p, int n)
{
    __m128i a = U4(p, 0), b = U4(p + n, 0), c = U4(p - n, 0);
    __m128i d = U4(p + 2 * n, 0), e = U4(p - 2 * n, 0), f = U4(p + 3 * n, 0);
    __m128i acc = _mm_add_epi32(TWENTY(_mm_add_epi32(a, b)), e);
    acc = _mm_sub_epi32(acc, FIVE(_mm_add_epi32(c, d)));
    return _mm_add_epi32(acc, f);
}

/* 8 unrounded vertical results, as two int32 vectors */
static inline void vsum32_8(const int16_t *p, int n, __m128i *lo, __m128i *hi)
{
    const __m128i k512 = _mm_set1_epi32(512);
    __m128i a = vsum32_4(p, n), b = vsum32_4(p + 4, n);
    *lo = _mm_srai_epi32(_mm_add_epi32(a, k512), 10);
    *hi = _mm_srai_epi32(_mm_add_epi32(b, k512), 10);
}

/* horizontal pass width: 8 output lanes plus the extra chunk a 16-wide block
 * needs when the vertical pass reads one column to the right */
#define HW_W 24

static void qpel_hv_sse2(uint8_t *dst, const uint8_t *src, int stride,
                         int w, int h, int x, int y, int avg)
{
    int16_t tmp[(16 + 5) * HW_W];
    int j, i;

    /* unrounded horizontal filter of the source rows the vertical pass needs
     * (output row j uses rows j-2 .. j+3), stored with row -2 at tmp[0] */
    for (j = -2; j < h + 3; j++) {
        int16_t *row = tmp + (j + 2) * HW_W;
        for (i = 0; i < HW_W; i += 8)
            _mm_storeu_si128((__m128i *)(row + i),
                             hfilter8_u(src + j * stride + i));
    }

    for (j = 0; j < h; j++) {
        const int16_t *t = tmp + (j + 2) * HW_W;   /* source row j */
        for (i = 0; i < w; i += 8) {
            const uint8_t *p = src + i;
            __m128i hlo, hhi, hv;
            __m128i hh = RND(_mm_loadu_si128((const __m128i *)
                                             (t + (y >> 1) * HW_W + i)));
            vsum32_8(t + i, HW_W, &hlo, &hhi);
            hv = _mm_packs_epi32(hlo, hhi);
            if (x == 2 && y == 2) {
                put8(dst + i, hv, w - i, avg);
            } else if (x == 2) {
                put8(dst + i, AVG16(hh, hv), w - i, avg);
            } else if (y == 2) {
                __m128i vv = RND(vfilter8_u(p + (x == 3), stride));
                put8(dst + i, AVG16(vv, hv), w - i, avg);
            } else {
                __m128i vv = RND(vfilter8_u(p + (x == 3), stride));
                put8(dst + i, AVG16(hh, vv), w - i, avg);
            }
        }
        dst += stride;
        src += stride;
    }
}

void h264_qpel_mc_sse2(uint8_t *dst, const uint8_t *src, int stride,
                       int w, int h, int x, int y, int avg)
{
    if (!y)
        qpel_h_sse2(dst, src, stride, w, h, x, avg);
    else if (!x)
        qpel_v_sse2(dst, src, stride, w, h, y, avg);
    else
        qpel_hv_sse2(dst, src, stride, w, h, x, y, avg);
}

void h264_chroma_mc_sse2(uint8_t *dst, const uint8_t *src, int stride,
                         int w, int h, int x, int y, int avg)
{
    const __m128i A = _mm_set1_epi16((8 - x) * (8 - y));
    const __m128i B = _mm_set1_epi16(x * (8 - y));
    const __m128i C = _mm_set1_epi16((8 - x) * y);
    const __m128i D = _mm_set1_epi16(x * y);
    const __m128i rnd = _mm_set1_epi16(32);
    int j, i;

    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i += 8) {
            const uint8_t *p = src + i;
            const uint8_t *q = p + stride;
            __m128i v = _mm_add_epi16(
                _mm_add_epi16(_mm_mullo_epi16(U8(p), A),
                              _mm_mullo_epi16(U8(p + 1), B)),
                _mm_add_epi16(_mm_mullo_epi16(U8(q), C),
                              _mm_mullo_epi16(U8(q + 1), D)));
            v = _mm_srai_epi16(_mm_add_epi16(v, rnd), 6);
            put8(dst + i, v, w - i, avg);
        }
        dst += stride;
        src += stride;
    }
}

/* ------------------------------------------------------------------ */
/* full-pel copy and average                                           */
/* ------------------------------------------------------------------ */
/* The width histogram of a 720p High stream says 1.36M of its 1.43M 16-wide
 * luma MC calls are at integer position, so this is the most often run kernel
 * in the decoder. pavgusb is exactly (a + b + 1) >> 1, which is the rounding
 * the scalar rnd_avg uses, and a copy needs no rounding at all. */
static inline void hpel_row(uint8_t *dst, const uint8_t *src, int w, int avg)
{
    __m128i v;

    if (w >= 16) {
        v = _mm_loadu_si128((const __m128i *)src);
        if (avg)
            v = _mm_avg_epu8(_mm_loadu_si128((const __m128i *)dst), v);
        _mm_storeu_si128((__m128i *)dst, v);
        return;
    }
    /* narrower rows load and store 8 bytes: the planes carry a 16-pixel
     * margin, so the extra bytes are there, and only w of them are written */
    v = _mm_loadl_epi64((const __m128i *)src);
    if (avg)
        v = _mm_avg_epu8(_mm_loadl_epi64((const __m128i *)dst), v);
    if (w >= 8) {
        _mm_storel_epi64((__m128i *)dst, v);
    } else {
        uint32_t t = (uint32_t)_mm_cvtsi128_si32(v);
        memcpy(dst, &t, 4);
    }
}

void h264_hpel_mc_sse2(uint8_t *dst, const uint8_t *src, int stride,
                       int w, int h, int avg)
{
    int j;

    for (j = 0; j < h; j++) {
        hpel_row(dst, src, w, avg);
        dst += stride;
        src += stride;
    }
}

/* ------------------------------------------------------------------ */
/* weighted prediction                                                 */
/* ------------------------------------------------------------------ */
/* One int16 lane per pixel. The dispatcher only routes a call here when the
 * products cannot overflow, and paddsw saturation of the sum is invisible
 * after the shift, because 32767 >> d is still >= 255 for every d the
 * specification allows: an out-of-range value can only clip to 0 or 255 either
 * way. psraw is the scalar arithmetic >> and packusw the clip_u8.
 *
 * A sub-block is not necessarily 8-byte aligned - chroma inherits x_off >> 1 -
 * so only the 16-pixel case uses a vector store. Rows are margined by 16
 * pixels, so a 16-byte load is always in bounds and the narrower results go
 * back through a scalar store. */
static inline __m128i wp_scale(__m128i x, __m128i wv, __m128i ov, int shift)
{
    return _mm_srai_epi16(_mm_adds_epi16(_mm_mullo_epi16(x, wv), ov), shift);
}

static inline __m128i wp_bipt(__m128i a, __m128i b, __m128i wa, __m128i wb,
                              __m128i ov, int shift)
{
    __m128i s = _mm_adds_epi16(_mm_mullo_epi16(a, wa), _mm_mullo_epi16(b, wb));
    return _mm_srai_epi16(_mm_adds_epi16(s, ov), shift);
}

/* the last up-to-8 pixels of a row: t holds them in byte order */
static inline void wp_store(uint8_t *dst, __m128i v, int n)
{
    uint64_t t = (uint64_t)_mm_cvtsi128_si64(v);
    int i;

    for (i = 0; i < n; i++)
        dst[i] = (uint8_t)(t >> (8 * i));
}

static inline uint64_t wp_bytes8(__m128i v)
{
    return (uint64_t)_mm_cvtsi128_si64(v);
}

void h264_weight_pixels_sse2(uint8_t *dst, int stride, int w, int h,
                             int log2_denom, int weight, int offset)
{
    const __m128i zero = _mm_setzero_si128();
    __m128i wv, ov;
    int i, j;

    offset = (unsigned)offset << log2_denom;
    if (log2_denom)
        offset += 1 << (log2_denom - 1);
    wv = _mm_set1_epi16((short)weight);
    ov = _mm_set1_epi16((short)offset);

    for (j = 0; j < h; j++) {
        for (i = 0; i + 16 <= w; i += 16) {
            __m128i p = _mm_loadu_si128((const __m128i *)(dst + i));
            __m128i lo = wp_scale(_mm_unpacklo_epi8(p, zero), wv, ov,
                                  log2_denom);
            __m128i hi = wp_scale(_mm_unpackhi_epi8(p, zero), wv, ov,
                                  log2_denom);
            _mm_storeu_si128((__m128i *)(dst + i), _mm_packus_epi16(lo, hi));
        }
        for (; i + 8 <= w; i += 8) {
            __m128i p = _mm_loadu_si128((const __m128i *)(dst + i));
            __m128i v = wp_scale(_mm_unpacklo_epi8(p, zero), wv, ov,
                                 log2_denom);
            uint64_t t = wp_bytes8(_mm_packus_epi16(v, zero));
            memcpy(dst + i, &t, 8);
        }
        if (i < w) {
            __m128i p = _mm_loadu_si128((const __m128i *)(dst + i));
            __m128i v = wp_scale(_mm_unpacklo_epi8(p, zero), wv, ov,
                                 log2_denom);
            wp_store(dst + i, _mm_packus_epi16(v, zero), w - i);
        }
        dst += stride;
    }
}

void h264_biweight_pixels_sse2(uint8_t *dst, const uint8_t *src, int stride,
                               int w, int h, int log2_denom, int weightd,
                               int weights, int offset)
{
    const __m128i zero = _mm_setzero_si128();
    __m128i wd, ws, ov;
    int i, j, shift = log2_denom + 1;

    offset = (unsigned)((offset + 1) | 1) << log2_denom;
    wd = _mm_set1_epi16((short)weightd);
    ws = _mm_set1_epi16((short)weights);
    ov = _mm_set1_epi16((short)offset);

    for (j = 0; j < h; j++) {
        for (i = 0; i + 16 <= w; i += 16) {
            __m128i a = _mm_loadu_si128((const __m128i *)(dst + i));
            __m128i b = _mm_loadu_si128((const __m128i *)(src + i));
            __m128i lo = wp_bipt(_mm_unpacklo_epi8(a, zero),
                                 _mm_unpacklo_epi8(b, zero), wd, ws, ov, shift);
            __m128i hi = wp_bipt(_mm_unpackhi_epi8(a, zero),
                                 _mm_unpackhi_epi8(b, zero), wd, ws, ov, shift);
            _mm_storeu_si128((__m128i *)(dst + i), _mm_packus_epi16(lo, hi));
        }
        for (; i + 8 <= w; i += 8) {
            __m128i a = _mm_loadu_si128((const __m128i *)(dst + i));
            __m128i b = _mm_loadu_si128((const __m128i *)(src + i));
            __m128i v = wp_bipt(_mm_unpacklo_epi8(a, zero),
                                _mm_unpacklo_epi8(b, zero), wd, ws, ov, shift);
            uint64_t t = wp_bytes8(_mm_packus_epi16(v, zero));
            memcpy(dst + i, &t, 8);
        }
        if (i < w) {
            __m128i a = _mm_loadu_si128((const __m128i *)(dst + i));
            __m128i b = _mm_loadu_si128((const __m128i *)(src + i));
            __m128i v = wp_bipt(_mm_unpacklo_epi8(a, zero),
                                _mm_unpacklo_epi8(b, zero), wd, ws, ov, shift);
            wp_store(dst + i, _mm_packus_epi16(v, zero), w - i);
        }
        dst += stride;
        src += stride;
    }
}

void h264_avg_pixels_sse2(uint8_t *dst, const uint8_t *src, int stride,
                          int w, int h)
{
    int j;

    /* the identity weighting is the plain bi-prediction average, and pavgusb
     * is its (a + b + 1) >> 1 */
    for (j = 0; j < h; j++) {
        hpel_row(dst, src, w, 1);
        dst += stride;
        src += stride;
    }
}

/* ------------------------------------------------------------------ */
/* inverse transforms                                                  */
/* ------------------------------------------------------------------ */
/* Translating FFmpeg's IDCT4_ADD. The first stage stays in int16 lanes, which
 * is exactly what the scalar reference does: it truncates every stage result to
 * int16, so the wrapping of paddw/psubw matches, and its >>1 of an int16 is
 * psraw. After the transpose each register holds one column of that stage, so
 * the second stage is widened to int32 - the scalar code computes its sums in
 * 32 bits, and widening keeps the two paths identical for every input rather
 * than only for the range the specification allows.
 *
 * Following the second stage, register j holds row j of the result (one lane
 * per column), which is why no second transpose is needed before the store. */
#define IDCT4_1D(r0, r1, r2, r3, a0, a1, a2, a3, ADD, SUB, SRA, wd)  \
    do {                                                              \
        a0 = ADD(r0, r2);                                             \
        a1 = SUB(r0, r2);                                             \
        a2 = SUB(SRA(r1, 1), r3);                                     \
        a3 = ADD(r1, SRA(r3, 1));                                     \
        r0 = ADD(a0, a3);                                             \
        r1 = ADD(a1, a2);                                             \
        r2 = SUB(a1, a2);                                             \
        r3 = SUB(a0, a3);                                             \
    } while (0)

/* Four registers holding one row each (the four int16 in the low half) become
 * four registers holding one column each. */
#define TRANSPOSE4W(r0, r1, r2, r3, t0, t1, s0, s2)         \
    do {                                                     \
        t0 = _mm_unpacklo_epi16(r0, r1);                     \
        t1 = _mm_unpacklo_epi16(r2, r3);                     \
        s0 = _mm_unpacklo_epi32(t0, t1);                     \
        s2 = _mm_unpackhi_epi32(t0, t1);                     \
        r0 = s0;                                             \
        r2 = s2;                                             \
        r1 = _mm_srli_si128(s0, 8);                           \
        r3 = _mm_srli_si128(s2, 8);                           \
    } while (0)

/* int16 in the low four lanes of v to int32 in the low four lanes */
#define WIDEN4(v, t) ((t = _mm_srai_epi16(v, 15)), _mm_unpacklo_epi16(v, t))

#define ADD16 _mm_add_epi16
#define SUB16 _mm_sub_epi16
#define SRA16 _mm_srai_epi16
#define ADD32 _mm_add_epi32
#define SUB32 _mm_sub_epi32
#define SRA32 _mm_srai_epi32

/* dst += (delta >> 6) with the Clip1 saturation of the scalar version */
static inline void idct_store_row(uint8_t *dst, __m128i delta)
{
    uint64_t packed;
    __m128i cur = _mm_loadl_epi64((const __m128i *)dst);
    cur = _mm_unpacklo_epi8(cur, ZERO);            /* four unsigned words */
    cur = _mm_unpacklo_epi16(cur, ZERO);           /* four unsigned dwords */
    cur = _mm_add_epi32(cur, _mm_srai_epi32(delta, 6));
    /* two narrowing saturations: int32 -> int16 -> unsigned 0..255 */
    cur = _mm_packs_epi32(cur, ZERO);
    _mm_storel_epi64((__m128i *)&packed, _mm_packus_epi16(cur, ZERO));
    memcpy(dst, &packed, 4);
}

void h264_idct_add_sse2(uint8_t *dst, int16_t *block, int stride)
{
    __m128i r0 = _mm_loadl_epi64((const __m128i *)(block +  0));
    __m128i r1 = _mm_loadl_epi64((const __m128i *)(block +  4));
    __m128i r2 = _mm_loadl_epi64((const __m128i *)(block +  8));
    __m128i r3 = _mm_loadl_epi64((const __m128i *)(block + 12));
    __m128i t0, t1, s0, s2, a0, a1, a2, a3;

    /* block[0] += 1 << 5, in the same int16 arithmetic as the scalar code */
    r0 = _mm_add_epi16(r0, _mm_set_epi32(0, 0, 0, 32));
    IDCT4_1D(r0, r1, r2, r3, a0, a1, a2, a3, ADD16, SUB16, SRA16, w);
    TRANSPOSE4W(r0, r1, r2, r3, t0, t1, s0, s2);

    r0 = WIDEN4(r0, t0);
    r1 = WIDEN4(r1, t1);
    r2 = WIDEN4(r2, s0);
    r3 = WIDEN4(r3, s2);
    IDCT4_1D(r0, r1, r2, r3, a0, a1, a2, a3, ADD32, SUB32, SRA32, d);

    idct_store_row(dst,             r0);
    idct_store_row(dst + stride,     r1);
    idct_store_row(dst + 2 * stride, r2);
    idct_store_row(dst + 3 * stride, r3);
    memset(block, 0, 16 * sizeof(int16_t));
}

static inline void dc_add_row(uint8_t *dst, __m128i delta, int n)
{
    uint64_t packed;
    __m128i cur = _mm_loadl_epi64((const __m128i *)dst);
    cur = _mm_unpacklo_epi8(cur, ZERO);
    cur = _mm_add_epi16(cur, delta);
    _mm_storel_epi64((__m128i *)&packed, _mm_packus_epi16(cur, cur));
    memcpy(dst, &packed, (size_t)n);
}

void h264_idct_dc_add_sse2(uint8_t *dst, int16_t *block, int stride)
{
    int i;
    __m128i delta;

    /* (block[0] + 32) >> 6 in 32 bits, as the scalar reference computes it */
    delta = _mm_set1_epi16((int16_t)((block[0] + 32) >> 6));
    block[0] = 0;
    for (i = 0; i < 4; i++)
        dc_add_row(dst + i * stride, delta, 4);
}

void h264_idct8_dc_add_sse2(uint8_t *dst, int16_t *block, int stride)
{
    int i;
    __m128i delta;

    delta = _mm_set1_epi16((int16_t)((block[0] + 32) >> 6));
    block[0] = 0;
    for (i = 0; i < 8; i++)
        dc_add_row(dst + i * stride, delta, 8);
}

/* ------------------------------------------------------------------ */
/* in-loop deblocking                                                  */
/* ------------------------------------------------------------------ */
/* One int16 lane per edge position. The eight taps of a position (offsets
 * -4..+3 across the edge) are separate registers, because every term the
 * filter forms mixes taps of the same position. How the taps get into
 * registers depends on which axis is contiguous in memory:
 *   ystride == 1 - positions are adjacent bytes and taps are a row apart,
 *                so each tap is one strided load;
 *   xstride == 1 - taps are adjacent bytes and positions are rows apart,
 *                so an 8x8 patch is transposed, filtered, transposed back.
 * Everything is computed in int16: the widest intermediate here is
 * 4*(q0-p0) + (p1-q1) + 4, under 1300 for byte samples. */

/* r[i] holds 8 samples of row i; afterwards r[k] holds tap k of 8 rows */
static inline void lf_transpose8(__m128i r[8])
{
    __m128i a0 = _mm_unpacklo_epi16(r[0], r[1]);
    __m128i a1 = _mm_unpackhi_epi16(r[0], r[1]);
    __m128i a2 = _mm_unpacklo_epi16(r[2], r[3]);
    __m128i a3 = _mm_unpackhi_epi16(r[2], r[3]);
    __m128i a4 = _mm_unpacklo_epi16(r[4], r[5]);
    __m128i a5 = _mm_unpackhi_epi16(r[4], r[5]);
    __m128i a6 = _mm_unpacklo_epi16(r[6], r[7]);
    __m128i a7 = _mm_unpackhi_epi16(r[6], r[7]);
    __m128i b0 = _mm_unpacklo_epi32(a0, a2);
    __m128i b1 = _mm_unpackhi_epi32(a0, a2);
    __m128i b2 = _mm_unpacklo_epi32(a1, a3);
    __m128i b3 = _mm_unpackhi_epi32(a1, a3);
    __m128i b4 = _mm_unpacklo_epi32(a4, a6);
    __m128i b5 = _mm_unpackhi_epi32(a4, a6);
    __m128i b6 = _mm_unpacklo_epi32(a5, a7);
    __m128i b7 = _mm_unpackhi_epi32(a5, a7);

    r[0] = _mm_unpacklo_epi64(b0, b4);
    r[1] = _mm_unpackhi_epi64(b0, b4);
    r[2] = _mm_unpacklo_epi64(b1, b5);
    r[3] = _mm_unpackhi_epi64(b1, b5);
    r[4] = _mm_unpacklo_epi64(b2, b6);
    r[5] = _mm_unpackhi_epi64(b2, b6);
    r[6] = _mm_unpacklo_epi64(b3, b7);
    r[7] = _mm_unpackhi_epi64(b3, b7);
}

/* the four pixels the inter luma filter may rewrite */
typedef struct LF4 {
    __m128i p1, p0, q0, q1;
} LF4;

static inline __m128i lf_abs16(__m128i v)
{
    return _mm_max_epi16(v, _mm_sub_epi16(ZERO, v));
}

static inline __m128i lf_lt16(__m128i a, __m128i b)
{
    return _mm_cmplt_epi16(a, b);
}

/* clamp to [-tc, tc], as H264_CLIP does */
static inline __m128i lf_clip_tc(__m128i v, __m128i tc)
{
    return _mm_min_epi16(_mm_max_epi16(v, _mm_sub_epi16(ZERO, tc)), tc);
}

static inline __m128i lf_blend(__m128i mask, __m128i yes, __m128i no)
{
    return _mm_or_si128(_mm_and_si128(mask, yes), _mm_andnot_si128(mask, no));
}

/* i_delta of the reference: ((q0-p0)*4 + (p1-q1) + 4) >> 3, arithmetic shift */
static inline __m128i lf_delta(__m128i p0, __m128i p1, __m128i q0, __m128i q1,
                               __m128i tc)
{
    __m128i v = _mm_add_epi16(_mm_slli_epi16(_mm_sub_epi16(q0, p0), 2),
                              _mm_sub_epi16(p1, q1));
    v = _mm_add_epi16(v, _mm_set1_epi16(4));
    return lf_clip_tc(_mm_srai_epi16(v, 3), tc);
}

static inline LF4 lf_luma_inter(LF4 e, __m128i p2, __m128i q2,
                                __m128i tc, __m128i on,
                                __m128i alpha, __m128i beta)
{
    __m128i cond = _mm_and_si128(on, lf_lt16(lf_abs16(_mm_sub_epi16(e.p0, e.q0)), alpha));
    cond = _mm_and_si128(cond, lf_lt16(lf_abs16(_mm_sub_epi16(e.p1, e.p0)), beta));
    cond = _mm_and_si128(cond, lf_lt16(lf_abs16(_mm_sub_epi16(e.q1, e.q0)), beta));

    {
        const __m128i cp2 = lf_lt16(lf_abs16(_mm_sub_epi16(p2, e.p0)), beta);
        const __m128i cq2 = lf_lt16(lf_abs16(_mm_sub_epi16(q2, e.q0)), beta);
        /* tc grows by one for each side that also passes the beta test; the
         * reference still raises it when tc is zero, hence the sum and not
         * the p1/q1 write is what the zero case depends on */
        const __m128i tcx = _mm_add_epi16(
            tc, _mm_add_epi16(_mm_and_si128(cp2, ONE), _mm_and_si128(cq2, ONE)));
        const __m128i mid = _mm_srli_epi16(
            _mm_add_epi16(_mm_add_epi16(e.p0, e.q0), ONE), 1);
        /* a clip of zero is a no-op, so the reference's "if (tc_orig)" guard
         * needs no mask of its own */
        const __m128i np1 = _mm_and_si128(_mm_add_epi16(
            e.p1, lf_clip_tc(_mm_sub_epi16(_mm_srli_epi16(_mm_add_epi16(p2, mid), 1),
                                           e.p1), tc)), MASK_FF);
        const __m128i nq1 = _mm_and_si128(_mm_add_epi16(
            e.q1, lf_clip_tc(_mm_sub_epi16(_mm_srli_epi16(_mm_add_epi16(q2, mid), 1),
                                           e.q1), tc)), MASK_FF);
        const __m128i delta = lf_delta(e.p0, e.p1, e.q0, e.q1, tcx);

        /* p1/q1 wrap like the reference's (uint8_t) cast; p0/q0 saturate like
         * its H264_CLIP_PIXEL, so they stay raw and packus does the clamp */
        e.p1 = lf_blend(_mm_and_si128(cond, cp2), np1, e.p1);
        e.q1 = lf_blend(_mm_and_si128(cond, cq2), nq1, e.q1);
        e.p0 = lf_blend(cond, _mm_add_epi16(e.p0, delta), e.p0);
        e.q0 = lf_blend(cond, _mm_sub_epi16(e.q0, delta), e.q0);
    }
    return e;
}

static inline LF4 lf_chroma_inter(LF4 e, __m128i tc, __m128i on,
                                  __m128i alpha, __m128i beta)
{
    __m128i cond = _mm_and_si128(on, lf_lt16(lf_abs16(_mm_sub_epi16(e.p0, e.q0)), alpha));
    cond = _mm_and_si128(cond, lf_lt16(lf_abs16(_mm_sub_epi16(e.p1, e.p0)), beta));
    cond = _mm_and_si128(cond, lf_lt16(lf_abs16(_mm_sub_epi16(e.q1, e.q0)), beta));

    {
        const __m128i delta = lf_delta(e.p0, e.p1, e.q0, e.q1, tc);
        e.p0 = lf_blend(cond, _mm_add_epi16(e.p0, delta), e.p0);
        e.q0 = lf_blend(cond, _mm_sub_epi16(e.q0, delta), e.q0);
    }
    return e;
}

/* per-lane clipping range and enable for the eight positions of one pass;
 * position p belongs to group p / inner_iters, which is where tc0 comes from */
static inline int lf_group_masks(const int8_t *tc0, int inner_iters, int pos0, int n,
                                 int skip_when_zero, __m128i *tc, __m128i *on)
{
    int16_t vtc[8], von[8];
    int i, any = 0;

    for (i = 0; i < 8; i++) {
        const int g = i < n ? (pos0 + i) / inner_iters : -1;
        const int t = g >= 0 ? tc0[g] : -1;
        const int usable = t >= 0 && (!skip_when_zero || t > 0);
        vtc[i] = usable ? (int16_t)t : 0;
        von[i] = usable ? (int16_t)0xFFFF : 0;
        any |= usable;
    }
    *tc = _mm_loadu_si128((const __m128i *)vtc);
    *on = _mm_loadu_si128((const __m128i *)von);
    return any;
}

static inline void lf_store_n(uint8_t *p, __m128i v, int n)
{
    if (n >= 8) {
        _mm_storel_epi64((__m128i *)p, v);
    } else {
        uint32_t w = (uint32_t)_mm_cvtsi128_si32(v);
        memcpy(p, &w, 4);
    }
}

/* luma inter edges are 16 positions wide (inner_iters == 4), so the two
 * groups of four lanes are two broadcasts and one 64-bit unpack; a group
 * whose tc0 is -1 (bS 0) still has to be disabled, exactly as the generic
 * helper does. This avoids its eight-entry stack array and load on the most
 * common filter. */
static inline int lf_luma_masks(const int8_t *tc0, int inner_iters, int pos0,
                                int n, __m128i *tc, __m128i *on)
{
    if (inner_iters == 4) {
        const int g = pos0 >> 2;
        const int t0 = tc0[g], t1 = tc0[g + 1];
        const int u0 = t0 >= 0, u1 = t1 >= 0;
        *tc = _mm_unpacklo_epi64(_mm_set1_epi16((short)(u0 ? t0 : 0)),
                                 _mm_set1_epi16((short)(u1 ? t1 : 0)));
        *on = _mm_unpacklo_epi64(_mm_set1_epi16((short)(u0 ? -1 : 0)),
                                 _mm_set1_epi16((short)(u1 ? -1 : 0)));
        return u0 | u1;
    }
    return lf_group_masks(tc0, inner_iters, pos0, n, 0, tc, on);
}

/* the eight taps of a pass in the xstride == 1 layout: 8 consecutive
 * positions, eight bytes each, transposed into one register per tap */
static inline void lf_load_rows(uint8_t *pix, int ystride, int pos0, int n, __m128i t[8])
{
    uint8_t *base = pix + pos0 * ystride;
    int i;

    for (i = 0; i < 8; i++)
        t[i] = U8(base + (i < n ? i : n - 1) * ystride - 4);
    lf_transpose8(t);
}

static inline void lf_store_rows(uint8_t *pix, int ystride, int pos0, int n, __m128i t[8])
{
    uint8_t *base = pix + pos0 * ystride;
    int i;

    lf_transpose8(t);
    for (i = 0; i < n; i++)
        lf_store_n(base + i * ystride - 4, _mm_packus_epi16(t[i], t[i]), 8);
}

void h264_loop_filter_luma_sse2(uint8_t *pix, int xstride, int ystride, int inner_iters,
                                int alpha, int beta, const int8_t *tc0)
{
    const __m128i av = _mm_set1_epi16((short)alpha);
    const __m128i bv = _mm_set1_epi16((short)beta);
    const int total = 4 * inner_iters;
    int pos0;

    if (tc0[0] < 0 && tc0[1] < 0 && tc0[2] < 0 && tc0[3] < 0)
        return;

    for (pos0 = 0; pos0 < total; pos0 += 8) {
        const int n = total - pos0 < 8 ? total - pos0 : 8;
        __m128i tc, on, t[8];
        LF4 e;

        if (!lf_luma_masks(tc0, inner_iters, pos0, n, &tc, &on))
            continue;

        if (ystride == 1) {
            const uint8_t *b = pix + pos0;
            e.p1 = U8(b - 2 * xstride);
            e.p0 = U8(b - xstride);
            e.q0 = U8(b);
            e.q1 = U8(b + xstride);
            e = lf_luma_inter(e, U8(b - 3 * xstride), U8(b + 2 * xstride),
                              tc, on, av, bv);
            lf_store_n((uint8_t *)(b - 2 * xstride), _mm_packus_epi16(e.p1, e.p1), n);
            lf_store_n((uint8_t *)(b - xstride),     _mm_packus_epi16(e.p0, e.p0), n);
            lf_store_n((uint8_t *)b,                 _mm_packus_epi16(e.q0, e.q0), n);
            lf_store_n((uint8_t *)(b + xstride),     _mm_packus_epi16(e.q1, e.q1), n);
        } else {
            lf_load_rows(pix, ystride, pos0, n, t);
            e.p1 = t[2]; e.p0 = t[3]; e.q0 = t[4]; e.q1 = t[5];
            e = lf_luma_inter(e, t[1], t[6], tc, on, av, bv);
            /* the taps the filter does not rewrite go back unchanged */
            t[2] = e.p1; t[3] = e.p0; t[4] = e.q0; t[5] = e.q1;
            lf_store_rows(pix, ystride, pos0, n, t);
        }
    }
}

/* The horizontal chroma edge has eight positions whose taps are four adjacent
 * bytes each: a full 8x8 transpose costs more than the filter it feeds, which
 * is why this direction used to stay scalar. Only four taps are needed and only
 * two are written, so a 4x8 byte transpose built from byte and word unpacks is
 * enough. It also lets the eight positions be filtered at once. */
static inline int lf_chroma_masks8(const int8_t *tc0, __m128i *tc, __m128i *on)
{
    const int t0 = tc0[0], t1 = tc0[1], t2 = tc0[2], t3 = tc0[3];
    const int u0 = t0 > 0, u1 = t1 > 0, u2 = t2 > 0, u3 = t3 > 0;
    const __m128i v01 = _mm_unpacklo_epi32(
        _mm_set1_epi16((short)(u0 ? t0 : 0)),
        _mm_set1_epi16((short)(u1 ? t1 : 0)));
    const __m128i v23 = _mm_unpacklo_epi32(
        _mm_set1_epi16((short)(u2 ? t2 : 0)),
        _mm_set1_epi16((short)(u3 ? t3 : 0)));
    const __m128i m01 = _mm_unpacklo_epi32(
        _mm_set1_epi16((short)(u0 ? -1 : 0)),
        _mm_set1_epi16((short)(u1 ? -1 : 0)));
    const __m128i m23 = _mm_unpacklo_epi32(
        _mm_set1_epi16((short)(u2 ? -1 : 0)),
        _mm_set1_epi16((short)(u3 ? -1 : 0)));

    *tc = _mm_unpacklo_epi64(v01, v23);
    *on = _mm_unpacklo_epi64(m01, m23);
    return u0 | u1 | u2 | u3;
}

static void lf_chroma_h8(uint8_t *pix, int ystride, int alpha, int beta,
                         const int8_t *tc0)
{
    const __m128i zero = _mm_setzero_si128();
    __m128i u0, u1, u2, u3, a0, a1, b0, b1, c01, c23, tc, on;
    LF4 e;
    int i;

    if (!lf_chroma_masks8(tc0, &tc, &on))
        return;

    /* four taps per position; rows are paired so that one 64-bit half holds
     * the taps of two positions */
    u0 = _mm_unpacklo_epi32(
        _mm_cvtsi32_si128((int)h264_rn32(pix + 0 * ystride - 2)),
        _mm_cvtsi32_si128((int)h264_rn32(pix + 1 * ystride - 2)));
    u1 = _mm_unpacklo_epi32(
        _mm_cvtsi32_si128((int)h264_rn32(pix + 2 * ystride - 2)),
        _mm_cvtsi32_si128((int)h264_rn32(pix + 3 * ystride - 2)));
    u2 = _mm_unpacklo_epi32(
        _mm_cvtsi32_si128((int)h264_rn32(pix + 4 * ystride - 2)),
        _mm_cvtsi32_si128((int)h264_rn32(pix + 5 * ystride - 2)));
    u3 = _mm_unpacklo_epi32(
        _mm_cvtsi32_si128((int)h264_rn32(pix + 6 * ystride - 2)),
        _mm_cvtsi32_si128((int)h264_rn32(pix + 7 * ystride - 2)));

    a0 = _mm_unpacklo_epi8(u0, u1);
    a1 = _mm_unpacklo_epi8(u2, u3);
    b0 = _mm_unpacklo_epi16(a0, a1);   /* taps 0..3 of the even positions */
    b1 = _mm_unpackhi_epi16(a0, a1);   /* taps 0..3 of the odd positions */
    c01 = _mm_unpacklo_epi8(b0, b1);   /* tap 0 then tap 1, all positions */
    c23 = _mm_unpackhi_epi8(b0, b1);   /* tap 2 then tap 3, all positions */

    e.p1 = _mm_unpacklo_epi8(c01, zero);
    e.p0 = _mm_unpackhi_epi8(c01, zero);
    e.q0 = _mm_unpacklo_epi8(c23, zero);
    e.q1 = _mm_unpackhi_epi8(c23, zero);

    e = lf_chroma_inter(e, tc, on, _mm_set1_epi16((short)alpha),
                        _mm_set1_epi16((short)beta));

    /* p0 and q0 are adjacent in memory, so the eight rewritten pairs go back
     * as eight 16-bit stores rather than another transpose */
    {
        const __m128i p0b = _mm_packus_epi16(e.p0, e.p0);
        const __m128i q0b = _mm_packus_epi16(e.q0, e.q0);
        const __m128i pairs = _mm_unpacklo_epi8(p0b, q0b);
        uint32_t d[4];

        d[0] = (uint32_t)_mm_cvtsi128_si32(pairs);
        d[1] = (uint32_t)_mm_cvtsi128_si32(_mm_srli_si128(pairs, 4));
        d[2] = (uint32_t)_mm_cvtsi128_si32(_mm_srli_si128(pairs, 8));
        d[3] = (uint32_t)_mm_cvtsi128_si32(_mm_srli_si128(pairs, 12));
        for (i = 0; i < 4; i++) {
            const uint16_t lo = (uint16_t)d[i];
            const uint16_t hi = (uint16_t)(d[i] >> 16);
            memcpy(pix + (2 * i) * ystride - 1, &lo, 2);
            memcpy(pix + (2 * i + 1) * ystride - 1, &hi, 2);
        }
    }
}

void h264_loop_filter_chroma_sse2(uint8_t *pix, int xstride, int ystride, int inner_iters,
                                  int alpha, int beta, const int8_t *tc0)
{
    const __m128i av = _mm_set1_epi16((short)alpha);
    const __m128i bv = _mm_set1_epi16((short)beta);
    const int total = 4 * inner_iters;
    int pos0;

    if (tc0[0] <= 0 && tc0[1] <= 0 && tc0[2] <= 0 && tc0[3] <= 0)
        return;

    if (xstride == 1 && inner_iters == 2) {
        lf_chroma_h8(pix, ystride, alpha, beta, tc0);
        return;
    }

    for (pos0 = 0; pos0 < total; pos0 += 8) {
        const int n = total - pos0 < 8 ? total - pos0 : 8;
        __m128i tc, on, t[8];
        LF4 e;

        /* every 4:2:0 edge is exactly eight positions, so the direct mask
         * builder covers the vertical direction as well; the generic helper
         * remains for the four-position MBAFF edges and the dsp sweep */
        if (inner_iters == 2) {
            if (!lf_chroma_masks8(tc0, &tc, &on))
                continue;
        } else if (!lf_group_masks(tc0, inner_iters, pos0, n, 1, &tc, &on)) {
            continue;
        }

        if (ystride == 1) {
            const uint8_t *b = pix + pos0;
            e.p1 = U8(b - 2 * xstride);
            e.p0 = U8(b - xstride);
            e.q0 = U8(b);
            e.q1 = U8(b + xstride);
            e = lf_chroma_inter(e, tc, on, av, bv);
            lf_store_n((uint8_t *)(b - xstride), _mm_packus_epi16(e.p0, e.p0), n);
            lf_store_n((uint8_t *)b,             _mm_packus_epi16(e.q0, e.q0), n);
        } else {
            lf_load_rows(pix, ystride, pos0, n, t);
            e.p1 = t[2]; e.p0 = t[3]; e.q0 = t[4]; e.q1 = t[5];
            e = lf_chroma_inter(e, tc, on, av, bv);
            t[3] = e.p0; t[4] = e.q0;
            lf_store_rows(pix, ystride, pos0, n, t);
        }
    }
}

/* the six pixels an intra edge can rewrite, plus the two it only reads */
typedef struct LF6 {
    __m128i p2, p1, p0, q0, q1, q2;
} LF6;

static inline __m128i lf_lanes_used(int n)
{
    if (n >= 8)
        return _mm_set1_epi16(-1);
    if (n >= 4)
        return _mm_set_epi64x(0, -1);
    {
        int16_t v[8];
        int i;

        for (i = 0; i < 8; i++)
            v[i] = i < n ? (int16_t)0xFFFF : 0;
        return _mm_loadu_si128((const __m128i *)v);
    }
}

static inline LF6 lf_luma_intra(LF6 e, __m128i p3, __m128i q3, __m128i on,
                                __m128i alpha, __m128i beta, __m128i alpha_q)
{
    const __m128i p2 = e.p2, p1 = e.p1, p0 = e.p0, q0 = e.q0, q1 = e.q1, q2 = e.q2;
    const __m128i two = _mm_set1_epi16(2), four = _mm_set1_epi16(4);
    const __m128i d0 = lf_abs16(_mm_sub_epi16(p0, q0));
    __m128i cond = _mm_and_si128(on, lf_lt16(d0, alpha));
    __m128i strong, mp2, mq2;

    cond = _mm_and_si128(cond, lf_lt16(lf_abs16(_mm_sub_epi16(p1, p0)), beta));
    cond = _mm_and_si128(cond, lf_lt16(lf_abs16(_mm_sub_epi16(q1, q0)), beta));
    strong = _mm_and_si128(cond, lf_lt16(d0, alpha_q));
    mp2 = _mm_and_si128(strong, lf_lt16(lf_abs16(_mm_sub_epi16(p2, p0)), beta));
    mq2 = _mm_and_si128(strong, lf_lt16(lf_abs16(_mm_sub_epi16(q2, q0)), beta));

    /* every candidate reads the samples the reference loaded first, even
     * though the reference writes the p side before it computes the q side */
    {
        const __m128i p0_w = _mm_srli_epi16(_mm_add_epi16(_mm_add_epi16(_mm_slli_epi16(p1, 1), p0),
                                                         _mm_add_epi16(q1, two)), 2);
        const __m128i q0_w = _mm_srli_epi16(_mm_add_epi16(_mm_add_epi16(_mm_slli_epi16(q1, 1), q0),
                                                         _mm_add_epi16(p1, two)), 2);
        /* sums of samples only: the widest is 8*255+4, so logical shifts */
        const __m128i p0_s = _mm_srli_epi16(_mm_add_epi16(_mm_add_epi16(_mm_add_epi16(p2, _mm_slli_epi16(p1, 1)),
                                                                       _mm_slli_epi16(p0, 1)),
                                                         _mm_add_epi16(_mm_slli_epi16(q0, 1),
                                                                       _mm_add_epi16(q1, four))), 3);
        const __m128i p1_s = _mm_srli_epi16(_mm_add_epi16(_mm_add_epi16(p2, p1),
                                                         _mm_add_epi16(p0, _mm_add_epi16(q0, two))), 2);
        const __m128i p2_s = _mm_srli_epi16(_mm_add_epi16(_mm_add_epi16(_mm_add_epi16(_mm_slli_epi16(p3, 1),
                                                                                     _mm_slli_epi16(p2, 1)),
                                                                       p2),
                                                         _mm_add_epi16(_mm_add_epi16(p1, p0),
                                                                       _mm_add_epi16(q0, four))), 3);
        const __m128i q0_s = _mm_srli_epi16(_mm_add_epi16(_mm_add_epi16(_mm_add_epi16(q2, _mm_slli_epi16(q1, 1)),
                                                                       _mm_slli_epi16(q0, 1)),
                                                         _mm_add_epi16(_mm_slli_epi16(p0, 1),
                                                                       _mm_add_epi16(p1, four))), 3);
        const __m128i q1_s = _mm_srli_epi16(_mm_add_epi16(_mm_add_epi16(q2, q1),
                                                         _mm_add_epi16(q0, _mm_add_epi16(p0, two))), 2);
        const __m128i q2_s = _mm_srli_epi16(_mm_add_epi16(_mm_add_epi16(_mm_add_epi16(_mm_slli_epi16(q3, 1),
                                                                                     _mm_slli_epi16(q2, 1)),
                                                                       q2),
                                                         _mm_add_epi16(_mm_add_epi16(q1, q0),
                                                                       _mm_add_epi16(p0, four))), 3);

        /* a side that fails its own beta test, and the whole weak case, fall
         * back to the 4-tap average on p0/q0 only */
        e.p0 = lf_blend(cond, lf_blend(mp2, p0_s, p0_w), p0);
        e.q0 = lf_blend(cond, lf_blend(mq2, q0_s, q0_w), q0);
        e.p1 = lf_blend(mp2, p1_s, p1);
        e.p2 = lf_blend(mp2, p2_s, p2);
        e.q1 = lf_blend(mq2, q1_s, q1);
        e.q2 = lf_blend(mq2, q2_s, q2);
    }
    return e;
}

static inline LF4 lf_chroma_intra(LF4 e, __m128i on, __m128i alpha, __m128i beta)
{
    __m128i cond = _mm_and_si128(on, lf_lt16(lf_abs16(_mm_sub_epi16(e.p0, e.q0)), alpha));
    const __m128i two = _mm_set1_epi16(2);

    cond = _mm_and_si128(cond, lf_lt16(lf_abs16(_mm_sub_epi16(e.p1, e.p0)), beta));
    cond = _mm_and_si128(cond, lf_lt16(lf_abs16(_mm_sub_epi16(e.q1, e.q0)), beta));
    e.p0 = lf_blend(cond, _mm_srli_epi16(_mm_add_epi16(_mm_add_epi16(_mm_slli_epi16(e.p1, 1), e.p0),
                                                       _mm_add_epi16(e.q1, two)), 2), e.p0);
    e.q0 = lf_blend(cond, _mm_srli_epi16(_mm_add_epi16(_mm_add_epi16(_mm_slli_epi16(e.q1, 1), e.q0),
                                                       _mm_add_epi16(e.p1, two)), 2), e.q0);
    return e;
}

/* the six taps an intra edge can rewrite, in the ystride == 1 layout */
#define LF_TAPS6(b, xs)                                                         \
    p2 = U8((b) - 3 * (xs)), p1 = U8((b) - 2 * (xs)), p0 = U8((b) - (xs)),       \
    q0 = U8(b),                 q1 = U8(b + (xs)),   q2 = U8(b + 2 * (xs))

void h264_loop_filter_luma_intra_sse2(uint8_t *pix, int xstride, int ystride,
                                      int inner_iters, int alpha, int beta)
{
    const __m128i av = _mm_set1_epi16((short)alpha);
    const __m128i bv = _mm_set1_epi16((short)beta);
    const __m128i aq = _mm_set1_epi16((short)((alpha >> 2) + 2));
    const int total = 4 * inner_iters;
    int pos0;

    for (pos0 = 0; pos0 < total; pos0 += 8) {
        const int n = total - pos0 < 8 ? total - pos0 : 8;
        const __m128i on = lf_lanes_used(n);
        __m128i t[8], p3, p2, p1, p0, q0, q1, q2, q3;
        LF6 e;

        if (ystride == 1) {
            const uint8_t *b = pix + pos0;
            p3 = U8(b - 4 * xstride);
            q3 = U8(b + 3 * xstride);
            LF_TAPS6(b, xstride);
            e.p2 = p2; e.p1 = p1; e.p0 = p0; e.q0 = q0; e.q1 = q1; e.q2 = q2;
            e = lf_luma_intra(e, p3, q3, on, av, bv, aq);
            lf_store_n((uint8_t *)(b - 3 * xstride), _mm_packus_epi16(e.p2, e.p2), n);
            lf_store_n((uint8_t *)(b - 2 * xstride), _mm_packus_epi16(e.p1, e.p1), n);
            lf_store_n((uint8_t *)(b - xstride),     _mm_packus_epi16(e.p0, e.p0), n);
            lf_store_n((uint8_t *)b,                 _mm_packus_epi16(e.q0, e.q0), n);
            lf_store_n((uint8_t *)(b + xstride),     _mm_packus_epi16(e.q1, e.q1), n);
            lf_store_n((uint8_t *)(b + 2 * xstride), _mm_packus_epi16(e.q2, e.q2), n);
        } else {
            lf_load_rows(pix, ystride, pos0, n, t);
            e.p2 = t[1]; e.p1 = t[2]; e.p0 = t[3];
            e.q0 = t[4]; e.q1 = t[5]; e.q2 = t[6];
            e = lf_luma_intra(e, t[0], t[7], on, av, bv, aq);
            t[1] = e.p2; t[2] = e.p1; t[3] = e.p0;
            t[4] = e.q0; t[5] = e.q1; t[6] = e.q2;
            lf_store_rows(pix, ystride, pos0, n, t);
        }
    }
}

void h264_loop_filter_chroma_intra_sse2(uint8_t *pix, int xstride, int ystride,
                                        int inner_iters, int alpha, int beta)
{
    const __m128i av = _mm_set1_epi16((short)alpha);
    const __m128i bv = _mm_set1_epi16((short)beta);
    const int total = 4 * inner_iters;
    int pos0;

    for (pos0 = 0; pos0 < total; pos0 += 8) {
        const int n = total - pos0 < 8 ? total - pos0 : 8;
        const __m128i on = lf_lanes_used(n);
        __m128i t[8];
        LF4 e;

        if (ystride == 1) {
            const uint8_t *b = pix + pos0;
            e.p1 = U8(b - 2 * xstride);
            e.p0 = U8(b - xstride);
            e.q0 = U8(b);
            e.q1 = U8(b + xstride);
            e = lf_chroma_intra(e, on, av, bv);
            lf_store_n((uint8_t *)(b - xstride), _mm_packus_epi16(e.p0, e.p0), n);
            lf_store_n((uint8_t *)b,             _mm_packus_epi16(e.q0, e.q0), n);
        } else {
            lf_load_rows(pix, ystride, pos0, n, t);
            e.p1 = t[2]; e.p0 = t[3]; e.q0 = t[4]; e.q1 = t[5];
            e = lf_chroma_intra(e, on, av, bv);
            t[3] = e.p0; t[4] = e.q0;
            lf_store_rows(pix, ystride, pos0, n, t);
        }
    }
}

#pragma GCC pop_options

#endif /* x86 */