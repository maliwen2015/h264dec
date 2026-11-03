/*
 * AVX2 luma quarter-pel motion compensation for 16-sample-wide blocks
 * (8-bit 4:2:0).
 *
 * The same arithmetic as h264_qpel_mc_sse2 with 16 samples per vector: the
 * 6-tap filters in int16 lanes, and the second pass of the combined filter in
 * int32 lanes, because six unrounded int16 intermediates weighted by 20 and -5
 * overflow 16 bits. Narrower blocks stay on the SSE2 kernel, where one vector
 * already spans the row.
 *
 * The read window does not grow: the horizontal pass needs source columns
 * -2..+18 and the shifted vertical pass -1..+19, against the 16-pixel plane
 * margin, and the tmp rows only carry the 16 columns the vertical pass reads.
 *
 * Compiled for AVX2 on its own and reached only when h264_have_avx2 says both
 * the CPU and the OS register state allow it; the SSE2 and scalar paths stay
 * available for everything else.
 */
#include "h264_internal.h"

#if defined(__i386__) || defined(__x86_64__)

#pragma GCC push_options
#pragma GCC target("avx2")

#include <immintrin.h>

#define Z     _mm256_setzero_si256()
#define V20   _mm256_set1_epi16(20)
#define V5    _mm256_set1_epi16(5)
#define V16   _mm256_set1_epi16(16)
#define V1    _mm256_set1_epi16(1)
#define V255  _mm256_set1_epi16(255)
/* 16 samples widened into int16 lanes */
#define U16(p) _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)(p)))
/* 8 int16 intermediates widened into int32 lanes */
#define W8(p, off) \
    _mm256_cvtepi16_epi32(_mm_loadu_si128((const __m128i *)((const int16_t *)(p) + (off))))
#define TWENTY32(x) _mm256_add_epi32(_mm256_slli_epi32(x, 4), _mm256_slli_epi32(x, 2))
#define FIVE32(x)   _mm256_add_epi32(_mm256_slli_epi32(x, 2), x)

static inline __m256i hfilter16_u(const uint8_t *p)
{
    __m256i acc = _mm256_mullo_epi16(_mm256_add_epi16(U16(p), U16(p + 1)), V20);
    acc = _mm256_sub_epi16(acc,
                           _mm256_mullo_epi16(_mm256_add_epi16(U16(p - 1),
                                                               U16(p + 2)), V5));
    return _mm256_add_epi16(acc, _mm256_add_epi16(U16(p - 2), U16(p + 3)));
}

static inline __m256i vfilter16_u(const uint8_t *p, int stride)
{
    __m256i acc = _mm256_mullo_epi16(_mm256_add_epi16(U16(p), U16(p + stride)), V20);
    acc = _mm256_sub_epi16(acc,
                           _mm256_mullo_epi16(_mm256_add_epi16(U16(p - stride),
                                                               U16(p + 2 * stride)), V5));
    return _mm256_add_epi16(acc, _mm256_add_epi16(U16(p - 2 * stride),
                                                 U16(p + 3 * stride)));
}

#define RND16(v)   _mm256_srai_epi16(_mm256_add_epi16(v, V16), 5)
#define CLIP16(v)  _mm256_min_epi16(_mm256_max_epi16(v, Z), V255)
/* the scalar path clips every half-pel result before averaging two of them, so
 * a single clamp at the end would not be bit-exact */
#define AVG16(a, b) \
    _mm256_srli_epi16(_mm256_add_epi16(_mm256_add_epi16(CLIP16(a), CLIP16(b)), V1), 1)

static inline void put16(uint8_t *dst, __m256i v, int avg)
{
    if (avg)
        v = _mm256_srli_epi16(_mm256_add_epi16(_mm256_add_epi16(CLIP16(v), U16(dst)), V1), 1);
    _mm_storeu_si128((__m128i *)dst,
                     _mm_packus_epi16(_mm256_castsi256_si128(v),
                                      _mm256_extracti128_si256(v, 1)));
}

/* vertical 6-tap of eight int16 intermediates, rounded, in int32 lanes */
static inline __m256i vsum32_8(const int16_t *p, int n)
{
    const __m256i k512 = _mm256_set1_epi32(512);
    __m256i a = W8(p, 0), b = W8(p + n, 0), c = W8(p - n, 0);
    __m256i d = W8(p + 2 * n, 0), e = W8(p - 2 * n, 0), f = W8(p + 3 * n, 0);
    __m256i acc = _mm256_add_epi32(TWENTY32(_mm256_add_epi32(a, b)), e);
    acc = _mm256_sub_epi32(acc, FIVE32(_mm256_add_epi32(c, d)));
    acc = _mm256_add_epi32(acc, f);
    return _mm256_srai_epi32(_mm256_add_epi32(acc, k512), 10);
}

/* eight int32 lanes back to eight int16, keeping column order */
static inline __m128i pack8(__m256i v)
{
    return _mm_packs_epi32(_mm256_castsi256_si128(v),
                           _mm256_extracti128_si256(v, 1));
}

static void qpel_h_avx2(uint8_t *dst, const uint8_t *src, int stride,
                        int h, int x, int avg)
{
    int j;

    for (j = 0; j < h; j++) {
        __m256i hp = RND16(hfilter16_u(src));
        __m256i v = (x == 2) ? hp : AVG16(U16(src + (x >> 1)), hp);
        put16(dst, v, avg);
        dst += stride;
        src += stride;
    }
}

static void qpel_v_avx2(uint8_t *dst, const uint8_t *src, int stride,
                        int h, int y, int avg)
{
    int j;

    for (j = 0; j < h; j++) {
        __m256i vp = RND16(vfilter16_u(src, stride));
        __m256i v = (y == 2) ? vp : AVG16(U16(src + (y == 3 ? stride : 0)), vp);
        put16(dst, v, avg);
        dst += stride;
        src += stride;
    }
}

/* tmp row width: the 16 output columns, which is all the vertical pass reads */
#define HW_W 16

static void qpel_hv_avx2(uint8_t *dst, const uint8_t *src, int stride,
                         int h, int x, int y, int avg)
{
    int16_t tmp[(16 + 5) * HW_W];
    int j;

    /* unrounded horizontal filter of the rows the vertical pass needs (output
     * row j uses rows j-2 .. j+3), with row -2 at tmp[0] */
    for (j = -2; j < h + 3; j++)
        _mm256_storeu_si256((__m256i *)(tmp + (j + 2) * HW_W),
                            hfilter16_u(src + j * stride));

    for (j = 0; j < h; j++) {
        const int16_t *t = tmp + (j + 2) * HW_W;
        __m256i hv = _mm256_setr_m128i(pack8(vsum32_8(t, HW_W)),
                                       pack8(vsum32_8(t + 8, HW_W)));
        __m256i r;

        if (x == 2 && y == 2) {
            r = hv;
        } else {
            __m256i hh = RND16(_mm256_loadu_si256((const __m256i *)
                                                  (t + (y >> 1) * HW_W)));
            if (x == 2) {
                r = AVG16(hh, hv);
            } else {
                __m256i vv = RND16(vfilter16_u(src + (x == 3), stride));
                r = (y == 2) ? AVG16(vv, hv) : AVG16(hh, vv);
            }
        }
        put16(dst, r, avg);
        dst += stride;
        src += stride;
    }
}

void h264_qpel_mc_avx2(uint8_t *dst, const uint8_t *src, int stride,
                       int w, int h, int x, int y, int avg)
{
    if (!y)
        qpel_h_avx2(dst, src, stride, h, x, avg);
    else if (!x)
        qpel_v_avx2(dst, src, stride, h, y, avg);
    else
        qpel_hv_avx2(dst, src, stride, h, x, y, avg);
}

/* ------------------------------------------------------------------ */
/* 8-bit luma deblocking, all sixteen positions in one vector          */
/* ------------------------------------------------------------------ */
/* In the ystride == 1 layout the sixteen positions of a luma edge are sixteen
 * consecutive bytes, so each tap is one 16-byte load widened to int16 lanes;
 * the SSE2 kernel runs the same edge in two passes of eight. The transposed
 * layout (xstride == 1) is left to SSE2, where the 8x8 byte transpose already
 * costs more than the arithmetic it feeds. */
#define VFF _mm256_set1_epi16(0xFF)

static inline __m256i lf_abs16_avx2(__m256i v)
{
    return _mm256_max_epi16(v, _mm256_sub_epi16(Z, v));
}

static inline __m256i lf_lt16_avx2(__m256i a, __m256i b)
{
    return _mm256_cmpgt_epi16(b, a);
}

static inline __m256i lf_clip_tc_avx2(__m256i v, __m256i tc)
{
    return _mm256_min_epi16(_mm256_max_epi16(v, _mm256_sub_epi16(Z, tc)), tc);
}

static inline __m256i lf_blend_avx2(__m256i mask, __m256i yes, __m256i no)
{
    return _mm256_or_si256(_mm256_and_si256(mask, yes),
                           _mm256_andnot_si256(mask, no));
}

/* four int16 lanes per tc0 group, as in the SSE2 kernel's lf_luma_masks: the
 * two 64-bit halves of a vector hold one group each, and a -1 group (bS 0)
 * still has to be disabled rather than merely clipping to zero */
static inline __m256i lf_luma_masks16(const int8_t *tc0, __m256i *on)
{
    const int t0 = tc0[0], t1 = tc0[1], t2 = tc0[2], t3 = tc0[3];
    const int u0 = t0 >= 0, u1 = t1 >= 0, u2 = t2 >= 0, u3 = t3 >= 0;
    const __m128i lo_tc = _mm_unpacklo_epi64(
        _mm_set1_epi16((short)(u0 ? t0 : 0)),
        _mm_set1_epi16((short)(u1 ? t1 : 0)));
    const __m128i hi_tc = _mm_unpacklo_epi64(
        _mm_set1_epi16((short)(u2 ? t2 : 0)),
        _mm_set1_epi16((short)(u3 ? t3 : 0)));
    const __m128i lo_on = _mm_unpacklo_epi64(
        _mm_set1_epi16((short)(u0 ? -1 : 0)),
        _mm_set1_epi16((short)(u1 ? -1 : 0)));
    const __m128i hi_on = _mm_unpacklo_epi64(
        _mm_set1_epi16((short)(u2 ? -1 : 0)),
        _mm_set1_epi16((short)(u3 ? -1 : 0)));

    *on = _mm256_inserti128_si256(_mm256_castsi128_si256(lo_on), hi_on, 1);
    return _mm256_inserti128_si256(_mm256_castsi128_si256(lo_tc), hi_tc, 1);
}

/* writing a vector of 16 int16 lanes back as 16 bytes: pack the two halves
 * against each other, not each half against itself, so lane order survives */
static inline void lf_store16(uint8_t *p, __m256i v)
{
    _mm_storeu_si128((__m128i *)p,
                     _mm_packus_epi16(_mm256_castsi256_si128(v),
                                      _mm256_extracti128_si256(v, 1)));
}

void h264_loop_filter_luma_avx2(uint8_t *pix, int xstride, int ystride,
                                int inner_iters, int alpha, int beta,
                                const int8_t *tc0)
{
    const __m256i av = _mm256_set1_epi16((short)alpha);
    const __m256i bv = _mm256_set1_epi16((short)beta);
    __m256i on, tc, p2, q2, p1, p0, q0, q1, cond, cp2, cq2, tcx, mid, delta;

    (void)ystride;
    if (inner_iters != 4)
        return;

    tc = lf_luma_masks16(tc0, &on);

    p2 = U16(pix - 3 * xstride);
    p1 = U16(pix - 2 * xstride);
    p0 = U16(pix - xstride);
    q0 = U16(pix);
    q1 = U16(pix + xstride);
    q2 = U16(pix + 2 * xstride);

    cond = _mm256_and_si256(on,
        lf_lt16_avx2(lf_abs16_avx2(_mm256_sub_epi16(p0, q0)), av));
    cond = _mm256_and_si256(cond,
        lf_lt16_avx2(lf_abs16_avx2(_mm256_sub_epi16(p1, p0)), bv));
    cond = _mm256_and_si256(cond,
        lf_lt16_avx2(lf_abs16_avx2(_mm256_sub_epi16(q1, q0)), bv));

    cp2 = lf_lt16_avx2(lf_abs16_avx2(_mm256_sub_epi16(p2, p0)), bv);
    cq2 = lf_lt16_avx2(lf_abs16_avx2(_mm256_sub_epi16(q2, q0)), bv);

    /* tc grows by one for each side that also passes the beta test; the
     * reference still raises it when tc is zero, hence the sum */
    tcx = _mm256_add_epi16(tc, _mm256_add_epi16(
        _mm256_and_si256(cp2, V1), _mm256_and_si256(cq2, V1)));
    mid = _mm256_srli_epi16(_mm256_add_epi16(_mm256_add_epi16(p0, q0), V1), 1);

    {
        const __m256i np1 = _mm256_and_si256(_mm256_add_epi16(p1,
            lf_clip_tc_avx2(_mm256_sub_epi16(
                _mm256_srli_epi16(_mm256_add_epi16(p2, mid), 1), p1), tc)),
            VFF);
        const __m256i nq1 = _mm256_and_si256(_mm256_add_epi16(q1,
            lf_clip_tc_avx2(_mm256_sub_epi16(
                _mm256_srli_epi16(_mm256_add_epi16(q2, mid), 1), q1), tc)),
            VFF);

        delta = _mm256_add_epi16(
            _mm256_slli_epi16(_mm256_sub_epi16(q0, p0), 2),
            _mm256_sub_epi16(p1, q1));
        delta = _mm256_add_epi16(delta, _mm256_set1_epi16(4));
        delta = lf_clip_tc_avx2(_mm256_srai_epi16(delta, 3), tcx);

        p1 = lf_blend_avx2(_mm256_and_si256(cond, cp2), np1, p1);
        q1 = lf_blend_avx2(_mm256_and_si256(cond, cq2), nq1, q1);
    }
    p0 = lf_blend_avx2(cond, _mm256_add_epi16(p0, delta), p0);
    q0 = lf_blend_avx2(cond, _mm256_sub_epi16(q0, delta), q0);

    lf_store16(pix - 2 * xstride, p1);
    lf_store16(pix - xstride, p0);
    lf_store16(pix, q0);
    lf_store16(pix + xstride, q1);
}

#pragma GCC pop_options

#endif /* x86 */
