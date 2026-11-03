/*
 * NEON kernels - the ARM counterpart of h264_dsp_sse2.c (8-bit 4:2:0).
 *
 * The structure follows the SSE2 file lane for lane, because both run one
 * 8-bit sample per int16 lane in a 128-bit register: eight lanes per vector,
 * the same intermediate widths, the same order of operations. The arithmetic is
 * chosen to match the scalar reference exactly rather than to be fastest:
 * wrapping int16 add/sub where the C code adds and subtracts ints that are then
 * truncated, saturating narrowing where the C code clips with Clip1, and an
 * arithmetic shift where it uses >>.
 *
 * Where SSE2 needs a trick and NEON has the instruction, the trick is gone:
 * pavgusb was already the same rounding as vhadd_u8, the unpack ladder that
 * SSE2 builds transposes come out of vtrn/vzip pairs, and a signed multiply by
 * a constant does not need the shift/add expansion the SSE2 path used to stay
 * inside the instruction set.
 *
 * Compiled only when the target has NEON, and reached only when h264_have_neon
 * says the CPU may use it; the scalar C kernels in h264_mc.c, h264_idct.c and
 * h264_deblock.c stay the reference and the fallback, and tests/dsp_check.c
 * compares the two sample by sample.
 */
#include "h264_internal.h"

#if defined(H264_SIMD_ARM)

#include <arm_neon.h>

/* one int16 lane per sample, always the "eight lanes" case */
#define ZERO16   vdupq_n_s16(0)
#define C20      vdupq_n_s16(20)
#define C5       vdupq_n_s16(5)
#define C16      vdupq_n_s16(16)
#define ONE16    vdupq_n_s16(1)
#define C255     vdupq_n_s16(255)
#define MASK_FF  vdupq_n_s16((short)0xFF)

/* 8 bytes of samples, zero-extended to 8 int16 lanes - SSE2's loadl+unpacklo */
#define U8(p)    vreinterpretq_s16_u16(vmovl_u8(vld1_u8(p)))
/* the next 8 bytes of the same 16-byte load, i.e. SSE2's unpackhi of a full
 * register: only the high half is wanted, so no 16-byte load is needed */
#define U8H(p)   U8((p) + 8)

/* SSE2's _mm_srai_epi16 / _mm_srli_epi16 on the same register */
#define SRA16(v, n)   vshrq_n_s16((v), (n))
#define SRL16(v, n)   vreinterpretq_s16_u16(vshrq_n_u16(vreinterpretq_u16_s16(v), (n)))

/* blends of mask/yes/no, the mask being all-ones lanes from a comparison */
#define LT16(a, b)    vreinterpretq_s16_u16(vcltq_s16(a, b))
#define ABS16(v)      vmaxq_s16(v, vsubq_s16(ZERO16, v))
#define CLIP8_16(v)   vminq_s16(vmaxq_s16(v, ZERO16), C255)
/* mask ? yes : no. vbic is "and not" with the *first* operand negated, so the
 * mask goes second here even though it is first in the SSE2 andnot it mirrors */
#define BLEND(m, y, n) \
    vorrq_s16(vandq_s16(m, y), vbicq_s16(n, m))
#define RND(v)        SRA16(vaddq_s16(v, C16), 5)

#define AVG16(a, b) \
    SRL16(vaddq_s16(vaddq_s16(CLIP8_16(a), CLIP8_16(b)), ONE16), 1)

/* round to byte range, optionally average onto dst, store the first w lanes */
static inline void put8(uint8_t *dst, int16x8_t v, int w, int avg)
{
    uint8x8_t b;

    if (avg) {
        /* the scalar path clips every half-pel sample to 0..255 before it
         * averages, so the clamp has to happen here as well */
        int16x8_t d = vreinterpretq_s16_u16(vmovl_u8(vld1_u8(dst)));
        v = vminq_s16(vmaxq_s16(v, ZERO16), C255);
        v = SRL16(vaddq_s16(vaddq_s16(v, d), ONE16), 1);
    }
    b = vqmovun_s16(v);                    /* saturating narrow, like packus */
    if (w >= 8) {
        vst1_u8(dst, b);
    } else {
        uint8_t tmp[8];
        int i;
        vst1_u8(tmp, b);
        for (i = 0; i < w; i++)
            dst[i] = tmp[i];
    }
}

/* ------------------------------------------------------------------ */
/* quarter-pel luma interpolation                                      */
/* ------------------------------------------------------------------ */

/* 8 unrounded horizontal 6-tap results for output columns p[0..7] */
static inline int16x8_t hfilter8_u(const uint8_t *p)
{
    int16x8_t acc = vmulq_s16(vaddq_s16(U8(p), U8(p + 1)), C20);
    acc = vsubq_s16(acc, vmulq_s16(vaddq_s16(U8(p - 1), U8(p + 2)), C5));
    return vaddq_s16(acc, vaddq_s16(U8(p - 2), U8(p + 3)));
}

/* 8 unrounded vertical 6-tap results for output columns p[0..7] */
static inline int16x8_t vfilter8_u(const uint8_t *p, int stride)
{
    int16x8_t acc = vmulq_s16(vaddq_s16(U8(p), U8(p + stride)), C20);
    acc = vsubq_s16(acc, vmulq_s16(vaddq_s16(U8(p - stride),
                                             U8(p + 2 * stride)), C5));
    return vaddq_s16(acc, vaddq_s16(U8(p - 2 * stride), U8(p + 3 * stride)));
}

static void qpel_h_neon(uint8_t *dst, const uint8_t *src, int stride,
                        int w, int h, int x, int avg)
{
    int j, i;

    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i += 8) {
            const uint8_t *p = src + i;
            int16x8_t hp = RND(hfilter8_u(p));
            int16x8_t v = (x == 2) ? hp : AVG16(U8(p + (x >> 1)), hp);
            put8(dst + i, v, w - i, avg);
        }
        dst += stride;
        src += stride;
    }
}

static void qpel_v_neon(uint8_t *dst, const uint8_t *src, int stride,
                        int w, int h, int y, int avg)
{
    int j, i;

    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i += 8) {
            const uint8_t *p = src + i;
            int16x8_t hp = RND(vfilter8_u(p, stride));
            int16x8_t v = (y == 2) ? hp
                                   : AVG16(U8(p + (y == 3 ? stride : 0)), hp);
            put8(dst + i, v, w - i, avg);
        }
        dst += stride;
        src += stride;
    }
}

/* The horizontal pass reaches ~11000 and the vertical combination of six of
 * them needs ~440000, so the second pass runs on int32 lanes, exactly as the
 * SSE2 kernel does. NEON widens an int16 vector to two int32 vectors directly,
 * which is what the U4/TRANSPOSE dance was working around. */
#define HW_W 24

/* four int16 samples at (row+off) widened to four int32 lanes */
static inline int32x4_t widen4_s(const int16_t *p)
{
    return vmovl_s16(vld1_s16(p));
}

static inline int32x4_t vsum32_4(const int16_t *p, int n)
{
    int32x4_t a = widen4_s(p),     b = widen4_s(p + n),     c = widen4_s(p - n);
    int32x4_t d = widen4_s(p + 2 * n), e = widen4_s(p - 2 * n), f = widen4_s(p + 3 * n);
    int32x4_t acc = vaddq_s32(vmulq_s32(vaddq_s32(a, b), vdupq_n_s32(20)), e);
    acc = vsubq_s32(acc, vmulq_s32(vaddq_s32(c, d), vdupq_n_s32(5)));
    return vaddq_s32(acc, f);
}

/* 8 unrounded vertical results, as two int32 vectors */
static inline void vsum32_8(const int16_t *p, int n, int32x4_t *lo, int32x4_t *hi)
{
    const int32x4_t k512 = vdupq_n_s32(512);
    int32x4_t a = vsum32_4(p, n), b = vsum32_4(p + 4, n);
    *lo = vshrq_n_s32(vaddq_s32(a, k512), 10);
    *hi = vshrq_n_s32(vaddq_s32(b, k512), 10);
}

static inline int16x8_t pack_2s32(int32x4_t lo, int32x4_t hi)
{
    /* signed saturating narrow, matching packs_epi32 */
    return vcombine_s16(vqmovn_s32(lo), vqmovn_s32(hi));
}

static void qpel_hv_neon(uint8_t *dst, const uint8_t *src, int stride,
                         int w, int h, int x, int y, int avg)
{
    int16_t tmp[(16 + 5) * HW_W];
    int j, i;

    for (j = -2; j < h + 3; j++) {
        int16_t *row = tmp + (j + 2) * HW_W;
        for (i = 0; i < HW_W; i += 8)
            vst1q_s16(row + i, hfilter8_u(src + j * stride + i));
    }

    for (j = 0; j < h; j++) {
        const int16_t *t = tmp + (j + 2) * HW_W;   /* source row j */
        for (i = 0; i < w; i += 8) {
            const uint8_t *p = src + i;
            int32x4_t lo, hi;
            int16x8_t hv;
            int16x8_t hh = RND(vld1q_s16(t + (y >> 1) * HW_W + i));

            vsum32_8(t + i, HW_W, &lo, &hi);
            hv = pack_2s32(lo, hi);
            if (x == 2 && y == 2) {
                put8(dst + i, hv, w - i, avg);
            } else if (x == 2) {
                put8(dst + i, AVG16(hh, hv), w - i, avg);
            } else if (y == 2) {
                int16x8_t vv = RND(vfilter8_u(p + (x == 3), stride));
                put8(dst + i, AVG16(vv, hv), w - i, avg);
            } else {
                int16x8_t vv = RND(vfilter8_u(p + (x == 3), stride));
                put8(dst + i, AVG16(hh, vv), w - i, avg);
            }
        }
        dst += stride;
        src += stride;
    }
}

void h264_qpel_mc_neon(uint8_t *dst, const uint8_t *src, int stride,
                       int w, int h, int x, int y, int avg)
{
    if (!y)
        qpel_h_neon(dst, src, stride, w, h, x, avg);
    else if (!x)
        qpel_v_neon(dst, src, stride, w, h, y, avg);
    else
        qpel_hv_neon(dst, src, stride, w, h, x, y, avg);
}

/* ------------------------------------------------------------------ */
/* chroma interpolation                                                */
/* ------------------------------------------------------------------ */
void h264_chroma_mc_neon(uint8_t *dst, const uint8_t *src, int stride,
                         int w, int h, int x, int y, int avg)
{
    const int16x8_t A = vdupq_n_s16((8 - x) * (8 - y));
    const int16x8_t B = vdupq_n_s16(x * (8 - y));
    const int16x8_t C = vdupq_n_s16((8 - x) * y);
    const int16x8_t D = vdupq_n_s16(x * y);
    const int16x8_t rnd = vdupq_n_s16(32);
    int j, i;

    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i += 8) {
            const uint8_t *p = src + i;
            const uint8_t *q = p + stride;
            int16x8_t v = vaddq_s16(
                vaddq_s16(vmulq_s16(U8(p), A), vmulq_s16(U8(p + 1), B)),
                vaddq_s16(vmulq_s16(U8(q), C), vmulq_s16(U8(q + 1), D)));
            v = SRA16(vaddq_s16(v, rnd), 6);
            put8(dst + i, v, w - i, avg);
        }
        dst += stride;
        src += stride;
    }
}

/* ------------------------------------------------------------------ */
/* full-pel copy and average                                           */
/* ------------------------------------------------------------------ */
/* The rounding the scalar rnd_avg uses is (a + b + 1) >> 1. vhadd_u8 is only
 * (a + b) >> 1, and the instruction that does round to nearest, vrhadd_u8, is
 * an ARMv8.1 extension this kernel may not emit, so the average is the
 * truncating halve plus the low bit of the sum - which is the low bit of a^b,
 * and the add cannot carry out because a truncating halve of 509 is 254. */
static inline uint8x8_t avg_u8(uint8x8_t a, uint8x8_t b)
{
    const uint8x8_t round = veor_u8(a, b);
    const uint8x8_t ones = vdup_n_u8(1);

    return vadd_u8(vhadd_u8(a, b), vand_u8(round, ones));
}

static inline uint8x16_t avg_u8q(uint8x16_t a, uint8x16_t b)
{
    const uint8x16_t round = veorq_u8(a, b);
    const uint8x16_t ones = vdupq_n_u8(1);

    return vaddq_u8(vhaddq_u8(a, b), vandq_u8(round, ones));
}

static inline void hpel_row(uint8_t *dst, const uint8_t *src, int w, int avg)
{
    if (w >= 16) {
        uint8x16_t v = vld1q_u8(src);
        if (avg)
            v = avg_u8q(vld1q_u8(dst), v);
        vst1q_u8(dst, v);
        return;
    }
    /* narrower rows still move 8 bytes: the planes carry a 16-pixel margin, so
     * the extra bytes are there, and only w of them are written */
    {
        uint8x8_t v = vld1_u8(src);
        if (avg)
            v = avg_u8(vld1_u8(dst), v);
        if (w >= 8) {
            vst1_u8(dst, v);
        } else {
            uint8_t tmp[8];
            int i;
            vst1_u8(tmp, v);
            for (i = 0; i < w; i++)
                dst[i] = tmp[i];
        }
    }
}

void h264_hpel_mc_neon(uint8_t *dst, const uint8_t *src, int stride,
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
/* One int16 lane per pixel, like the SSE2 kernel: the dispatcher only routes a
 * call here when the products cannot overflow, and the saturating add is then
 * invisible after the shift. vqaddq_s16 is paddsw, vshrq_n_s16 is psraw and
 * vqmovun_s16 is the clip_u8 of packus. */
static inline int16x8_t wp_scale(int16x8_t x, int16x8_t wv, int16x8_t ov,
                                 int shift)
{
    int16x8_t s = vqaddq_s16(vmulq_s16(x, wv), ov);

    return vshrq_n_s16(s, shift);
}

static inline int16x8_t wp_bipt(int16x8_t a, int16x8_t b, int16x8_t wa,
                                int16x8_t wb, int16x8_t ov, int shift)
{
    int16x8_t s = vqaddq_s16(vmulq_s16(a, wa), vmulq_s16(b, wb));

    return vshrq_n_s16(vqaddq_s16(s, ov), shift);
}

/* the low n bytes of a packed vector; a sub-block is not necessarily aligned
 * to anything, so this goes through a temporary rather than a vector store */
static inline void wp_store(uint8_t *dst, uint8x8_t b, int n)
{
    uint8_t tmp[8];
    int i;

    vst1_u8(tmp, b);
    for (i = 0; i < n; i++)
        dst[i] = tmp[i];
}

static inline int16x8_t wp_load8(const uint8_t *p)
{
    return vreinterpretq_s16_u16(vmovl_u8(vld1_u8(p)));
}

static inline int16x8_t wp_load8_hi(const uint8_t *p)
{
    return vreinterpretq_s16_u16(vmovl_u8(vget_high_u8(vld1q_u8(p))));
}

void h264_weight_pixels_neon(uint8_t *dst, int stride, int w, int h,
                             int log2_denom, int weight, int offset)
{
    int16x8_t wv, ov;
    int i, j;

    offset = (unsigned)offset << log2_denom;
    if (log2_denom)
        offset += 1 << (log2_denom - 1);
    wv = vdupq_n_s16((short)weight);
    ov = vdupq_n_s16((short)offset);

    for (j = 0; j < h; j++) {
        for (i = 0; i + 16 <= w; i += 16) {
            int16x8_t lo = wp_scale(wp_load8(dst + i), wv, ov, log2_denom);
            int16x8_t hi = wp_scale(wp_load8_hi(dst + i), wv, ov, log2_denom);
            uint8x8_t b0 = vqmovun_s16(lo), b1 = vqmovun_s16(hi);
            vst1q_u8(dst + i, vcombine_u8(b0, b1));
        }
        for (; i + 8 <= w; i += 8) {
            int16x8_t v = wp_scale(wp_load8(dst + i), wv, ov, log2_denom);
            vst1_u8(dst + i, vqmovun_s16(v));
        }
        if (i < w) {
            int16x8_t v = wp_scale(wp_load8(dst + i), wv, ov, log2_denom);
            wp_store(dst + i, vqmovun_s16(v), w - i);
        }
        dst += stride;
    }
}

void h264_biweight_pixels_neon(uint8_t *dst, const uint8_t *src, int stride,
                               int w, int h, int log2_denom, int weightd,
                               int weights, int offset)
{
    int16x8_t wd, ws, ov;
    int i, j, shift = log2_denom + 1;

    offset = (unsigned)((offset + 1) | 1) << log2_denom;
    wd = vdupq_n_s16((short)weightd);
    ws = vdupq_n_s16((short)weights);
    ov = vdupq_n_s16((short)offset);

    for (j = 0; j < h; j++) {
        for (i = 0; i + 16 <= w; i += 16) {
            int16x8_t lo = wp_bipt(wp_load8(dst + i), wp_load8(src + i),
                                   wd, ws, ov, shift);
            int16x8_t hi = wp_bipt(wp_load8_hi(dst + i), wp_load8_hi(src + i),
                                   wd, ws, ov, shift);
            uint8x8_t b0 = vqmovun_s16(lo), b1 = vqmovun_s16(hi);
            vst1q_u8(dst + i, vcombine_u8(b0, b1));
        }
        for (; i + 8 <= w; i += 8) {
            int16x8_t v = wp_bipt(wp_load8(dst + i), wp_load8(src + i),
                                  wd, ws, ov, shift);
            vst1_u8(dst + i, vqmovun_s16(v));
        }
        if (i < w) {
            int16x8_t v = wp_bipt(wp_load8(dst + i), wp_load8(src + i),
                                  wd, ws, ov, shift);
            wp_store(dst + i, vqmovun_s16(v), w - i);
        }
        dst += stride;
        src += stride;
    }
}

void h264_avg_pixels_neon(uint8_t *dst, const uint8_t *src, int stride,
                          int w, int h)
{
    int j;

    /* the identity weighting is the plain bi-prediction average, and vhadd_u8
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
/* Translating the SSE2 IDCT4_ADD structure, and for the same reason: the first
 * stage stays in int16 lanes because the scalar reference truncates every stage
 * result to int16, and the second runs in int32 because the scalar code computes
 * its sums in 32 bits - widening keeps the two paths identical for every input
 * rather than only for the range the specification allows.
 *
 * Each of the four registers holds one row of the block in its four low lanes,
 * so the first butterfly processes all four rows at once. The rows then become
 * columns and the second butterfly processes all four columns at once, which is
 * why register j holds row j of the result afterwards and no second transpose
 * is needed before the store. */

/* SSE2's unpacklo_epi16 pair is NEON's zip1 on 16-bit lanes, its unpacklo and
 * unpackhi_epi32 are zip1 and zip2 on 32-bit lanes, and the 8-byte shift that
 * picks the second half is a four-lane extract. */
static inline void transpose4w(int16x8_t *r0, int16x8_t *r1,
                               int16x8_t *r2, int16x8_t *r3)
{
    const int16x8_t a = *r0, b = *r1, c = *r2, d = *r3;
    const int16x8_t t0 = vzip1q_s16(a, b);      /* a0 b0 a1 b1 a2 b2 a3 b3 */
    const int16x8_t t1 = vzip1q_s16(c, d);      /* c0 d0 c1 d1 c2 d2 c3 d3 */
    const int16x8_t s0 = vreinterpretq_s16_s32(vzip1q_s32(
                           vreinterpretq_s32_s16(t0), vreinterpretq_s32_s16(t1)));
    const int16x8_t s2 = vreinterpretq_s16_s32(vzip2q_s32(
                           vreinterpretq_s32_s16(t0), vreinterpretq_s32_s16(t1)));
    const int16x8_t z = vdupq_n_s16(0);

    *r0 = s0;                        /* column 0 in lanes 0..3 */
    *r2 = s2;                        /* column 2 */
    *r1 = vextq_s16(s0, z, 4);       /* column 1 */
    *r3 = vextq_s16(s2, z, 4);       /* column 3 */
}

/* int16 in the low four lanes to int32 in all four lanes */
static inline int32x4_t widen4(int16x8_t v)
{
    return vmovl_s16(vget_low_s16(v));
}

#define IDCT4_1D32(r0, r1, r2, r3, a0, a1, a2, a3)   \
    do {                                              \
        a0 = vaddq_s32(r0, r2);                       \
        a1 = vsubq_s32(r0, r2);                       \
        a2 = vsubq_s32(vshrq_n_s32(r1, 1), r3);       \
        a3 = vaddq_s32(r1, vshrq_n_s32(r3, 1));       \
        r0 = vaddq_s32(a0, a3);                       \
        r1 = vaddq_s32(a1, a2);                       \
        r2 = vsubq_s32(a1, a2);                       \
        r3 = vsubq_s32(a0, a3);                       \
    } while (0)

#define IDCT4_1D16(r0, r1, r2, r3, a0, a1, a2, a3)   \
    do {                                              \
        a0 = vaddq_s16(r0, r2);                       \
        a1 = vsubq_s16(r0, r2);                       \
        a2 = vsubq_s16(SRA16(r1, 1), r3);             \
        a3 = vaddq_s16(r1, SRA16(r3, 1));             \
        r0 = vaddq_s16(a0, a3);                       \
        r1 = vaddq_s16(a1, a2);                       \
        r2 = vsubq_s16(a1, a2);                       \
        r3 = vsubq_s16(a0, a3);                       \
    } while (0)

/* dst[0..3] += (delta >> 6), with the Clip1 saturation of the scalar version:
 * widen the four bytes to int32, add, then narrow twice with saturation, which
 * is int32 -> int16 -> unsigned 0..255 in the same order the C code clips. */
static inline void idct_store_row(uint8_t *dst, int32x4_t delta)
{
    const uint16x4_t lo = vget_low_u16(vmovl_u8(vld1_u8(dst)));
    const int32x4_t cur = vreinterpretq_s32_u32(vmovl_u16(lo));
    const int16x4_t h = vqmovn_s32(vaddq_s32(cur, vshrq_n_s32(delta, 6)));
    const uint8x8_t b = vqmovun_s16(vcombine_s16(h, vdup_n_s16(0)));
    const uint32_t w = vget_lane_u32(vreinterpret_u32_u8(b), 0);

    memcpy(dst, &w, 4);
}

void h264_idct_add_neon(uint8_t *dst, int16_t *block, int stride)
{
    int16x8_t r0 = vld1q_s16(block +  0);
    int16x8_t r1 = vld1q_s16(block +  4);
    int16x8_t r2 = vld1q_s16(block +  8);
    int16x8_t r3 = vld1q_s16(block + 12);
    int16x8_t a0, a1, a2, a3;
    int32x4_t d0, d1, d2, d3, b0, b1, b2, b3;

    /* block[0] += 1 << 5, in the same int16 arithmetic as the scalar code. The
     * constant is built in a register rather than loaded from a literal, so no
     * vector load has to trust the alignment of a stack temporary. */
    r0 = vaddq_s16(r0, vsetq_lane_s16(32, vdupq_n_s16(0), 0));
    IDCT4_1D16(r0, r1, r2, r3, a0, a1, a2, a3);
    transpose4w(&r0, &r1, &r2, &r3);

    d0 = widen4(r0);
    d1 = widen4(r1);
    d2 = widen4(r2);
    d3 = widen4(r3);
    IDCT4_1D32(d0, d1, d2, d3, b0, b1, b2, b3);

    idct_store_row(dst,              d0);
    idct_store_row(dst + stride,     d1);
    idct_store_row(dst + 2 * stride, d2);
    idct_store_row(dst + 3 * stride, d3);
    memset(block, 0, 16 * sizeof(int16_t));
}

/* dst[0..n) += delta, clipped to a byte like the scalar H264_CLIP_PIXEL. The
 * rows are margined, so an 8-byte read is in bounds even for a 4-wide block,
 * and only n bytes go back. */
static inline void dc_add_row(uint8_t *dst, int16x8_t delta, int n)
{
    int16x8_t cur = vreinterpretq_s16_u16(vmovl_u8(vld1_u8(dst)));
    uint8_t tmp[8];
    int i;

    vst1_u8(tmp, vqmovun_s16(vaddq_s16(cur, delta)));
    for (i = 0; i < n; i++)
        dst[i] = tmp[i];
}

void h264_idct_dc_add_neon(uint8_t *dst, int16_t *block, int stride)
{
    int i;
    int16x8_t delta;

    /* (block[0] + 32) >> 6 in 32 bits, as the scalar reference computes it */
    delta = vdupq_n_s16((int16_t)((block[0] + 32) >> 6));
    block[0] = 0;
    for (i = 0; i < 4; i++)
        dc_add_row(dst + i * stride, delta, 4);
}

void h264_idct8_dc_add_neon(uint8_t *dst, int16_t *block, int stride)
{
    int i;
    int16x8_t delta;

    delta = vdupq_n_s16((int16_t)((block[0] + 32) >> 6));
    block[0] = 0;
    for (i = 0; i < 8; i++)
        dc_add_row(dst + i * stride, delta, 8);
}
/* ------------------------------------------------------------------ */
/* in-loop deblocking                                                  */
/* ------------------------------------------------------------------ */
/* The same shape as the SSE2 kernels: one int16 lane per edge position, the
 * eight taps of a position in separate registers, and whichever axis is
 * contiguous in memory decides how the taps get loaded. int16 is wide enough
 * for every intermediate - the widest term here is 4*(q0-p0) + (p1-q1) + 4,
 * under 1300 for byte samples. */

/* r[i] holds 8 samples of row i; afterwards r[k] holds tap k of 8 rows.
 * SSE2 builds this out of unpacklo/unpackhi pairs; the direct equivalents here
 * are the zip pairs - vzip1 interleaves the low halves of each element group
 * and vzip2 the high halves, which is exactly what unpacklo/unpackhi do. (TRN
 * is the other family: it takes even- and odd-numbered lanes, and would be a
 * different - also correct - way to build the same transpose, but the ladder
 * below is a line-for-line translation of the kernel that is already proven.) */
static inline void lf_transpose8(int16x8_t r[8])
{
    const int16x8_t a0 = vzip1q_s16(r[0], r[1]);
    const int16x8_t a1 = vzip2q_s16(r[0], r[1]);
    const int16x8_t a2 = vzip1q_s16(r[2], r[3]);
    const int16x8_t a3 = vzip2q_s16(r[2], r[3]);
    const int16x8_t a4 = vzip1q_s16(r[4], r[5]);
    const int16x8_t a5 = vzip2q_s16(r[4], r[5]);
    const int16x8_t a6 = vzip1q_s16(r[6], r[7]);
    const int16x8_t a7 = vzip2q_s16(r[6], r[7]);
    const int32x4_t b0 = vzip1q_s32(vreinterpretq_s32_s16(a0), vreinterpretq_s32_s16(a2));
    const int32x4_t b1 = vzip2q_s32(vreinterpretq_s32_s16(a0), vreinterpretq_s32_s16(a2));
    const int32x4_t b2 = vzip1q_s32(vreinterpretq_s32_s16(a1), vreinterpretq_s32_s16(a3));
    const int32x4_t b3 = vzip2q_s32(vreinterpretq_s32_s16(a1), vreinterpretq_s32_s16(a3));
    const int32x4_t b4 = vzip1q_s32(vreinterpretq_s32_s16(a4), vreinterpretq_s32_s16(a6));
    const int32x4_t b5 = vzip2q_s32(vreinterpretq_s32_s16(a4), vreinterpretq_s32_s16(a6));
    const int32x4_t b6 = vzip1q_s32(vreinterpretq_s32_s16(a5), vreinterpretq_s32_s16(a7));
    const int32x4_t b7 = vzip2q_s32(vreinterpretq_s32_s16(a5), vreinterpretq_s32_s16(a7));

    r[0] = vreinterpretq_s16_s64(vzip1q_s64(vreinterpretq_s64_s32(b0), vreinterpretq_s64_s32(b4)));
    r[1] = vreinterpretq_s16_s64(vzip2q_s64(vreinterpretq_s64_s32(b0), vreinterpretq_s64_s32(b4)));
    r[2] = vreinterpretq_s16_s64(vzip1q_s64(vreinterpretq_s64_s32(b1), vreinterpretq_s64_s32(b5)));
    r[3] = vreinterpretq_s16_s64(vzip2q_s64(vreinterpretq_s64_s32(b1), vreinterpretq_s64_s32(b5)));
    r[4] = vreinterpretq_s16_s64(vzip1q_s64(vreinterpretq_s64_s32(b2), vreinterpretq_s64_s32(b6)));
    r[5] = vreinterpretq_s16_s64(vzip2q_s64(vreinterpretq_s64_s32(b2), vreinterpretq_s64_s32(b6)));
    r[6] = vreinterpretq_s16_s64(vzip1q_s64(vreinterpretq_s64_s32(b3), vreinterpretq_s64_s32(b7)));
    r[7] = vreinterpretq_s16_s64(vzip2q_s64(vreinterpretq_s64_s32(b3), vreinterpretq_s64_s32(b7)));
}

/* the four pixels the inter luma filter may rewrite */
typedef struct LF4 {
    int16x8_t p1, p0, q0, q1;
} LF4;

/* the six pixels an intra luma edge may rewrite */
typedef struct LF6 {
    int16x8_t p2, p1, p0, q0, q1, q2;
} LF6;

/* clamp to [-tc, tc], as H264_CLIP does */
static inline int16x8_t lf_clip_tc(int16x8_t v, int16x8_t tc)
{
    return vminq_s16(vmaxq_s16(v, vsubq_s16(ZERO16, tc)), tc);
}

/* i_delta of the reference: ((q0-p0)*4 + (p1-q1) + 4) >> 3, arithmetic shift */
static inline int16x8_t lf_delta(int16x8_t p0, int16x8_t p1, int16x8_t q0,
                                 int16x8_t q1, int16x8_t tc)
{
    int16x8_t v = vaddq_s16(vshlq_n_s16(vsubq_s16(q0, p0), 2),
                            vsubq_s16(p1, q1));
    v = vaddq_s16(v, vdupq_n_s16(4));
    return lf_clip_tc(SRA16(v, 3), tc);
}

/* per-lane clipping range and enable for the eight positions of one pass;
 * position p belongs to group p / inner_iters, which is where tc0 comes from */
static inline int lf_group_masks(const int8_t *tc0, int inner_iters, int pos0, int n,
                                 int skip_when_zero, int16x8_t *tc, int16x8_t *on)
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
    *tc = vld1q_s16(vtc);
    *on = vld1q_s16(von);
    return any;
}

/* store n samples of a pass back to memory: v already holds packed bytes, so
 * a wide row is one 8-byte store and a narrow one writes the 4 bytes the pass
 * can reach - the lanes past n carry the samples they were loaded with, which
 * is what makes re-storing them harmless */
static inline void lf_store_n(uint8_t *p, int16x8_t v, int n)
{
    const uint8x8_t b = vqmovun_s16(v);

    if (n >= 8) {
        vst1_u8(p, b);
    } else {
        const uint32_t w = vget_lane_u32(vreinterpret_u32_u8(b), 0);
        memcpy(p, &w, 4);
    }
}

static inline int16x8_t lf_pack8(int16x8_t v)
{
    return vreinterpretq_s16_u16(vmovl_u8(vqmovun_s16(v)));
}

/* the eight taps of a pass in the xstride == 1 layout: 8 consecutive
 * positions, eight bytes each, transposed into one register per tap */
static inline void lf_load_rows(uint8_t *pix, int ystride, int pos0, int n, int16x8_t t[8])
{
    uint8_t *base = pix + pos0 * ystride;
    int i;

    for (i = 0; i < 8; i++)
        t[i] = U8(base + (i < n ? i : n - 1) * ystride - 4);
    lf_transpose8(t);
}

static inline void lf_store_rows(uint8_t *pix, int ystride, int pos0, int n, int16x8_t t[8])
{
    uint8_t *base = pix + pos0 * ystride;
    int i;

    lf_transpose8(t);
    for (i = 0; i < n; i++)
        lf_store_n(base + i * ystride - 4, lf_pack8(t[i]), 8);
}

static inline int16x8_t lf_lanes_used(int n)
{
    int16_t v[8];
    int i;

    for (i = 0; i < 8; i++)
        v[i] = i < n ? (int16_t)0xFFFF : 0;
    return vld1q_s16(v);
}

static inline LF4 lf_luma_inter(LF4 e, int16x8_t p2, int16x8_t q2,
                                int16x8_t tc, int16x8_t on,
                                int16x8_t alpha, int16x8_t beta)
{
    int16x8_t cond = vandq_s16(on, LT16(ABS16(vsubq_s16(e.p0, e.q0)), alpha));
    cond = vandq_s16(cond, LT16(ABS16(vsubq_s16(e.p1, e.p0)), beta));
    cond = vandq_s16(cond, LT16(ABS16(vsubq_s16(e.q1, e.q0)), beta));

    {
        const int16x8_t cp2 = LT16(ABS16(vsubq_s16(p2, e.p0)), beta);
        const int16x8_t cq2 = LT16(ABS16(vsubq_s16(q2, e.q0)), beta);
        /* tc grows by one for each side that also passes the beta test; the
         * reference still raises it when tc is zero, hence the sum and not
         * the p1/q1 write is what the zero case depends on */
        const int16x8_t tcx = vaddq_s16(tc, vandq_s16(cp2, ONE16));
        const int16x8_t mid = SRL16(vaddq_s16(vaddq_s16(e.p0, e.q0), ONE16), 1);
        /* a clip of zero is a no-op, so the reference's "if (tc_orig)" guard
         * needs no mask of its own */
        const int16x8_t np1 = vandq_s16(vaddq_s16(e.p1, lf_clip_tc(
                vsubq_s16(SRL16(vaddq_s16(p2, mid), 1), e.p1), tc)), MASK_FF);
        const int16x8_t nq1 = vandq_s16(vaddq_s16(e.q1, lf_clip_tc(
                vsubq_s16(SRL16(vaddq_s16(q2, mid), 1), e.q1), tc)), MASK_FF);
        const int16x8_t delta = lf_delta(e.p0, e.p1, e.q0, e.q1,
                vaddq_s16(tcx, vandq_s16(cq2, ONE16)));

        /* p1/q1 wrap like the reference's (uint8_t) cast; p0/q0 saturate like
         * its H264_CLIP_PIXEL, so they stay raw and the pack does the clamp */
        e.p1 = BLEND(vandq_s16(cond, cp2), np1, e.p1);
        e.q1 = BLEND(vandq_s16(cond, cq2), nq1, e.q1);
        e.p0 = BLEND(cond, vaddq_s16(e.p0, delta), e.p0);
        e.q0 = BLEND(cond, vsubq_s16(e.q0, delta), e.q0);
    }
    return e;
}

static inline LF4 lf_chroma_inter(LF4 e, int16x8_t tc, int16x8_t on,
                                  int16x8_t alpha, int16x8_t beta)
{
    int16x8_t cond = vandq_s16(on, LT16(ABS16(vsubq_s16(e.p0, e.q0)), alpha));
    cond = vandq_s16(cond, LT16(ABS16(vsubq_s16(e.p1, e.p0)), beta));
    cond = vandq_s16(cond, LT16(ABS16(vsubq_s16(e.q1, e.q0)), beta));

    {
        const int16x8_t delta = lf_delta(e.p0, e.p1, e.q0, e.q1, tc);
        e.p0 = BLEND(cond, vaddq_s16(e.p0, delta), e.p0);
        e.q0 = BLEND(cond, vsubq_s16(e.q0, delta), e.q0);
    }
    return e;
}

void h264_loop_filter_luma_neon(uint8_t *pix, int xstride, int ystride,
                                int inner_iters, int alpha, int beta,
                                const int8_t *tc0)
{
    const int16x8_t av = vdupq_n_s16((short)alpha);
    const int16x8_t bv = vdupq_n_s16((short)beta);
    const int total = 4 * inner_iters;
    int pos0;

    if (tc0[0] < 0 && tc0[1] < 0 && tc0[2] < 0 && tc0[3] < 0)
        return;

    for (pos0 = 0; pos0 < total; pos0 += 8) {
        const int n = total - pos0 < 8 ? total - pos0 : 8;
        int16x8_t tc, on, t[8];
        LF4 e;

        if (!lf_group_masks(tc0, inner_iters, pos0, n, 0, &tc, &on))
            continue;

        if (ystride == 1) {
            const uint8_t *b = pix + pos0;
            e.p1 = U8(b - 2 * xstride);
            e.p0 = U8(b - xstride);
            e.q0 = U8(b);
            e.q1 = U8(b + xstride);
            e = lf_luma_inter(e, U8(b - 3 * xstride), U8(b + 2 * xstride),
                              tc, on, av, bv);
            lf_store_n((uint8_t *)(b - 2 * xstride), lf_pack8(e.p1), n);
            lf_store_n((uint8_t *)(b - xstride),     lf_pack8(e.p0), n);
            lf_store_n((uint8_t *)b,                 lf_pack8(e.q0), n);
            lf_store_n((uint8_t *)(b + xstride),     lf_pack8(e.q1), n);
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

void h264_loop_filter_chroma_neon(uint8_t *pix, int xstride, int ystride,
                                  int inner_iters, int alpha, int beta,
                                  const int8_t *tc0)
{
    const int16x8_t av = vdupq_n_s16((short)alpha);
    const int16x8_t bv = vdupq_n_s16((short)beta);
    const int total = 4 * inner_iters;
    int pos0;

    if (tc0[0] <= 0 && tc0[1] <= 0 && tc0[2] <= 0 && tc0[3] <= 0)
        return;

    for (pos0 = 0; pos0 < total; pos0 += 8) {
        const int n = total - pos0 < 8 ? total - pos0 : 8;
        int16x8_t tc, on, t[8];
        LF4 e;

        if (!lf_group_masks(tc0, inner_iters, pos0, n, 1, &tc, &on))
            continue;

        if (ystride == 1) {
            const uint8_t *b = pix + pos0;
            e.p1 = U8(b - 2 * xstride);
            e.p0 = U8(b - xstride);
            e.q0 = U8(b);
            e.q1 = U8(b + xstride);
            e = lf_chroma_inter(e, tc, on, av, bv);
            lf_store_n((uint8_t *)(b - xstride), lf_pack8(e.p0), n);
            lf_store_n((uint8_t *)b,            lf_pack8(e.q0), n);
        } else {
            lf_load_rows(pix, ystride, pos0, n, t);
            e.p1 = t[2]; e.p0 = t[3]; e.q0 = t[4]; e.q1 = t[5];
            e = lf_chroma_inter(e, tc, on, av, bv);
            t[3] = e.p0; t[4] = e.q0;
            lf_store_rows(pix, ystride, pos0, n, t);
        }
    }
}

static inline LF4 lf_chroma_intra(LF4 e, int16x8_t on, int16x8_t alpha,
                                  int16x8_t beta)
{
    const int16x8_t two = vdupq_n_s16(2);
    int16x8_t cond = vandq_s16(on, LT16(ABS16(vsubq_s16(e.p0, e.q0)), alpha));

    cond = vandq_s16(cond, LT16(ABS16(vsubq_s16(e.p1, e.p0)), beta));
    cond = vandq_s16(cond, LT16(ABS16(vsubq_s16(e.q1, e.q0)), beta));
    e.p0 = BLEND(cond, SRL16(vaddq_s16(vaddq_s16(vshlq_n_s16(e.p1, 1), e.p0),
                                      vaddq_s16(e.q1, two)), 2), e.p0);
    e.q0 = BLEND(cond, SRL16(vaddq_s16(vaddq_s16(vshlq_n_s16(e.q1, 1), e.q0),
                                      vaddq_s16(e.p1, two)), 2), e.q0);
    return e;
}

static inline LF6 lf_luma_intra(LF6 e, int16x8_t p3, int16x8_t q3,
                                int16x8_t on, int16x8_t alpha, int16x8_t beta,
                                int16x8_t alpha_q)
{
    const int16x8_t p2 = e.p2, p1 = e.p1, p0 = e.p0, q0 = e.q0, q1 = e.q1, q2 = e.q2;
    const int16x8_t two = vdupq_n_s16(2), four = vdupq_n_s16(4);
    const int16x8_t d0 = ABS16(vsubq_s16(p0, q0));
    int16x8_t cond = vandq_s16(on, LT16(d0, alpha));
    int16x8_t strong, mp2, mq2;

    cond = vandq_s16(cond, LT16(ABS16(vsubq_s16(p1, p0)), beta));
    cond = vandq_s16(cond, LT16(ABS16(vsubq_s16(q1, q0)), beta));
    strong = vandq_s16(cond, LT16(d0, alpha_q));
    mp2 = vandq_s16(strong, LT16(ABS16(vsubq_s16(p2, p0)), beta));
    mq2 = vandq_s16(strong, LT16(ABS16(vsubq_s16(q2, q0)), beta));

    /* every candidate reads the samples the reference loaded first, even
     * though the reference writes the p side before it computes the q side;
     * sums of samples only, so the widest is 8*255+4 and logical shifts work */
    {
        const int16x8_t p0_w = SRL16(vaddq_s16(vaddq_s16(vshlq_n_s16(p1, 1), p0),
                                               vaddq_s16(q1, two)), 2);
        const int16x8_t q0_w = SRL16(vaddq_s16(vaddq_s16(vshlq_n_s16(q1, 1), q0),
                                               vaddq_s16(p1, two)), 2);
        const int16x8_t p0_s = SRL16(vaddq_s16(vaddq_s16(vaddq_s16(p2, vshlq_n_s16(p1, 1)),
                                                        vshlq_n_s16(p0, 1)),
                                               vaddq_s16(vaddq_s16(vshlq_n_s16(q0, 1), q1), four)), 3);
        const int16x8_t p1_s = SRL16(vaddq_s16(vaddq_s16(p2, p1),
                                               vaddq_s16(p0, vaddq_s16(q0, two))), 2);
        const int16x8_t p2_s = SRL16(vaddq_s16(vaddq_s16(vaddq_s16(vshlq_n_s16(p3, 1),
                                                                  vshlq_n_s16(p2, 1)), p2),
                                               vaddq_s16(vaddq_s16(p1, p0), vaddq_s16(q0, four))), 3);
        const int16x8_t q0_s = SRL16(vaddq_s16(vaddq_s16(vaddq_s16(q2, vshlq_n_s16(q1, 1)),
                                                        vshlq_n_s16(q0, 1)),
                                               vaddq_s16(vaddq_s16(vshlq_n_s16(p0, 1), p1), four)), 3);
        const int16x8_t q1_s = SRL16(vaddq_s16(vaddq_s16(q2, q1),
                                               vaddq_s16(q0, vaddq_s16(p0, two))), 2);
        const int16x8_t q2_s = SRL16(vaddq_s16(vaddq_s16(vaddq_s16(vshlq_n_s16(q3, 1),
                                                                  vshlq_n_s16(q2, 1)), q2),
                                               vaddq_s16(vaddq_s16(q1, q0), vaddq_s16(p0, four))), 3);

        /* a side that fails its own beta test, and the whole weak case, fall
         * back to the 4-tap average on p0/q0 only */
        e.p0 = BLEND(cond, BLEND(mp2, p0_s, p0_w), p0);
        e.q0 = BLEND(cond, BLEND(mq2, q0_s, q0_w), q0);
        e.p1 = BLEND(mp2, p1_s, p1);
        e.p2 = BLEND(mp2, p2_s, p2);
        e.q1 = BLEND(mq2, q1_s, q1);
        e.q2 = BLEND(mq2, q2_s, q2);
    }
    return e;
}

/* the six taps an intra edge can rewrite, in the ystride == 1 layout */
#define LF_TAPS6(b, xs)                                                         \
    p2 = U8((b) - 3 * (xs)), p1 = U8((b) - 2 * (xs)), p0 = U8((b) - (xs)),       \
    q0 = U8(b),                 q1 = U8(b + (xs)),   q2 = U8(b + 2 * (xs))

void h264_loop_filter_luma_intra_neon(uint8_t *pix, int xstride, int ystride,
                                      int inner_iters, int alpha, int beta)
{
    const int16x8_t av = vdupq_n_s16((short)alpha);
    const int16x8_t bv = vdupq_n_s16((short)beta);
    const int16x8_t aq = vdupq_n_s16((short)((alpha >> 2) + 2));
    const int total = 4 * inner_iters;
    int pos0;

    for (pos0 = 0; pos0 < total; pos0 += 8) {
        const int n = total - pos0 < 8 ? total - pos0 : 8;
        const int16x8_t on = lf_lanes_used(n);
        int16x8_t t[8], p3, p2, p1, p0, q0, q1, q2, q3;
        LF6 e;

        if (ystride == 1) {
            const uint8_t *b = pix + pos0;
            p3 = U8(b - 4 * xstride);
            q3 = U8(b + 3 * xstride);
            LF_TAPS6(b, xstride);
            e.p2 = p2; e.p1 = p1; e.p0 = p0; e.q0 = q0; e.q1 = q1; e.q2 = q2;
            e = lf_luma_intra(e, p3, q3, on, av, bv, aq);
            lf_store_n((uint8_t *)(b - 3 * xstride), lf_pack8(e.p2), n);
            lf_store_n((uint8_t *)(b - 2 * xstride), lf_pack8(e.p1), n);
            lf_store_n((uint8_t *)(b - xstride),     lf_pack8(e.p0), n);
            lf_store_n((uint8_t *)b,                 lf_pack8(e.q0), n);
            lf_store_n((uint8_t *)(b + xstride),     lf_pack8(e.q1), n);
            lf_store_n((uint8_t *)(b + 2 * xstride), lf_pack8(e.q2), n);
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

void h264_loop_filter_chroma_intra_neon(uint8_t *pix, int xstride, int ystride,
                                        int inner_iters, int alpha, int beta)
{
    const int16x8_t av = vdupq_n_s16((short)alpha);
    const int16x8_t bv = vdupq_n_s16((short)beta);
    const int total = 4 * inner_iters;
    int pos0;

    for (pos0 = 0; pos0 < total; pos0 += 8) {
        const int n = total - pos0 < 8 ? total - pos0 : 8;
        const int16x8_t on = lf_lanes_used(n);
        int16x8_t t[8];
        LF4 e;

        if (ystride == 1) {
            const uint8_t *b = pix + pos0;
            e.p1 = U8(b - 2 * xstride);
            e.p0 = U8(b - xstride);
            e.q0 = U8(b);
            e.q1 = U8(b + xstride);
            e = lf_chroma_intra(e, on, av, bv);
            lf_store_n((uint8_t *)(b - xstride), lf_pack8(e.p0), n);
            lf_store_n((uint8_t *)b,            lf_pack8(e.q0), n);
        } else {
            lf_load_rows(pix, ystride, pos0, n, t);
            e.p1 = t[2]; e.p0 = t[3]; e.q0 = t[4]; e.q1 = t[5];
            e = lf_chroma_intra(e, on, av, bv);
            t[3] = e.p0; t[4] = e.q0;
            lf_store_rows(pix, ystride, pos0, n, t);
        }
    }
}

#endif /* H264_SIMD_ARM */
