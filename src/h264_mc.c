/*
 * H.264 motion compensation (8-bit 4:2:0), weighted prediction and
 * edge emulation.  Algorithms from ITU-T H.264 and FFmpeg.
 */
#include "h264_internal.h"

static inline uint8_t clip_u8(int v)
{
    return (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
}

static inline int rnd_avg(int a, int b)
{
    return (a + b + 1) >> 1;
}

/* ------------------------------------------------------------------ */
/* Edge emulation                                                      */
/* ------------------------------------------------------------------ */
void h264_emulated_edge_mc(uint8_t *buf, const uint8_t *src, int stride,
                           int block_w, int block_h, int src_x, int src_y,
                           int pic_w, int pic_h)
{
    int x, y;
    int start_y, start_x, end_y, end_x;

    if (!pic_w || !pic_h)
        return;

    if (src_y >= pic_h) {
        src -= src_y * stride;
        src += (pic_h - 1) * stride;
        src_y = pic_h - 1;
    } else if (src_y <= -block_h) {
        src -= src_y * stride;
        src += (1 - block_h) * stride;
        src_y = 1 - block_h;
    }
    if (src_x >= pic_w) {
        src -= (1 + src_x - pic_w);
        src_x = pic_w - 1;
    } else if (src_x <= -block_w) {
        src += (1 - block_w - src_x);
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
        src += start_y * stride + start_x;
        buf += start_x;

        for (y = 0; y < start_y; y++) {
            memcpy(buf, src, w);
            buf += stride;
        }
        for (; y < end_y; y++) {
            memcpy(buf, src, w);
            src += stride;
            buf += stride;
        }
        src -= stride;
        for (; y < block_h; y++) {
            memcpy(buf, src, w);
            buf += stride;
        }
        buf -= block_h * stride + start_x;
        while (block_h--) {
            for (x = 0; x < start_x; x++)
                buf[x] = buf[start_x];
            for (x = end_x; x < block_w; x++)
                buf[x] = buf[end_x - 1];
            buf += stride;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Quarter-pel interpolation                                           */
/* ------------------------------------------------------------------ */
static inline int hp_h_raw(const uint8_t *p)
{
    return (p[0] + p[1]) * 20 - (p[-1] + p[2]) * 5 + (p[-2] + p[3]);
}

static inline int hp_v_raw(const uint8_t *p, int stride)
{
    return (p[0] + p[stride]) * 20 - (p[-stride] + p[2 * stride]) * 5 +
           (p[-2 * stride] + p[3 * stride]);
}

static inline int hp_h(const uint8_t *p)
{
    return clip_u8((hp_h_raw(p) + 16) >> 5);
}

static inline int hp_v(const uint8_t *p, int stride)
{
    return clip_u8((hp_v_raw(p, stride) + 16) >> 5);
}

static inline int hp_hv(const uint8_t *p, int stride)
{
    int bm2 = hp_h_raw(p - 2 * stride);
    int bm1 = hp_h_raw(p - 1 * stride);
    int b0  = hp_h_raw(p);
    int b1  = hp_h_raw(p + 1 * stride);
    int b2  = hp_h_raw(p + 2 * stride);
    int b3  = hp_h_raw(p + 3 * stride);
    int v = (b0 + b1) * 20 - (bm1 + b2) * 5 + (bm2 + b3);
    return clip_u8((v + 512) >> 10);
}

void h264_qpel_mc_c(uint8_t *dst, const uint8_t *src, int stride,
                    int w, int h, int x, int y, int avg)
{
    int i, j;
    if (x == 0 && y == 0) {
        for (j = 0; j < h; j++) {
            if (avg) {
                for (i = 0; i < w; i++)
                    dst[i] = (uint8_t)rnd_avg(dst[i], src[i]);
            } else {
                memcpy(dst, src, w);
            }
            dst += stride;
            src += stride;
        }
        return;
    }

    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i++) {
            const uint8_t *p = src + i;
            int v;
            switch (x + 4 * y) {
            case 1:  v = rnd_avg(p[0], hp_h(p)); break;
            case 2:  v = hp_h(p); break;
            case 3:  v = rnd_avg(p[1], hp_h(p)); break;
            case 4:  v = rnd_avg(p[0], hp_v(p, stride)); break;
            case 5:  v = rnd_avg(hp_h(p), hp_v(p, stride)); break;
            case 6:  v = rnd_avg(hp_h(p), hp_hv(p, stride)); break;
            case 7:  v = rnd_avg(hp_h(p), hp_v(p + 1, stride)); break;
            case 8:  v = hp_v(p, stride); break;
            case 9:  v = rnd_avg(hp_v(p, stride), hp_hv(p, stride)); break;
            case 10: v = hp_hv(p, stride); break;
            case 11: v = rnd_avg(hp_v(p + 1, stride), hp_hv(p, stride)); break;
            case 12: v = rnd_avg(p[stride], hp_v(p, stride)); break;
            case 13: v = rnd_avg(hp_h(p + stride), hp_v(p, stride)); break;
            case 14: v = rnd_avg(hp_h(p + stride), hp_hv(p, stride)); break;
            case 15: v = rnd_avg(hp_h(p + stride), hp_v(p + 1, stride)); break;
            default: v = p[0]; break;
            }
            dst[i] = avg ? (uint8_t)rnd_avg(dst[i], v) : (uint8_t)v;
        }
        dst += stride;
        src += stride;
    }
}

/* ------------------------------------------------------------------ */
/* Chroma interpolation (bilinear)                                     */
/* ------------------------------------------------------------------ */
void h264_chroma_mc_c(uint8_t *dst, const uint8_t *src, int stride,
                      int w, int h, int x, int y, int avg)
{
    const int A = (8 - x) * (8 - y);
    const int B = x * (8 - y);
    const int C = (8 - x) * y;
    const int D = x * y;
    int i, j;

    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i++) {
            int v = (A * src[i] + B * src[i + 1] +
                     C * src[stride + i] + D * src[stride + i + 1] + 32) >> 6;
            dst[i] = avg ? (uint8_t)rnd_avg(dst[i], v) : (uint8_t)v;
        }
        dst += stride;
        src += stride;
    }
}

/* ------------------------------------------------------------------ */
/* Dispatch: the SSE2 kernels produce bit-identical results to the     */
/* scalar ones (tests/dsp_check.c verifies that), so the choice is a    */
/* pure speed decision, taken at run time from the detected features.   */
/* ------------------------------------------------------------------ */
void h264_qpel_mc(uint8_t *dst, const uint8_t *src, int stride,
                  int w, int h, int x, int y, int avg)
{
#if defined(H264_SIMD)
    if (!x && !y && H264_HAVE_SIMD) {
        /* integer motion is most of the 16-wide calls in a typical stream, and
         * the kernels below only interpolate, so this is a separate kernel
         * rather than another case of them */
        h264_hpel_mc_simd(dst, src, stride, w, h, avg);
        return;
    }
#if defined(H264_SIMD_X86)
    if (h264_have_avx2 && w >= 16) {
        /* 16 lanes per vector only pay off when one vector spans the row. The
         * second half of the register file is an x86 extension, so NEON has no
         * counterpart here and the 8-lane kernel covers every width. */
        h264_qpel_mc_avx2(dst, src, stride, w, h, x, y, avg);
        return;
    }
#endif
    if (H264_HAVE_SIMD) {
        h264_qpel_mc_simd(dst, src, stride, w, h, x, y, avg);
        return;
    }
#endif
    h264_qpel_mc_c(dst, src, stride, w, h, x, y, avg);
}

void h264_chroma_mc(uint8_t *dst, const uint8_t *src, int stride,
                    int w, int h, int x, int y, int avg)
{
    int j;

    if (!x && !y) {
        /* At an integer position the bilinear weights are 64, 0, 0, 0 and the
         * sum is (64*p + 32) >> 6 = p, so the filter is a copy - and 93.4% of
         * the chroma pixels of a 720p High stream are filtered at an integer
         * position (13.99M of 15.57M calls on the bench clip), because a
         * chroma block only moves when its luma motion vector is not a whole
         * number of chroma samples. The kernels below spend four multiplies
         * and eight 16-bit unpacks on each of those pixels; the byte kernels
         * spend one load and one op.
         *
         * The SSE2 narrow-row path stores four bytes at a time, so a 2-wide
         * block (a 4x4 luma partition) keeps the scalar loop. */
#if defined(H264_SIMD)
        if (H264_HAVE_SIMD && w >= 4) {
            h264_hpel_mc_simd(dst, src, stride, w, h, avg);
            return;
        }
#endif
        for (j = 0; j < h; j++) {
            if (avg) {
                int i;
                for (i = 0; i < w; i++)
                    dst[i] = (uint8_t)rnd_avg(dst[i], src[i]);
            } else {
                memcpy(dst, src, w);
            }
            dst += stride;
            src += stride;
        }
        return;
    }

#if defined(H264_SIMD)
    /* The vector kernels work on 8 lanes. A 4-wide block can be served by them
     * - put8() stores only the bytes the block asked for, and the 32-pixel edge
     * padding covers the over-read - but it is worth nothing: the width
     * histogram of a 720p High stream says 14.9% of its 16.8M chroma MC calls
     * are 4-wide (10.0% of 12.0M on an interlaced clip), and running all of
     * those through the kernel measured 1.000x and 1.0045x, medians of seven
     * and five paired runs. Blocks this small are bound by the load and store
     * overhead either way, so the narrow widths stay scalar. */
    if (H264_HAVE_SIMD && w >= 8) {
        h264_chroma_mc_simd(dst, src, stride, w, h, x, y, avg);
        return;
    }
#endif
    h264_chroma_mc_c(dst, src, stride, w, h, x, y, avg);
}

/* ------------------------------------------------------------------ */
/* Weighted prediction                                                 */
/* ------------------------------------------------------------------ */
void h264_weight_pixels_c(uint8_t *dst, int stride, int w, int h,
                          int log2_denom, int weight, int offset)
{
    int i, j;
    offset = (unsigned)offset << log2_denom;
    if (log2_denom)
        offset += 1 << (log2_denom - 1);
    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i++)
            dst[i] = clip_u8((dst[i] * weight + offset) >> log2_denom);
        dst += stride;
    }
}

void h264_biweight_pixels_c(uint8_t *dst, const uint8_t *src, int stride,
                            int w, int h, int log2_denom, int weightd,
                            int weights, int offset)
{
    int i, j;
    offset = (unsigned)((offset + 1) | 1) << log2_denom;
    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i++)
            dst[i] = clip_u8((src[i] * weights + dst[i] * weightd + offset) >>
                             (log2_denom + 1));
        dst += stride;
        src += stride;
    }
}

/* the scaled offset the kernel has to keep inside int16 */
static inline int64_t wp_off1(int log2_denom, int offset)
{
    /* offset is negative for a fair share of weighted streams, and shifting a
     * negative value left is undefined behaviour rather than the wrap every
     * target here performs - UBSAN reported it once per stream that uses
     * weights. Multiplying gives the same value by definition and the compiler
     * emits the same shift. */
    return (int64_t)offset * ((int64_t)1 << log2_denom) +
           (log2_denom ? (int64_t)1 << (log2_denom - 1) : 0);
}

static inline int64_t wp_off2(int log2_denom, int offset)
{
    return (((int64_t)offset + 1) | 1) * ((int64_t)1 << log2_denom);
}

/* A single-list prediction is weighted on every macroblock that is copied, and
 * for a P stream 98% of those requests are the identity - the pass would read
 * and rewrite the whole macroblock to multiply by one. The histogram in README
 * is what sets these two branches. */
void h264_weight_pixels(uint8_t *dst, int stride, int w, int h,
                        int log2_denom, int weight, int offset)
{
#if defined(H264_SIMD)
    int64_t off = wp_off1(log2_denom, offset);

    if (weight == (1 << log2_denom) && offset == 0)
        return;
    if (H264_HAVE_SIMD && log2_denom >= 0 && log2_denom <= 7 &&
        weight >= -128 && weight <= 128 && off >= -32768 && off <= 32767) {
        h264_weight_pixels_simd(dst, stride, w, h, log2_denom, weight, offset);
        return;
    }
#endif
    h264_weight_pixels_c(dst, stride, w, h, log2_denom, weight, offset);
}

void h264_biweight_pixels(uint8_t *dst, const uint8_t *src, int stride,
                          int w, int h, int log2_denom, int weightd,
                          int weights, int offset)
{
#if defined(H264_SIMD)
    int64_t off = wp_off2(log2_denom, offset);

    if (weightd == weights && weightd == (1 << log2_denom) && offset == 0) {
        if (H264_HAVE_SIMD) {
            h264_avg_pixels_simd(dst, src, stride, w, h);
            return;
        }
    } else if (H264_HAVE_SIMD && log2_denom >= 0 && log2_denom <= 6 &&
               weightd >= -128 && weightd <= 128 &&
               weights >= -128 && weights <= 128 &&
               off >= -32768 && off <= 32767) {
        h264_biweight_pixels_simd(dst, src, stride, w, h, log2_denom, weightd,
                                  weights, offset);
        return;
    }
#endif
    h264_biweight_pixels_c(dst, src, stride, w, h, log2_denom, weightd,
                           weights, offset);
}
