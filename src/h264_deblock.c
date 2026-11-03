/*
 * H.264 deblocking filter (8-bit 4:2:0).
 * Ported from FFmpeg's h264_loopfilter.c / h264dsp_template.c (LGPL-2.1+).
 * Filtering is deferred until a picture is fully reconstructed, then applied
 * in macroblock address order (which matches the reference decoder).
 */
#include "h264_internal.h"

static const uint8_t alpha_table[52 * 3] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  4,  4,  5,  6,  7,  8,  9, 10, 12, 13,
   15, 17, 20, 22, 25, 28, 32, 36, 40, 45, 50, 56, 63,
   71, 80, 90,101,113,127,144,162,182,203,226,255,255,
  255,255,255,255,255,255,255,255,255,255,255,255,255,
  255,255,255,255,255,255,255,255,255,255,255,255,255,
  255,255,255,255,255,255,255,255,255,255,255,255,255,
  255,255,255,255,255,255,255,255,255,255,255,255,255,
};

static const uint8_t beta_table[52 * 3] = {
     0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
     0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
     0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
     0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
     0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
     0,  0,  0,  0,  0,  0,  2,  2,  2,  3,
     3,  3,  3,  4,  4,  4,  6,  6,  7,  7,
     8,  8,  9,  9, 10, 10, 11, 11, 12, 12,
    13, 13, 14, 14, 15, 15, 16, 16, 17, 17,
    18, 18,
    18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18,
    18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18,
    18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18,
    18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18,
};

static const uint8_t tc0_table[52 * 3][4] = {
    {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 },
    {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 },
    {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 },
    {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 },
    {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 },
    {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 },
    {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 },
    {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 },
    {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 },
    {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 },
    {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 },
    {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 1 },
    {-1, 0, 0, 1 }, {-1, 0, 0, 1 }, {-1, 0, 0, 1 }, {-1, 0, 1, 1 }, {-1, 0, 1, 1 }, {-1, 1, 1, 1 },
    {-1, 1, 1, 1 }, {-1, 1, 1, 1 }, {-1, 1, 1, 1 }, {-1, 1, 1, 2 }, {-1, 1, 1, 2 }, {-1, 1, 1, 2 },
    {-1, 1, 1, 2 }, {-1, 1, 2, 3 }, {-1, 1, 2, 3 }, {-1, 2, 2, 3 }, {-1, 2, 2, 4 }, {-1, 2, 3, 4 },
    {-1, 2, 3, 4 }, {-1, 3, 3, 5 }, {-1, 3, 4, 6 }, {-1, 3, 4, 6 }, {-1, 4, 5, 7 }, {-1, 4, 5, 8 },
    {-1, 4, 6, 9 }, {-1, 5, 7,10 }, {-1, 6, 8,11 }, {-1, 6, 8,13 }, {-1, 7,10,14 }, {-1, 8,11,16 },
    {-1, 9,12,18 }, {-1,10,13,20 }, {-1,11,15,23 }, {-1,13,17,25 },
    {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 },
    {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 },
    {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 },
    {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 },
    {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 },
    {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 },
    {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 },
    {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 },
    {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 },
};

/* ------------------------------------------------------------------ */
/* pixel filters                                                       */
/* ------------------------------------------------------------------ */
void h264_loop_filter_luma_c(uint8_t *pix, int xstride, int ystride,
                               int inner_iters, int alpha, int beta, const int8_t *tc0)
{
    int i, d;
    for (i = 0; i < 4; i++) {
        const int tc_orig = tc0[i];
        if (tc_orig < 0) {
            pix += inner_iters * ystride;
            continue;
        }
        for (d = 0; d < inner_iters; d++) {
            const int p0 = pix[-1 * xstride];
            const int p1 = pix[-2 * xstride];
            const int p2 = pix[-3 * xstride];
            const int q0 = pix[0];
            const int q1 = pix[1 * xstride];
            const int q2 = pix[2 * xstride];

            if (H264_ABS(p0 - q0) < alpha &&
                H264_ABS(p1 - p0) < beta &&
                H264_ABS(q1 - q0) < beta) {
                int tc = tc_orig;
                int i_delta;

                if (H264_ABS(p2 - p0) < beta) {
                    if (tc_orig)
                        pix[-2 * xstride] = (uint8_t)(p1 + H264_CLIP(((p2 + ((p0 + q0 + 1) >> 1)) >> 1) - p1, -tc_orig, tc_orig));
                    tc++;
                }
                if (H264_ABS(q2 - q0) < beta) {
                    if (tc_orig)
                        pix[xstride] = (uint8_t)(q1 + H264_CLIP(((q2 + ((p0 + q0 + 1) >> 1)) >> 1) - q1, -tc_orig, tc_orig));
                    tc++;
                }
                i_delta = H264_CLIP((((q0 - p0) * 4) + (p1 - q1) + 4) >> 3, -tc, tc);
                pix[-xstride] = H264_CLIP_PIXEL(p0 + i_delta);
                pix[0] = H264_CLIP_PIXEL(q0 - i_delta);
            }
            pix += ystride;
        }
    }
}

void h264_loop_filter_luma_intra_c(uint8_t *pix, int xstride, int ystride,
                                     int inner_iters, int alpha, int beta)
{
    int d;
    for (d = 0; d < 4 * inner_iters; d++) {
        const int p2 = pix[-3 * xstride];
        const int p1 = pix[-2 * xstride];
        const int p0 = pix[-1 * xstride];
        const int q0 = pix[0];
        const int q1 = pix[1 * xstride];
        const int q2 = pix[2 * xstride];

        if (H264_ABS(p0 - q0) < alpha &&
            H264_ABS(p1 - p0) < beta &&
            H264_ABS(q1 - q0) < beta) {
            if (H264_ABS(p0 - q0) < ((alpha >> 2) + 2)) {
                if (H264_ABS(p2 - p0) < beta) {
                    const int p3 = pix[-4 * xstride];
                    pix[-1 * xstride] = (uint8_t)((p2 + 2 * p1 + 2 * p0 + 2 * q0 + q1 + 4) >> 3);
                    pix[-2 * xstride] = (uint8_t)((p2 + p1 + p0 + q0 + 2) >> 2);
                    pix[-3 * xstride] = (uint8_t)((2 * p3 + 3 * p2 + p1 + p0 + q0 + 4) >> 3);
                } else {
                    pix[-1 * xstride] = (uint8_t)((2 * p1 + p0 + q1 + 2) >> 2);
                }
                if (H264_ABS(q2 - q0) < beta) {
                    const int q3 = pix[3 * xstride];
                    pix[0 * xstride] = (uint8_t)((p1 + 2 * p0 + 2 * q0 + 2 * q1 + q2 + 4) >> 3);
                    pix[1 * xstride] = (uint8_t)((p0 + q0 + q1 + q2 + 2) >> 2);
                    pix[2 * xstride] = (uint8_t)((2 * q3 + 3 * q2 + q1 + q0 + p0 + 4) >> 3);
                } else {
                    pix[0 * xstride] = (uint8_t)((2 * q1 + q0 + p1 + 2) >> 2);
                }
            } else {
                pix[-1 * xstride] = (uint8_t)((2 * p1 + p0 + q1 + 2) >> 2);
                pix[0 * xstride] = (uint8_t)((2 * q1 + q0 + p1 + 2) >> 2);
            }
        }
        pix += ystride;
    }
}

void h264_loop_filter_chroma_c(uint8_t *pix, int xstride, int ystride,
                                 int inner_iters, int alpha, int beta, const int8_t *tc0)
{
    int i, d;
    for (i = 0; i < 4; i++) {
        const int tc = tc0[i];
        if (tc <= 0) {
            pix += inner_iters * ystride;
            continue;
        }
        for (d = 0; d < inner_iters; d++) {
            const int p0 = pix[-1 * xstride];
            const int p1 = pix[-2 * xstride];
            const int q0 = pix[0];
            const int q1 = pix[1 * xstride];

            if (H264_ABS(p0 - q0) < alpha &&
                H264_ABS(p1 - p0) < beta &&
                H264_ABS(q1 - q0) < beta) {
                int delta = H264_CLIP(((q0 - p0) * 4 + (p1 - q1) + 4) >> 3, -tc, tc);
                pix[-xstride] = H264_CLIP_PIXEL(p0 + delta);
                pix[0] = H264_CLIP_PIXEL(q0 - delta);
            }
            pix += ystride;
        }
    }
}

void h264_loop_filter_chroma_intra_c(uint8_t *pix, int xstride, int ystride,
                                       int inner_iters, int alpha, int beta)
{
    int d;
    for (d = 0; d < 4 * inner_iters; d++) {
        const int p0 = pix[-1 * xstride];
        const int p1 = pix[-2 * xstride];
        const int q0 = pix[0];
        const int q1 = pix[1 * xstride];

        if (H264_ABS(p0 - q0) < alpha &&
            H264_ABS(p1 - p0) < beta &&
            H264_ABS(q1 - q0) < beta) {
            pix[-xstride] = (uint8_t)((2 * p1 + p0 + q1 + 2) >> 2);
            pix[0] = (uint8_t)((2 * q1 + q0 + p1 + 2) >> 2);
        }
        pix += ystride;
    }
}

/* ------------------------------------------------------------------ */
/* 10-bit pixel filters: same arithmetic, 16-bit samples, 0..1023 clip, */
/* thresholds scaled by 1 << (bit_depth-8), i.e. by four.              */
/* ------------------------------------------------------------------ */
static inline uint16_t clip10(int v)
{
    return (uint16_t)(v < 0 ? 0 : v > 1023 ? 1023 : v);
}

void h264_loop_filter_luma_16_c(uint8_t *pix8, int xstride, int ystride,
                                int inner_iters, int alpha, int beta,
                                const int8_t *tc0)
{
    uint16_t *pix = (uint16_t *)pix8;
    int xs = xstride >> 1, ys = ystride >> 1;
    int i, d;

    alpha <<= 2;
    beta <<= 2;
    for (i = 0; i < 4; i++) {
        const int tc_orig = tc0[i] * 4;
        if (tc_orig < 0) {
            pix += inner_iters * ys;
            continue;
        }
        for (d = 0; d < inner_iters; d++) {
            const int p0 = pix[-1 * xs];
            const int p1 = pix[-2 * xs];
            const int p2 = pix[-3 * xs];
            const int q0 = pix[0];
            const int q1 = pix[1 * xs];
            const int q2 = pix[2 * xs];

            if (H264_ABS(p0 - q0) < alpha &&
                H264_ABS(p1 - p0) < beta &&
                H264_ABS(q1 - q0) < beta) {
                int tc = tc_orig;
                int i_delta;

                if (H264_ABS(p2 - p0) < beta) {
                    if (tc_orig)
                        pix[-2 * xs] = (uint16_t)(p1 + H264_CLIP(((p2 + ((p0 + q0 + 1) >> 1)) >> 1) - p1, -tc_orig, tc_orig));
                    tc++;
                }
                if (H264_ABS(q2 - q0) < beta) {
                    if (tc_orig)
                        pix[xs] = (uint16_t)(q1 + H264_CLIP(((q2 + ((p0 + q0 + 1) >> 1)) >> 1) - q1, -tc_orig, tc_orig));
                    tc++;
                }
                i_delta = H264_CLIP((((q0 - p0) * 4) + (p1 - q1) + 4) >> 3, -tc, tc);
                pix[-xs] = clip10(p0 + i_delta);
                pix[0] = clip10(q0 - i_delta);
            }
            pix += ys;
        }
    }
}

void h264_loop_filter_luma_intra_16_c(uint8_t *pix8, int xstride, int ystride,
                                      int inner_iters, int alpha, int beta)
{
    uint16_t *pix = (uint16_t *)pix8;
    int xs = xstride >> 1, ys = ystride >> 1;
    int d;

    alpha <<= 2;
    beta <<= 2;
    for (d = 0; d < 4 * inner_iters; d++) {
        const int p2 = pix[-3 * xs];
        const int p1 = pix[-2 * xs];
        const int p0 = pix[-1 * xs];
        const int q0 = pix[0];
        const int q1 = pix[1 * xs];
        const int q2 = pix[2 * xs];

        if (H264_ABS(p0 - q0) < alpha &&
            H264_ABS(p1 - p0) < beta &&
            H264_ABS(q1 - q0) < beta) {
            if (H264_ABS(p0 - q0) < ((alpha >> 2) + 2)) {
                if (H264_ABS(p2 - p0) < beta) {
                    const int p3 = pix[-4 * xs];
                    pix[-1 * xs] = (uint16_t)((p2 + 2 * p1 + 2 * p0 + 2 * q0 + q1 + 4) >> 3);
                    pix[-2 * xs] = (uint16_t)((p2 + p1 + p0 + q0 + 2) >> 2);
                    pix[-3 * xs] = (uint16_t)((2 * p3 + 3 * p2 + p1 + p0 + q0 + 4) >> 3);
                } else {
                    pix[-1 * xs] = (uint16_t)((2 * p1 + p0 + q1 + 2) >> 2);
                }
                if (H264_ABS(q2 - q0) < beta) {
                    const int q3 = pix[3 * xs];
                    pix[0 * xs] = (uint16_t)((p1 + 2 * p0 + 2 * q0 + 2 * q1 + q2 + 4) >> 3);
                    pix[1 * xs] = (uint16_t)((p0 + q0 + q1 + q2 + 2) >> 2);
                    pix[2 * xs] = (uint16_t)((2 * q3 + 3 * q2 + q1 + q0 + p0 + 4) >> 3);
                } else {
                    pix[0 * xs] = (uint16_t)((2 * q1 + q0 + p1 + 2) >> 2);
                }
            } else {
                pix[-1 * xs] = (uint16_t)((2 * p1 + p0 + q1 + 2) >> 2);
                pix[0 * xs] = (uint16_t)((2 * q1 + q0 + p1 + 2) >> 2);
            }
        }
        pix += ys;
    }
}

void h264_loop_filter_chroma_16_c(uint8_t *pix8, int xstride, int ystride,
                                  int inner_iters, int alpha, int beta,
                                  const int8_t *tc0)
{
    uint16_t *pix = (uint16_t *)pix8;
    int xs = xstride >> 1, ys = ystride >> 1;
    int i, d;

    alpha <<= 2;
    beta <<= 2;
    for (i = 0; i < 4; i++) {
        int tc;
        if (tc0[i] <= 0) {
            pix += inner_iters * ys;
            continue;
        }
        /* computed after the check: tc0 is -1 for bS 0, and shifting that
         * left would be undefined before the group is skipped */
        tc = ((tc0[i] - 1) << 2) + 1;
        for (d = 0; d < inner_iters; d++) {
            const int p0 = pix[-1 * xs];
            const int p1 = pix[-2 * xs];
            const int q0 = pix[0];
            const int q1 = pix[1 * xs];

            if (H264_ABS(p0 - q0) < alpha &&
                H264_ABS(p1 - p0) < beta &&
                H264_ABS(q1 - q0) < beta) {
                int delta = H264_CLIP(((q0 - p0) * 4 + (p1 - q1) + 4) >> 3, -tc, tc);
                pix[-xs] = clip10(p0 + delta);
                pix[0] = clip10(q0 - delta);
            }
            pix += ys;
        }
    }
}

void h264_loop_filter_chroma_intra_16_c(uint8_t *pix8, int xstride, int ystride,
                                        int inner_iters, int alpha, int beta)
{
    uint16_t *pix = (uint16_t *)pix8;
    int xs = xstride >> 1, ys = ystride >> 1;
    int d;

    alpha <<= 2;
    beta <<= 2;
    for (d = 0; d < 4 * inner_iters; d++) {
        const int p0 = pix[-1 * xs];
        const int p1 = pix[-2 * xs];
        const int q0 = pix[0];
        const int q1 = pix[1 * xs];

        if (H264_ABS(p0 - q0) < alpha &&
            H264_ABS(p1 - p0) < beta &&
            H264_ABS(q1 - q0) < beta) {
            pix[-xs] = (uint16_t)((2 * p1 + p0 + q1 + 2) >> 2);
            pix[0] = (uint16_t)((2 * q1 + q0 + p1 + 2) >> 2);
        }
        pix += ys;
    }
}

/* ------------------------------------------------------------------ */
/* Dispatch: the SSE2 kernels give bit-identical results to the scalar ones
 * (tests/dsp_check.c compares them position by position), so the choice is a
 * pure speed decision, taken at run time from the detected features. */
static void loop_filter_luma(uint8_t *pix, int xstride, int ystride, int inner_iters,
                             int alpha, int beta, const int8_t *tc0)
{
#if defined(H264_SIMD_X86)
    /* the sixteen positions of a luma edge fit one AVX2 register, and all of
     * them are contiguous when ystride is 1; the transposed direction keeps
     * the SSE2 kernel, whose transpose is already the expensive part */
    if (h264_have_avx2 && ystride == 1 && inner_iters == 4) {
        h264_loop_filter_luma_avx2(pix, xstride, ystride, inner_iters,
                                   alpha, beta, tc0);
        return;
    }
#endif
#if defined(H264_SIMD)
    if (H264_HAVE_SIMD) {
        h264_loop_filter_luma_simd(pix, xstride, ystride, inner_iters, alpha, beta, tc0);
        return;
    }
#endif
    h264_loop_filter_luma_c(pix, xstride, ystride, inner_iters, alpha, beta, tc0);
}

static void loop_filter_chroma(uint8_t *pix, int xstride, int ystride, int inner_iters,
                               int alpha, int beta, const int8_t *tc0)
{
#if defined(H264_SIMD)
    /* Chroma edges are eight positions wide. The strided layout is 1.65x the
     * scalar filter in tests/dsp_bench.c. The transposed layout used to be
     * 0.98x, but the SSE2 file now has a four-tap transpose for the
     * eight-position shape every 8-bit 4:2:0 edge takes; the NEON kernel still
     * carries the generic 8x8 one, so that direction stays scalar there. */
    {
        int use_simd = xstride != 1;
#if defined(H264_SIMD_X86)
        use_simd = use_simd || inner_iters == 2;
#endif
        if (H264_HAVE_SIMD && use_simd) {
            h264_loop_filter_chroma_simd(pix, xstride, ystride, inner_iters, alpha, beta, tc0);
            return;
        }
    }
#endif
    h264_loop_filter_chroma_c(pix, xstride, ystride, inner_iters, alpha, beta, tc0);
}

static void loop_filter_luma_intra(uint8_t *pix, int xstride, int ystride, int inner_iters,
                                  int alpha, int beta)
{
#if defined(H264_SIMD)
    if (H264_HAVE_SIMD) {
        h264_loop_filter_luma_intra_simd(pix, xstride, ystride, inner_iters, alpha, beta);
        return;
    }
#endif
    h264_loop_filter_luma_intra_c(pix, xstride, ystride, inner_iters, alpha, beta);
}

static void loop_filter_chroma_intra(uint8_t *pix, int xstride, int ystride, int inner_iters,
                                     int alpha, int beta)
{
#if defined(H264_SIMD)
    /* same measurement as loop_filter_chroma: 2.15x strided, 0.78x transposed */
    if (H264_HAVE_SIMD && xstride != 1) {
        h264_loop_filter_chroma_intra_simd(pix, xstride, ystride, inner_iters, alpha, beta);
        return;
    }
#endif
    h264_loop_filter_chroma_intra_c(pix, xstride, ystride, inner_iters, alpha, beta);
}

static void v_luma(uint8_t *pix, int stride, int alpha, int beta, const int8_t *tc)
{
    loop_filter_luma(pix, stride, 1, 4, alpha, beta, tc);
}
static void h_luma(uint8_t *pix, int stride, int alpha, int beta, const int8_t *tc)
{
    loop_filter_luma(pix, 1, stride, 4, alpha, beta, tc);
}
static void h_luma_mbaff(uint8_t *pix, int stride, int alpha, int beta, const int8_t *tc)
{
    loop_filter_luma(pix, 1, stride, 2, alpha, beta, tc);
}
static void v_luma_intra(uint8_t *pix, int stride, int alpha, int beta)
{
    loop_filter_luma_intra(pix, stride, 1, 4, alpha, beta);
}
static void h_luma_intra(uint8_t *pix, int stride, int alpha, int beta)
{
    loop_filter_luma_intra(pix, 1, stride, 4, alpha, beta);
}
static void h_luma_mbaff_intra(uint8_t *pix, int stride, int alpha, int beta)
{
    loop_filter_luma_intra(pix, 1, stride, 2, alpha, beta);
}
static void v_chroma(uint8_t *pix, int stride, int alpha, int beta, const int8_t *tc)
{
    loop_filter_chroma(pix, stride, 1, 2, alpha, beta, tc);
}
static void h_chroma(uint8_t *pix, int stride, int alpha, int beta, const int8_t *tc)
{
    loop_filter_chroma(pix, 1, stride, 2, alpha, beta, tc);
}
static void h_chroma_mbaff(uint8_t *pix, int stride, int alpha, int beta, const int8_t *tc)
{
    loop_filter_chroma(pix, 1, stride, 1, alpha, beta, tc);
}
static void v_chroma_intra(uint8_t *pix, int stride, int alpha, int beta)
{
    loop_filter_chroma_intra(pix, stride, 1, 2, alpha, beta);
}
static void h_chroma_intra(uint8_t *pix, int stride, int alpha, int beta)
{
    loop_filter_chroma_intra(pix, 1, stride, 2, alpha, beta);
}
static void h_chroma_mbaff_intra(uint8_t *pix, int stride, int alpha, int beta)
{
    loop_filter_chroma_intra(pix, 1, stride, 1, alpha, beta);
}

/* 10-bit scalar counterparts; no SIMD path yet. */
static void v_luma_16(uint8_t *pix, int stride, int alpha, int beta, const int8_t *tc)
{
    h264_loop_filter_luma_16_c(pix, stride, 2, 4, alpha, beta, tc);
}
static void h_luma_16(uint8_t *pix, int stride, int alpha, int beta, const int8_t *tc)
{
    h264_loop_filter_luma_16_c(pix, 2, stride, 4, alpha, beta, tc);
}
static void h_luma_mbaff_16(uint8_t *pix, int stride, int alpha, int beta, const int8_t *tc)
{
    h264_loop_filter_luma_16_c(pix, 2, stride, 2, alpha, beta, tc);
}
static void v_luma_intra_16(uint8_t *pix, int stride, int alpha, int beta)
{
    h264_loop_filter_luma_intra_16_c(pix, stride, 2, 4, alpha, beta);
}
static void h_luma_intra_16(uint8_t *pix, int stride, int alpha, int beta)
{
    h264_loop_filter_luma_intra_16_c(pix, 2, stride, 4, alpha, beta);
}
static void h_luma_mbaff_intra_16(uint8_t *pix, int stride, int alpha, int beta)
{
    h264_loop_filter_luma_intra_16_c(pix, 2, stride, 2, alpha, beta);
}
static void h_chroma_422_16(uint8_t *pix, int stride, int alpha, int beta, const int8_t *tc)
{
    h264_loop_filter_chroma_16_c(pix, 2, stride, 4, alpha, beta, tc);
}

static void h_chroma_422_intra_16(uint8_t *pix, int stride, int alpha, int beta)
{
    h264_loop_filter_chroma_intra_16_c(pix, 2, stride, 4, alpha, beta);
}

static void h_chroma_mbaff_422_16(uint8_t *pix, int stride, int alpha, int beta, const int8_t *tc)
{
    h264_loop_filter_chroma_16_c(pix, 2, stride, 2, alpha, beta, tc);
}

static void h_chroma_mbaff_422_intra_16(uint8_t *pix, int stride, int alpha, int beta)
{
    h264_loop_filter_chroma_intra_16_c(pix, 2, stride, 2, alpha, beta);
}

static void v_chroma_16(uint8_t *pix, int stride, int alpha, int beta, const int8_t *tc)
{
    h264_loop_filter_chroma_16_c(pix, stride, 2, 2, alpha, beta, tc);
}
static void h_chroma_16(uint8_t *pix, int stride, int alpha, int beta, const int8_t *tc)
{
    h264_loop_filter_chroma_16_c(pix, 2, stride, 2, alpha, beta, tc);
}
static void h_chroma_mbaff_16(uint8_t *pix, int stride, int alpha, int beta, const int8_t *tc)
{
    h264_loop_filter_chroma_16_c(pix, 2, stride, 1, alpha, beta, tc);
}
static void v_chroma_intra_16(uint8_t *pix, int stride, int alpha, int beta)
{
    h264_loop_filter_chroma_intra_16_c(pix, stride, 2, 2, alpha, beta);
}
static void h_chroma_intra_16(uint8_t *pix, int stride, int alpha, int beta)
{
    h264_loop_filter_chroma_intra_16_c(pix, 2, stride, 2, alpha, beta);
}
static void h_chroma_mbaff_intra_16(uint8_t *pix, int stride, int alpha, int beta)
{
    h264_loop_filter_chroma_intra_16_c(pix, 2, stride, 1, alpha, beta);
}

/* ------------------------------------------------------------------ */
/* edge helpers                                                        */
/* ------------------------------------------------------------------ */
static void filter_mb_edgev(uint8_t *pix, int stride, const int bS[4],
                            unsigned qp, int a, int b, int intra, int high)
{
    const unsigned index_a = qp + a;
    const int alpha = alpha_table[index_a];
    const int beta = beta_table[qp + b];
    if (alpha == 0 || beta == 0) return;
    if (bS[0] < 4 || !intra) {
        int8_t tc[4];
        tc[0] = (int8_t)tc0_table[index_a][bS[0]];
        tc[1] = (int8_t)tc0_table[index_a][bS[1]];
        tc[2] = (int8_t)tc0_table[index_a][bS[2]];
        tc[3] = (int8_t)tc0_table[index_a][bS[3]];
        if (high) h_luma_16(pix, stride, alpha, beta, tc);
        else h_luma(pix, stride, alpha, beta, tc);
    } else {
        if (high) h_luma_intra_16(pix, stride, alpha, beta);
        else h_luma_intra(pix, stride, alpha, beta);
    }
}

static void filter_mb_edgecv(uint8_t *pix, int stride, const int bS[4],
                             unsigned qp, int a, int b, int intra, int high,
                             int c422)
{
    const unsigned index_a = qp + a;
    const int alpha = alpha_table[index_a];
    const int beta = beta_table[qp + b];
    if (alpha == 0 || beta == 0) return;
    if (bS[0] < 4 || !intra) {
        int8_t tc[4];
        tc[0] = (int8_t)(tc0_table[index_a][bS[0]] + 1);
        tc[1] = (int8_t)(tc0_table[index_a][bS[1]] + 1);
        tc[2] = (int8_t)(tc0_table[index_a][bS[2]] + 1);
        tc[3] = (int8_t)(tc0_table[index_a][bS[3]] + 1);
        if (high) {
            if (c422) h_chroma_422_16(pix, stride, alpha, beta, tc);
            else h_chroma_16(pix, stride, alpha, beta, tc);
        } else {
            h_chroma(pix, stride, alpha, beta, tc);
        }
    } else {
        if (high) {
            if (c422) h_chroma_422_intra_16(pix, stride, alpha, beta);
            else h_chroma_intra_16(pix, stride, alpha, beta);
        } else {
            h_chroma_intra(pix, stride, alpha, beta);
        }
    }
}

static void filter_mb_edgeh(uint8_t *pix, int stride, const int bS[4],
                            unsigned qp, int a, int b, int intra, int high)
{
    const unsigned index_a = qp + a;
    const int alpha = alpha_table[index_a];
    const int beta = beta_table[qp + b];
    if (alpha == 0 || beta == 0) return;
    if (bS[0] < 4 || !intra) {
        int8_t tc[4];
        tc[0] = (int8_t)tc0_table[index_a][bS[0]];
        tc[1] = (int8_t)tc0_table[index_a][bS[1]];
        tc[2] = (int8_t)tc0_table[index_a][bS[2]];
        tc[3] = (int8_t)tc0_table[index_a][bS[3]];
        if (high) v_luma_16(pix, stride, alpha, beta, tc);
        else v_luma(pix, stride, alpha, beta, tc);
    } else {
        if (high) v_luma_intra_16(pix, stride, alpha, beta);
        else v_luma_intra(pix, stride, alpha, beta);
    }
}

static void filter_mb_edgech(uint8_t *pix, int stride, const int bS[4],
                             unsigned qp, int a, int b, int intra, int high)
{
    const unsigned index_a = qp + a;
    const int alpha = alpha_table[index_a];
    const int beta = beta_table[qp + b];
    if (alpha == 0 || beta == 0) return;
    if (bS[0] < 4 || !intra) {
        int8_t tc[4];
        tc[0] = (int8_t)(tc0_table[index_a][bS[0]] + 1);
        tc[1] = (int8_t)(tc0_table[index_a][bS[1]] + 1);
        tc[2] = (int8_t)(tc0_table[index_a][bS[2]] + 1);
        tc[3] = (int8_t)(tc0_table[index_a][bS[3]] + 1);
        if (high) v_chroma_16(pix, stride, alpha, beta, tc);
        else v_chroma(pix, stride, alpha, beta, tc);
    } else {
        if (high) v_chroma_intra_16(pix, stride, alpha, beta);
        else v_chroma_intra(pix, stride, alpha, beta);
    }
}

static void filter_mb_mbaff_edgev(uint8_t *pix, int stride, const int bS[], int bsi,
                                  int qp, int a, int b, int intra, int high)
{
    const unsigned index_a = qp + a;
    const int alpha = alpha_table[index_a];
    const int beta = beta_table[qp + b];
    if (alpha == 0 || beta == 0) return;
    if (bS[0] < 4 || !intra) {
        int8_t tc[4];
        tc[0] = (int8_t)tc0_table[index_a][bS[0 * bsi]];
        tc[1] = (int8_t)tc0_table[index_a][bS[1 * bsi]];
        tc[2] = (int8_t)tc0_table[index_a][bS[2 * bsi]];
        tc[3] = (int8_t)tc0_table[index_a][bS[3 * bsi]];
        if (high) h_luma_mbaff_16(pix, stride, alpha, beta, tc);
        else h_luma_mbaff(pix, stride, alpha, beta, tc);
    } else {
        if (high) h_luma_mbaff_intra_16(pix, stride, alpha, beta);
        else h_luma_mbaff_intra(pix, stride, alpha, beta);
    }
}

static void filter_mb_mbaff_edgecv(uint8_t *pix, int stride, const int bS[], int bsi,
                                   int qp, int a, int b, int intra, int high,
                                   int c422)
{
    const unsigned index_a = qp + a;
    const int alpha = alpha_table[index_a];
    const int beta = beta_table[qp + b];
    if (alpha == 0 || beta == 0) return;
    if (bS[0] < 4 || !intra) {
        int8_t tc[4];
        tc[0] = (int8_t)(tc0_table[index_a][bS[0 * bsi]] + 1);
        tc[1] = (int8_t)(tc0_table[index_a][bS[1 * bsi]] + 1);
        tc[2] = (int8_t)(tc0_table[index_a][bS[2 * bsi]] + 1);
        tc[3] = (int8_t)(tc0_table[index_a][bS[3 * bsi]] + 1);
        if (high) {
            if (c422) h_chroma_mbaff_422_16(pix, stride, alpha, beta, tc);
            else h_chroma_mbaff_16(pix, stride, alpha, beta, tc);
        } else {
            h_chroma_mbaff(pix, stride, alpha, beta, tc);
        }
    } else {
        if (high) {
            if (c422) h_chroma_mbaff_422_intra_16(pix, stride, alpha, beta);
            else h_chroma_mbaff_intra_16(pix, stride, alpha, beta);
        } else {
            h_chroma_mbaff_intra(pix, stride, alpha, beta);
        }
    }
}

/* ------------------------------------------------------------------ */
/* filter caches (based on FFmpeg's fill_filter_caches)                */
/* ------------------------------------------------------------------ */
static void fill_filter_caches_inter(H264Decoder *h, H264Slice *sl, H264Frame *f,
                                     uint32_t mb_type, int top_xy,
                                     const int left_xy[2], int top_type,
                                     const int left_type[2], int mb_xy, int list)
{
    const int b_stride = h->b_stride;
    int16_t (*mv_dst)[2] = &sl->mv_cache[list][h264_scan8[0]];
    int8_t *ref_cache = &sl->ref_cache[list][h264_scan8[0]];
    int i;
    const int mb_field = MB_FIELD(sl);

    if (IS_INTER(mb_type) || IS_DIRECT(mb_type)) {
        if (USES_LIST(top_type, list)) {
            const int b_xy = h->mb2b_xy[top_xy] + 3 * b_stride;
            const int b8_xy = 4 * top_xy + 2;
            const int sl_id = f->slice_table[top_xy];
            const int *ref2frm = f->ref2frm[sl_id & (H264_MAX_SLICES - 1)][list];
            const int off = mb_field ? 20 : 2;
            memcpy(mv_dst - 1 * 8, f->motion_val[list][b_xy + 0], 16);
            ref_cache[0 - 1 * 8] = (int8_t)ref2frm[off + f->ref_index[list][b8_xy + 0]];
            ref_cache[1 - 1 * 8] = (int8_t)ref2frm[off + f->ref_index[list][b8_xy + 0]];
            ref_cache[2 - 1 * 8] = (int8_t)ref2frm[off + f->ref_index[list][b8_xy + 1]];
            ref_cache[3 - 1 * 8] = (int8_t)ref2frm[off + f->ref_index[list][b8_xy + 1]];
        } else {
            memset(mv_dst - 8, 0, 32);
            memset(ref_cache - 8, (uint8_t)LIST_NOT_USED, 4);
        }

        if (!IS_INTERLACED(mb_type ^ left_type[0])) {
            if (USES_LIST(left_type[0], list)) {
                const int b_xy = h->mb2b_xy[left_xy[0]] + 3;
                const int b8_xy = 4 * left_xy[0] + 1;
                const int sl_id = f->slice_table[left_xy[0]];
                const int *ref2frm = f->ref2frm[sl_id & (H264_MAX_SLICES - 1)][list];
                const int off = mb_field ? 20 : 2;
                memcpy(mv_dst - 1 + 0, f->motion_val[list][b_xy + b_stride * 0], 4);
                memcpy(mv_dst - 1 + 8, f->motion_val[list][b_xy + b_stride * 1], 4);
                memcpy(mv_dst - 1 + 16, f->motion_val[list][b_xy + b_stride * 2], 4);
                memcpy(mv_dst - 1 + 24, f->motion_val[list][b_xy + b_stride * 3], 4);
                ref_cache[-1 + 0] = (int8_t)ref2frm[off + f->ref_index[list][b8_xy + 0]];
                ref_cache[-1 + 8] = (int8_t)ref2frm[off + f->ref_index[list][b8_xy + 0]];
                ref_cache[-1 + 16] = (int8_t)ref2frm[off + f->ref_index[list][b8_xy + 2]];
                ref_cache[-1 + 24] = (int8_t)ref2frm[off + f->ref_index[list][b8_xy + 2]];
            } else {
                memset(mv_dst - 1 + 0, 0, 4);
                memset(mv_dst - 1 + 8, 0, 4);
                memset(mv_dst - 1 + 16, 0, 4);
                memset(mv_dst - 1 + 24, 0, 4);
                ref_cache[-1 + 0] = ref_cache[-1 + 8] =
                ref_cache[-1 + 16] = ref_cache[-1 + 24] = LIST_NOT_USED;
            }
        }
    }

    if (!USES_LIST(mb_type, list)) {
        for (i = 0; i < 4; i++) {
            memset(mv_dst + 8 * i, 0, 16);
            memset(ref_cache + 8 * i, (uint8_t)LIST_NOT_USED, 4);
        }
        return;
    }

    {
        const int8_t *ref = &f->ref_index[list][4 * mb_xy];
        const int sl_id = f->slice_table[mb_xy];
        const int *ref2frm = f->ref2frm[sl_id & (H264_MAX_SLICES - 1)][list];
        const int off = mb_field ? 20 : 2;
        /* each cache row holds the two 8x8 refs of that row, duplicated over
         * the four 4x4 slots: [ref0 ref0 ref1 ref1] */
        uint32_t w01 = (((uint32_t)ref2frm[off + ref[0]] & 0xff) |
                        ((uint32_t)ref2frm[off + ref[1]] & 0xff) * 0x10000) * 0x0101u;
        uint32_t w23 = (((uint32_t)ref2frm[off + ref[2]] & 0xff) |
                        ((uint32_t)ref2frm[off + ref[3]] & 0xff) * 0x10000) * 0x0101u;
        memcpy(&ref_cache[0 * 8], &w01, 4);
        memcpy(&ref_cache[1 * 8], &w01, 4);
        memcpy(&ref_cache[2 * 8], &w23, 4);
        memcpy(&ref_cache[3 * 8], &w23, 4);
    }

    {
        int16_t (*mv_src)[2] = &f->motion_val[list][4 * sl->mb_x + 4 * sl->mb_y * b_stride];
        memcpy(mv_dst + 8 * 0, mv_src + 0 * b_stride, 16);
        memcpy(mv_dst + 8 * 1, mv_src + 1 * b_stride, 16);
        memcpy(mv_dst + 8 * 2, mv_src + 2 * b_stride, 16);
        memcpy(mv_dst + 8 * 3, mv_src + 3 * b_stride, 16);
    }
}

/* A macroblock whose filtering cannot move a single sample is skipped whole.
 *
 * The reference does this with a hardcoded QP of 15 and only the pair averages.
 * The number is derivable, so this does it from the tables instead: a luma edge
 * clips its delta to +-tc0[index][bS] and a chroma edge only runs at all when
 * alpha[index_c] is nonzero, so while every index the macroblock and the
 * neighbours it owns an edge with can reach stays inside the run where tc0 is
 * zero for all four bS *and* alpha is zero, nothing here can change. Nor can
 * anything change on the edges this macroblock does not own: the macroblock on
 * the other side of one of those checks the same two QPs from its own side, so
 * either it filters the edge or the edge is inert for it too.
 *
 * LF_FILTER_INERT_INDEX is the last index of that joint run - alpha is zero up to
 * 15, tc0 for every bS up to 16 - and tests/dsp_check.c checks both against the
 * tables, so editing one of them without changing this number turns the suite red. */
#define LF_FILTER_INERT_INDEX 15

/* Checked by tests/dsp_check.c against the tables themselves, so the constant
 * above cannot quietly stop being true: 1 for an index inside the run where the
 * joint condition fails, 2 for one beyond it where the condition still holds. */
static int lf_index_inert(int i)
{
    return !alpha_table[52 + i] && !tc0_table[52 + i][1] &&
           !tc0_table[52 + i][2] && !tc0_table[52 + i][3];
}

int h264_deblock_check_inert_index(void)
{
    int i;

    for (i = 0; i <= LF_FILTER_INERT_INDEX; i++)
        if (!lf_index_inert(i))
            return 1;
    if (lf_index_inert(LF_FILTER_INERT_INDEX + 1))
        return 2;
    return 0;
}

/* A 16x16 inter macroblock with no luma residual only ever filters its left
 * and top edges: mask_edge is 3, so filter_mb_dir sets edges to 1, and the
 * mask_par0 shortcut derives all four lanes of each edge from the single
 * corner 4x4 block the MV prediction uses. If that block's reference and
 * motion vector equal the neighbour's adjacent block, and the luma nnz on the
 * edge is zero, both edges - and the chroma edges, which reuse the luma bS -
 * have boundary strength zero. filter_mb_dir would then fill the filter
 * caches, walk both directions and filter nothing. Prove that case from the
 * frame metadata before the caches are built. The neighbour partition modes
 * are restricted to the ones the mask_par0 shortcut covers; other modes
 * compute per-lane bS and fall through to the normal path. */
static int filter_mb_uniform_mv(H264Decoder *h, H264Slice *sl, H264Frame *f,
                                uint32_t mb_type, int mb_xy, int top_xy,
                                const int left_xy[2], int top_type,
                                const int left_type[2])
{
    const int b_stride = f->b_stride;
    const int base = h->mb2b_xy[mb_xy];
    int list;

    if (f->mbaff || f->field_picture)
        return 0;
    if (!IS_16X16(mb_type) || IS_INTRA(mb_type))
        return 0;
    if (f->cbp_table[mb_xy] & 15)
        return 0;
    if (sl->list_count == 2 &&
        (!USES_LIST(mb_type, 0) || !USES_LIST(mb_type, 1)))
        return 0;
    {
        const uint8_t *n = f->nnz[mb_xy];
        if (n[0] | n[1] | n[2] | n[3] | n[4] | n[5] | n[6] | n[7] |
            n[8] | n[9] | n[10] | n[11] | n[12] | n[13] | n[14] | n[15])
            return 0;
    }
    if (left_type[0]) {
        const uint8_t *n = f->nnz[left_xy[0]];
        if (n[3] | n[7] | n[11] | n[15])
            return 0;
    }
    if (top_type) {
        const uint8_t *n = f->nnz[top_xy];
        if (n[12] | n[13] | n[14] | n[15])
            return 0;
    }

    for (list = 0; list < sl->list_count; list++) {
        const int16_t (*cur)[2];
        const int8_t *ref = f->ref_index[list];
        const int cur_sl = f->slice_table[mb_xy] & (H264_MAX_SLICES - 1);
        int16_t mv0[2];
        int cur_ref;

        if (!USES_LIST(mb_type, list))
            continue;
        cur = &f->motion_val[list][base];
        mv0[0] = cur[0][0];
        mv0[1] = cur[0][1];
        cur_ref = ref[4 * mb_xy];

        if (left_type[0]) {
            const int16_t *lmv;
            const int lsl = f->slice_table[left_xy[0]] & (H264_MAX_SLICES - 1);
            const int lref = ref[4 * left_xy[0] + 1];
            if (IS_INTRA(left_type[0]) || !USES_LIST(left_type[0], list))
                return 0;
            if (!(left_type[0] & (MB_TYPE_16x16 | MB_TYPE_8x16)))
                return 0;
            if (!f->cabac && IS_8x8DCT(left_type[0]))
                return 0;
            if (lsl == cur_sl ? lref != cur_ref :
                f->ref2frm[lsl][list][2 + lref] !=
                    f->ref2frm[cur_sl][list][2 + cur_ref])
                return 0;
            lmv = f->motion_val[list][h->mb2b_xy[left_xy[0]] + 3];
            if (lmv[0] != mv0[0] || lmv[1] != mv0[1])
                return 0;
        }
        if (top_type) {
            const int16_t *tmv;
            const int tsl = f->slice_table[top_xy] & (H264_MAX_SLICES - 1);
            const int tref = ref[4 * top_xy + 2];
            if (IS_INTRA(top_type) || !USES_LIST(top_type, list))
                return 0;
            if (!(top_type & (MB_TYPE_16x16 | MB_TYPE_16x8)))
                return 0;
            if (!f->cabac && IS_8x8DCT(top_type))
                return 0;
            if (tsl == cur_sl ? tref != cur_ref :
                f->ref2frm[tsl][list][2 + tref] !=
                    f->ref2frm[cur_sl][list][2 + cur_ref])
                return 0;
            tmv = f->motion_val[list][h->mb2b_xy[top_xy] + 3 * b_stride];
            if (tmv[0] != mv0[0] || tmv[1] != mv0[1])
                return 0;
        }
    }
    return 1;
}

static int filter_mb_inert(H264Slice *sl, H264Frame *f,
                           int mb_xy, int top_xy, const int left_xy[2])
{
    const int a = sl->slice_alpha_c0_offset;
    int q[4];

    /* Ordered for the case that costs: almost every macroblock in a normal stream
     * answers on its own QP with one load, one add and one compare.
     *
     * No clamping is needed to ask the question: an index pushed below 0 lands on
     * table entry 0, which is inside the inert run, and one pushed above 51 fails
     * the compare anyway. */
    q[0] = f->qscale_table[mb_xy];
    if (q[0] + a > LF_FILTER_INERT_INDEX)
        return 0;
    q[1] = f->qscale_table[top_xy];
    q[2] = f->qscale_table[left_xy[0]];
    q[3] = f->qscale_table[left_xy[1]];
    if (q[1] + a > LF_FILTER_INERT_INDEX || q[2] + a > LF_FILTER_INERT_INDEX ||
        q[3] + a > LF_FILTER_INERT_INDEX)
        return 0;
    /* the chroma edges address the same two tables through the picture's chroma
     * mapping, as filter_mb_edgecv and filter_mb_edgech do */
    {
        int i;
        for (i = 0; i < 4; i++) {
            if (f->chroma_qp_table[0][q[i]] + a > LF_FILTER_INERT_INDEX ||
                f->chroma_qp_table[1][q[i]] + a > LF_FILTER_INERT_INDEX)
                return 0;
        }
    }
    return 1;
}

static int fill_filter_caches(H264Decoder *h, H264Slice *sl, H264Frame *f, uint32_t mb_type)
{
    const int mb_xy = sl->mb_xy;
    int top_xy, left_xy[2];
    int top_type, left_type[2];

    top_xy = mb_xy - (h->mb_stride << MB_FIELD(sl));
    left_xy[0] = left_xy[1] = mb_xy - 1;

    if (f->mbaff) {
        const int left_mb_field_flag = mb_xy > 0 ? (int)IS_INTERLACED(f->mb_type[mb_xy - 1]) : 0;
        const int curr_mb_field_flag = (int)IS_INTERLACED(mb_type);
        if (sl->mb_y & 1) {
            if (left_mb_field_flag != curr_mb_field_flag)
                left_xy[0] -= h->mb_stride;
        } else {
            if (curr_mb_field_flag)
                top_xy += h->mb_stride &
                          (((f->mb_type[top_xy] >> 7) & 1) - 1);
            if (left_mb_field_flag != curr_mb_field_flag)
                left_xy[1] += h->mb_stride;
        }
    }

    sl->top_mb_xy = top_xy;
    sl->left_mb_xy[0] = left_xy[0];
    sl->left_mb_xy[1] = left_xy[1];

    top_type = f->mb_type[top_xy];
    left_type[0] = f->mb_type[left_xy[0]];
    left_type[1] = f->mb_type[left_xy[1]];
    if (sl->deblocking_filter == 2) {
        if (f->slice_table[top_xy] != sl->slice_num)
            top_type = 0;
        if (f->slice_table[left_xy[1]] != sl->slice_num)
            left_type[0] = left_type[1] = 0;
    } else {
        if (f->slice_table[top_xy] == 0xFFFF)
            top_type = 0;
        if (f->slice_table[left_xy[1]] == 0xFFFF)
            left_type[0] = left_type[1] = 0;
    }
    sl->top_type = top_type;
    sl->left_type[0] = left_type[0];
    sl->left_type[1] = left_type[1];

    /* Say it before the caches are filled: filling them is most of what a
     * filtered-but-inert macroblock would have cost, and the answer does not
     * depend on them. Not for MBAFF: there the edges of a frame-coded pair also
     * address the QP of the partner field's macroblock, which is not one of the
     * four this looks at, and skipping a field pair on that evidence changes
     * pixels - measured, on the two MBAFF streams in the corpus. */
    if (!f->mbaff && f->bit_depth == 8 &&
        filter_mb_inert(sl, f, mb_xy, top_xy, left_xy))
        return 1;
    if (!f->mbaff && f->bit_depth == 8 &&
        filter_mb_uniform_mv(h, sl, f, mb_type, mb_xy, top_xy, left_xy,
                             top_type, left_type))
        return 1;

    if (IS_INTRA(mb_type))
        return 0;

    fill_filter_caches_inter(h, sl, f, mb_type, top_xy, left_xy,
                             top_type, left_type, mb_xy, 0);
    if (sl->list_count == 2)
        fill_filter_caches_inter(h, sl, f, mb_type, top_xy, left_xy,
                                 top_type, left_type, mb_xy, 1);

    {
        const uint8_t *nnz = f->nnz[mb_xy];
        uint8_t *nnz_cache = sl->non_zero_count_cache;
        memcpy(&nnz_cache[4 + 8 * 1], &nnz[0], 4);
        memcpy(&nnz_cache[4 + 8 * 2], &nnz[4], 4);
        memcpy(&nnz_cache[4 + 8 * 3], &nnz[8], 4);
        memcpy(&nnz_cache[4 + 8 * 4], &nnz[12], 4);
        sl->cbp = f->cbp_table[mb_xy];
    }

    if (top_type) {
        const uint8_t *nnz = f->nnz[top_xy];
        memcpy(&sl->non_zero_count_cache[4 + 8 * 0], &nnz[3 * 4], 4);
    }
    if (left_type[0]) {
        const uint8_t *nnz = f->nnz[left_xy[0]];
        sl->non_zero_count_cache[3 + 8 * 1] = nnz[3 + 0 * 4];
        sl->non_zero_count_cache[3 + 8 * 2] = nnz[3 + 1 * 4];
        sl->non_zero_count_cache[3 + 8 * 3] = nnz[3 + 2 * 4];
        sl->non_zero_count_cache[3 + 8 * 4] = nnz[3 + 3 * 4];
    }

    /* CAVLC 8x8dct nnz adjustments (needed for bS) */
    if (!f->cabac && 1 /* transform_8x8_mode handled per MB type */) {
        if (IS_8x8DCT(top_type)) {
            sl->non_zero_count_cache[4 + 8 * 0] =
            sl->non_zero_count_cache[5 + 8 * 0] = (f->cbp_table[top_xy] & 0x4000) >> 12;
            sl->non_zero_count_cache[6 + 8 * 0] =
            sl->non_zero_count_cache[7 + 8 * 0] = (f->cbp_table[top_xy] & 0x8000) >> 12;
        }
        if (IS_8x8DCT(left_type[0])) {
            sl->non_zero_count_cache[3 + 8 * 1] =
            sl->non_zero_count_cache[3 + 8 * 2] = (f->cbp_table[left_xy[0]] & 0x2000) >> 12;
        }
        if (IS_8x8DCT(left_type[1])) {
            sl->non_zero_count_cache[3 + 8 * 3] =
            sl->non_zero_count_cache[3 + 8 * 4] = (f->cbp_table[left_xy[1]] & 0x8000) >> 12;
        }
        if (IS_8x8DCT(mb_type)) {
            sl->non_zero_count_cache[h264_scan8[0]] = sl->non_zero_count_cache[h264_scan8[1]] =
            sl->non_zero_count_cache[h264_scan8[2]] = sl->non_zero_count_cache[h264_scan8[3]] =
                (sl->cbp & 0x1000) >> 12;
            sl->non_zero_count_cache[h264_scan8[0 + 4]] = sl->non_zero_count_cache[h264_scan8[1 + 4]] =
            sl->non_zero_count_cache[h264_scan8[2 + 4]] = sl->non_zero_count_cache[h264_scan8[3 + 4]] =
                (sl->cbp & 0x2000) >> 12;
            sl->non_zero_count_cache[h264_scan8[0 + 8]] = sl->non_zero_count_cache[h264_scan8[1 + 8]] =
            sl->non_zero_count_cache[h264_scan8[2 + 8]] = sl->non_zero_count_cache[h264_scan8[3 + 8]] =
                (sl->cbp & 0x4000) >> 12;
            sl->non_zero_count_cache[h264_scan8[0 + 12]] = sl->non_zero_count_cache[h264_scan8[1 + 12]] =
            sl->non_zero_count_cache[h264_scan8[2 + 12]] = sl->non_zero_count_cache[h264_scan8[3 + 12]] =
                (sl->cbp & 0x8000) >> 12;
        }
    }

    return 0;
}

/* ------------------------------------------------------------------ */
/* per-MB filtering                                                    */
/* ------------------------------------------------------------------ */
/* The reference compares the horizontal motion difference unsigned:
 * (dx + 3) >= 7 is true for any magnitude at or beyond 4, negatives included. */
static int mv_x_differs(int a, int b)
{
    return (unsigned)(a - b + 3) >= 7U;
}

static int mv_y_differs(int a, int b, int mvy_limit)
{
    return H264_ABS(a - b) >= mvy_limit;
}

static int check_mv(H264Slice *sl, int b_idx, int bn_idx, int mvy_limit)
{
    int v;
    v = sl->ref_cache[0][b_idx] != sl->ref_cache[0][bn_idx];
    if (!v && sl->ref_cache[0][b_idx] != -1)
        v = mv_x_differs(sl->mv_cache[0][b_idx][0], sl->mv_cache[0][bn_idx][0]) ||
            mv_y_differs(sl->mv_cache[0][b_idx][1], sl->mv_cache[0][bn_idx][1], mvy_limit);

    if (sl->list_count == 2) {
        if (!v)
            v = sl->ref_cache[1][b_idx] != sl->ref_cache[1][bn_idx] ||
                mv_x_differs(sl->mv_cache[1][b_idx][0], sl->mv_cache[1][bn_idx][0]) ||
                mv_y_differs(sl->mv_cache[1][b_idx][1], sl->mv_cache[1][bn_idx][1], mvy_limit);
        if (v) {
            if (sl->ref_cache[0][b_idx] != sl->ref_cache[1][bn_idx] ||
                sl->ref_cache[1][b_idx] != sl->ref_cache[0][bn_idx])
                return 1;
            return mv_x_differs(sl->mv_cache[0][b_idx][0], sl->mv_cache[1][bn_idx][0]) ||
                   mv_y_differs(sl->mv_cache[0][b_idx][1], sl->mv_cache[1][bn_idx][1], mvy_limit) ||
                   mv_x_differs(sl->mv_cache[1][b_idx][0], sl->mv_cache[0][bn_idx][0]) ||
                   mv_y_differs(sl->mv_cache[1][b_idx][1], sl->mv_cache[0][bn_idx][1], mvy_limit);
        }
    }
    return v;
}

static void filter_mb_dir(H264Decoder *h, H264Slice *sl, H264Frame *f,
                          int mb_x, int mb_y, uint8_t *img_y, uint8_t *img_cb,
                          uint8_t *img_cr, int linesize, int uvlinesize,
                          int mb_xy, uint32_t mb_type, int mvy_limit,
                          int first_vertical_edge_done, int a, int b, int dir)
{
    int edge;
    const int high = f->bit_depth > 8;
    const int bytes = high ? 2 : 1;
    const int c444 = f->chroma_format_idc == 3;
    const int c422 = f->chroma_format_idc == 2;
    const int chroma = f->chroma_format_idc != 0;
    int chroma_qp_avg[2];
    const int mbm_xy = dir == 0 ? mb_xy - 1 : sl->top_mb_xy;
    const int mbm_type = dir == 0 ? sl->left_type[0] : sl->top_type;
    static const uint8_t mask_edge_tab[2][8] = {
        { 0, 3, 3, 3, 1, 1, 1, 1 },
        { 0, 3, 1, 1, 3, 3, 3, 3 }
    };
    const int mask_edge = mask_edge_tab[dir][(mb_type >> 3) & 7];
    const int edges = (mask_edge == 3 && !(sl->cbp & 15)) ? 1 : 4;
    const int mask_par0 = mb_type & (MB_TYPE_16x16 | (MB_TYPE_8x16 >> dir));

    if (mbm_type && !first_vertical_edge_done) {
        if (f->mbaff && dir == 1 && ((mb_y & 1) == 0) &&
            IS_INTERLACED(mbm_type & ~mb_type)) {
            unsigned int tmp_linesize = 2 * linesize;
            unsigned int tmp_uvlinesize = 2 * uvlinesize;
            int mbn_xy = mb_xy - 2 * h->mb_stride;
            int j;
            for (j = 0; j < 2; j++, mbn_xy += h->mb_stride) {
                int bS[4];
                int qp;
                if (IS_INTRA(mb_type | f->mb_type[mbn_xy])) {
                    bS[0] = bS[1] = bS[2] = bS[3] = 3;
                } else {
                    if (!f->cabac && IS_8x8DCT(f->mb_type[mbn_xy])) {
                        bS[0] = 1 + ((f->cbp_table[mbn_xy] & 0x4000) || sl->non_zero_count_cache[h264_scan8[0] + 0]);
                        bS[1] = 1 + ((f->cbp_table[mbn_xy] & 0x4000) || sl->non_zero_count_cache[h264_scan8[0] + 1]);
                        bS[2] = 1 + ((f->cbp_table[mbn_xy] & 0x8000) || sl->non_zero_count_cache[h264_scan8[0] + 2]);
                        bS[3] = 1 + ((f->cbp_table[mbn_xy] & 0x8000) || sl->non_zero_count_cache[h264_scan8[0] + 3]);
                    } else {
                        const uint8_t *mbn_nnz = f->nnz[mbn_xy] + 3 * 4;
                        int i;
                        for (i = 0; i < 4; i++)
                            bS[i] = 1 + !!(sl->non_zero_count_cache[h264_scan8[0] + i] | mbn_nnz[i]);
                    }
                }
                qp = (f->qscale_table[mb_xy] + f->qscale_table[mbn_xy] + 1) >> 1;
                filter_mb_edgeh(&img_y[j * linesize], tmp_linesize, bS, qp, a, b, 0, high);
                chroma_qp_avg[0] = (h264_frame_get_chroma_qp(f, 0, f->qscale_table[mb_xy]) +
                                    h264_frame_get_chroma_qp(f, 0, f->qscale_table[mbn_xy]) + 1) >> 1;
                chroma_qp_avg[1] = (h264_frame_get_chroma_qp(f, 1, f->qscale_table[mb_xy]) +
                                    h264_frame_get_chroma_qp(f, 1, f->qscale_table[mbn_xy]) + 1) >> 1;
                if (chroma) {
                    if (c444) {
                        filter_mb_edgeh(&img_cb[j * uvlinesize], tmp_uvlinesize, bS,
                                        chroma_qp_avg[0], a, b, 0, high);
                        filter_mb_edgeh(&img_cr[j * uvlinesize], tmp_uvlinesize, bS,
                                        chroma_qp_avg[0], a, b, 0, high);
                    } else {
                        filter_mb_edgech(&img_cb[j * uvlinesize], tmp_uvlinesize, bS, chroma_qp_avg[0], a, b, 0, high);
                        filter_mb_edgech(&img_cr[j * uvlinesize], tmp_uvlinesize, bS, chroma_qp_avg[1], a, b, 0, high);
                    }
                }
            }
        } else {
            int bS[4];
            int qp;

            if (IS_INTRA(mb_type | mbm_type)) {
                bS[0] = bS[1] = bS[2] = bS[3] = 3;
                if ((!IS_INTERLACED(mb_type | mbm_type)) ||
                    ((f->mbaff || (f->picture_structure != PICT_FRAME)) && (dir == 0)))
                    bS[0] = bS[1] = bS[2] = bS[3] = 4;
            } else {
                int i;
                int mv_done;

                if (dir && f->mbaff && IS_INTERLACED(mb_type ^ mbm_type)) {
                    bS[0] = bS[1] = bS[2] = bS[3] = 1;
                    mv_done = 1;
                } else if (mask_par0 && ((mbm_type & (MB_TYPE_16x16 | (MB_TYPE_8x16 >> dir))))) {
                    const int b_idx = 8 + 4;
                    const int bn_idx = b_idx - (dir ? 8 : 1);
                    bS[0] = bS[1] = bS[2] = bS[3] = check_mv(sl, b_idx, bn_idx, mvy_limit);
                    mv_done = 1;
                } else {
                    mv_done = 0;
                }

                for (i = 0; i < 4; i++) {
                    int x = dir == 0 ? 0 : i;
                    int y = dir == 0 ? i : 0;
                    int b_idx = 8 + 4 + x + 8 * y;
                    int bn_idx = b_idx - (dir ? 8 : 1);

                    if (sl->non_zero_count_cache[b_idx] | sl->non_zero_count_cache[bn_idx])
                        bS[i] = 2;
                    else if (!mv_done)
                        bS[i] = check_mv(sl, b_idx, bn_idx, mvy_limit);
                }
            }

            if (bS[0] + bS[1] + bS[2] + bS[3]) {
                qp = (f->qscale_table[mb_xy] + f->qscale_table[mbm_xy] + 1) >> 1;
                chroma_qp_avg[0] = (h264_frame_get_chroma_qp(f, 0, f->qscale_table[mb_xy]) +
                                    h264_frame_get_chroma_qp(f, 0, f->qscale_table[mbm_xy]) + 1) >> 1;
                chroma_qp_avg[1] = (h264_frame_get_chroma_qp(f, 1, f->qscale_table[mb_xy]) +
                                    h264_frame_get_chroma_qp(f, 1, f->qscale_table[mbm_xy]) + 1) >> 1;
                if (dir == 0) {
                    filter_mb_edgev(&img_y[0], linesize, bS, qp, a, b, 1, high);
                    if (chroma) {
                        if (c444) {
                            const int qp_cr = IS_INTRA(mb_type) ? chroma_qp_avg[0] : chroma_qp_avg[1];
                            filter_mb_edgev(&img_cb[0], uvlinesize, bS, chroma_qp_avg[0], a, b, 1, high);
                            filter_mb_edgev(&img_cr[0], uvlinesize, bS, qp_cr, a, b, 1, high);
                        } else {
                            const int qp_cr = IS_INTRA(mb_type) ? chroma_qp_avg[0] : chroma_qp_avg[1];
                            filter_mb_edgecv(&img_cb[0], uvlinesize, bS, chroma_qp_avg[0], a, b, 1, high, c422);
                            filter_mb_edgecv(&img_cr[0], uvlinesize, bS, qp_cr, a, b, 1, high, c422);
                        }
                    }
                } else {
                    filter_mb_edgeh(&img_y[0], linesize, bS, qp, a, b, 1, high);
                    if (chroma) {
                        if (c444) {
                            const int qp_cr = IS_INTRA(mb_type) ? chroma_qp_avg[0] : chroma_qp_avg[1];
                            filter_mb_edgeh(&img_cb[0], uvlinesize, bS, chroma_qp_avg[0], a, b, 1, high);
                            filter_mb_edgeh(&img_cr[0], uvlinesize, bS, qp_cr, a, b, 1, high);
                        } else {
                            const int qp_cr = IS_INTRA(mb_type) ? chroma_qp_avg[0] : chroma_qp_avg[1];
                            filter_mb_edgech(&img_cb[0], uvlinesize, bS, chroma_qp_avg[0], a, b, 1, high);
                            filter_mb_edgech(&img_cr[0], uvlinesize, bS, qp_cr, a, b, 1, high);
                        }
                    }
                }
            }
        }
    }

    for (edge = 1; edge < edges; edge++) {
        int bS[4];
        int qp;
        const int deblock_edge = !IS_8x8DCT(mb_type & (edge << 24));
        int i, mv_done;

        if (!deblock_edge && (!c422 || dir == 0))
            continue;

        if (IS_INTRA(mb_type)) {
            bS[0] = bS[1] = bS[2] = bS[3] = 3;
        } else {
            if (edge & mask_edge) {
                bS[0] = bS[1] = bS[2] = bS[3] = 0;
                mv_done = 1;
            } else if (mask_par0) {
                const int b_idx = 8 + 4 + edge * (dir ? 8 : 1);
                const int bn_idx = b_idx - (dir ? 8 : 1);
                bS[0] = bS[1] = bS[2] = bS[3] = check_mv(sl, b_idx, bn_idx, mvy_limit);
                mv_done = 1;
            } else {
                mv_done = 0;
            }

            for (i = 0; i < 4; i++) {
                int x = dir == 0 ? edge : i;
                int y = dir == 0 ? i : edge;
                int b_idx = 8 + 4 + x + 8 * y;
                int bn_idx = b_idx - (dir ? 8 : 1);

                if (sl->non_zero_count_cache[b_idx] | sl->non_zero_count_cache[bn_idx])
                    bS[i] = 2;
                else if (!mv_done)
                    bS[i] = check_mv(sl, b_idx, bn_idx, mvy_limit);
            }
            if (bS[0] + bS[1] + bS[2] + bS[3] == 0)
                continue;
        }

        qp = f->qscale_table[mb_xy];
        if (dir == 0) {
            filter_mb_edgev(&img_y[4 * edge * bytes], linesize, bS, qp, a, b, 0, high);
            if (chroma) {
                if (c444) {
                    const int qpc0 = h264_frame_get_chroma_qp(f, 0, qp);
                    const int qpc1 = IS_INTRA(mb_type) ? qpc0 : h264_frame_get_chroma_qp(f, 1, qp);
                    filter_mb_edgev(&img_cb[4 * edge * bytes], uvlinesize, bS, qpc0, a, b, 0, high);
                    filter_mb_edgev(&img_cr[4 * edge * bytes], uvlinesize, bS, qpc1, a, b, 0, high);
                } else if ((edge & 1) == 0) {
                    const int qpc0 = h264_frame_get_chroma_qp(f, 0, qp);
                    const int qpc1 = IS_INTRA(mb_type) ? qpc0 : h264_frame_get_chroma_qp(f, 1, qp);
                    filter_mb_edgecv(&img_cb[2 * edge * bytes], uvlinesize, bS,
                                     qpc0, a, b, 0, high, c422);
                    filter_mb_edgecv(&img_cr[2 * edge * bytes], uvlinesize, bS,
                                     qpc1, a, b, 0, high, c422);
                }
            }
        } else {
            if (deblock_edge)
                filter_mb_edgeh(&img_y[4 * edge * linesize], linesize, bS, qp, a, b, 0, high);
            if (chroma) {
                if (c444) {
                    const int qpc0 = h264_frame_get_chroma_qp(f, 0, qp);
                    const int qpc1 = IS_INTRA(mb_type) ? qpc0 : h264_frame_get_chroma_qp(f, 1, qp);
                    filter_mb_edgeh(&img_cb[4 * edge * uvlinesize], uvlinesize, bS, qpc0, a, b, 0, high);
                    filter_mb_edgeh(&img_cr[4 * edge * uvlinesize], uvlinesize, bS, qpc1, a, b, 0, high);
                } else if (c422) {
                    const int qpc0 = h264_frame_get_chroma_qp(f, 0, qp);
                    const int qpc1 = IS_INTRA(mb_type) ? qpc0 : h264_frame_get_chroma_qp(f, 1, qp);
                    filter_mb_edgech(&img_cb[4 * edge * uvlinesize], uvlinesize, bS,
                                     qpc0, a, b, 0, high);
                    filter_mb_edgech(&img_cr[4 * edge * uvlinesize], uvlinesize, bS,
                                     qpc1, a, b, 0, high);
                } else if ((edge & 1) == 0) {
                    const int qpc0 = h264_frame_get_chroma_qp(f, 0, qp);
                    const int qpc1 = IS_INTRA(mb_type) ? qpc0 : h264_frame_get_chroma_qp(f, 1, qp);
                    filter_mb_edgech(&img_cb[2 * edge * uvlinesize], uvlinesize, bS,
                                     qpc0, a, b, 0, high);
                    filter_mb_edgech(&img_cr[2 * edge * uvlinesize], uvlinesize, bS,
                                     qpc1, a, b, 0, high);
                }
            }
        }
    }
}

void h264_filter_mb(H264Decoder *h, H264Slice *sl, H264Frame *f, int mb_x, int mb_y,
                    uint8_t *img_y, uint8_t *img_cb, uint8_t *img_cr,
                    int linesize, int uvlinesize)
{
    const int mb_xy = mb_x + mb_y * h->mb_stride;
    const uint32_t mb_type = f->mb_type[mb_xy];
    const int mvy_limit = IS_INTERLACED(mb_type) ? 2 : 4;
    const int high = f->bit_depth > 8;
    const int bd_off = 6 * (f->bit_depth - 8);
    int first_vertical_edge_done = 0;
    int a = 52 + sl->slice_alpha_c0_offset - bd_off;
    int b = 52 + sl->slice_beta_offset - bd_off;

    if (f->mbaff && IS_INTERLACED(mb_type ^ sl->left_type[0]) && sl->left_type[0]) {
        int bS[8];
        int qp[2], bqp[2], rqp[2];
        int mb_qp, mbn0_qp, mbn1_qp;
        int i;
        static const uint8_t offset_tab[2][2][8] = {
            {
                { 3+4*0, 3+4*0, 3+4*0, 3+4*0, 3+4*1, 3+4*1, 3+4*1, 3+4*1 },
                { 3+4*2, 3+4*2, 3+4*2, 3+4*2, 3+4*3, 3+4*3, 3+4*3, 3+4*3 },
            }, {
                { 3+4*0, 3+4*1, 3+4*2, 3+4*3, 3+4*0, 3+4*1, 3+4*2, 3+4*3 },
                { 3+4*0, 3+4*1, 3+4*2, 3+4*3, 3+4*0, 3+4*1, 3+4*2, 3+4*3 },
            }
        };
        first_vertical_edge_done = 1;

        if (IS_INTRA(mb_type)) {
            for (i = 0; i < 8; i++) bS[i] = 4;
        } else {
            const uint8_t *off = offset_tab[MB_FIELD(sl)][mb_y & 1];
            for (i = 0; i < 8; i++) {
                int j = MB_FIELD(sl) ? i >> 2 : i & 1;
                int mbn_xy = sl->left_mb_xy[j];
                uint32_t mbn_type = (uint32_t)sl->left_type[j];

                if (IS_INTRA(mbn_type)) {
                    bS[i] = 4;
                } else {
                    bS[i] = 1 + !!(sl->non_zero_count_cache[12 + 8 * (i >> 1)] |
                         ((!f->cabac && IS_8x8DCT(mbn_type)) ?
                            (f->cbp_table[mbn_xy] & (((MB_FIELD(sl) ? (i & 2) : (mb_y & 1)) ? 8 : 2) << 12)) :
                            f->nnz[mbn_xy][off[i]]));
                }
            }
        }

        mb_qp = f->qscale_table[mb_xy];
        mbn0_qp = f->qscale_table[sl->left_mb_xy[0]];
        mbn1_qp = f->qscale_table[sl->left_mb_xy[1]];
        qp[0] = (mb_qp + mbn0_qp + 1) >> 1;
        bqp[0] = (h264_frame_get_chroma_qp(f, 0, mb_qp) + h264_frame_get_chroma_qp(f, 0, mbn0_qp) + 1) >> 1;
        rqp[0] = (h264_frame_get_chroma_qp(f, 1, mb_qp) + h264_frame_get_chroma_qp(f, 1, mbn0_qp) + 1) >> 1;
        qp[1] = (mb_qp + mbn1_qp + 1) >> 1;
        bqp[1] = (h264_frame_get_chroma_qp(f, 0, mb_qp) + h264_frame_get_chroma_qp(f, 0, mbn1_qp) + 1) >> 1;
        rqp[1] = (h264_frame_get_chroma_qp(f, 1, mb_qp) + h264_frame_get_chroma_qp(f, 1, mbn1_qp) + 1) >> 1;

        if (MB_FIELD(sl)) {
            filter_mb_mbaff_edgev(img_y, linesize, bS, 1, qp[0], a, b, 1, high);
            filter_mb_mbaff_edgev(img_y + 8 * linesize, linesize, bS + 4, 1, qp[1], a, b, 1, high);
            if (f->chroma_format_idc == 3) {
                filter_mb_mbaff_edgev(img_cb, uvlinesize, bS, 1, bqp[0], a, b, 1, high);
                filter_mb_mbaff_edgev(img_cb + 8 * uvlinesize, uvlinesize, bS + 4, 1, bqp[1], a, b, 1, high);
                filter_mb_mbaff_edgev(img_cr, uvlinesize, bS, 1, rqp[0], a, b, 1, high);
                filter_mb_mbaff_edgev(img_cr + 8 * uvlinesize, uvlinesize, bS + 4, 1, rqp[1], a, b, 1, high);
            } else if (f->chroma_format_idc == 2) {
                filter_mb_mbaff_edgecv(img_cb, uvlinesize, bS, 1, bqp[0], a, b, 1, high, 1);
                filter_mb_mbaff_edgecv(img_cb + 8 * uvlinesize, uvlinesize, bS + 4, 1, bqp[1], a, b, 1, high, 1);
                filter_mb_mbaff_edgecv(img_cr, uvlinesize, bS, 1, rqp[0], a, b, 1, high, 1);
                filter_mb_mbaff_edgecv(img_cr + 8 * uvlinesize, uvlinesize, bS + 4, 1, rqp[1], a, b, 1, high, 1);
            } else if (f->chroma_format_idc) {
                filter_mb_mbaff_edgecv(img_cb, uvlinesize, bS, 1, bqp[0], a, b, 1, high, 0);
                filter_mb_mbaff_edgecv(img_cb + 4 * uvlinesize, uvlinesize, bS + 4, 1, bqp[1], a, b, 1, high, 0);
                filter_mb_mbaff_edgecv(img_cr, uvlinesize, bS, 1, rqp[0], a, b, 1, high, 0);
                filter_mb_mbaff_edgecv(img_cr + 4 * uvlinesize, uvlinesize, bS + 4, 1, rqp[1], a, b, 1, high, 0);
            }
        } else {
            filter_mb_mbaff_edgev(img_y, 2 * linesize, bS, 2, qp[0], a, b, 1, high);
            filter_mb_mbaff_edgev(img_y + linesize, 2 * linesize, bS + 1, 2, qp[1], a, b, 1, high);
            if (f->chroma_format_idc == 3) {
                filter_mb_mbaff_edgev(img_cb, 2 * uvlinesize, bS, 2, bqp[0], a, b, 1, high);
                filter_mb_mbaff_edgev(img_cb + uvlinesize, 2 * uvlinesize, bS + 1, 2, bqp[1], a, b, 1, high);
                filter_mb_mbaff_edgev(img_cr, 2 * uvlinesize, bS, 2, rqp[0], a, b, 1, high);
                filter_mb_mbaff_edgev(img_cr + uvlinesize, 2 * uvlinesize, bS + 1, 2, rqp[1], a, b, 1, high);
            } else if (f->chroma_format_idc) {
                filter_mb_mbaff_edgecv(img_cb, 2 * uvlinesize, bS, 2, bqp[0], a, b, 1, high,
                                       f->chroma_format_idc == 2);
                filter_mb_mbaff_edgecv(img_cb + uvlinesize, 2 * uvlinesize, bS + 1, 2, bqp[1], a, b, 1, high,
                                       f->chroma_format_idc == 2);
                filter_mb_mbaff_edgecv(img_cr, 2 * uvlinesize, bS, 2, rqp[0], a, b, 1, high,
                                       f->chroma_format_idc == 2);
                filter_mb_mbaff_edgecv(img_cr + uvlinesize, 2 * uvlinesize, bS + 1, 2, rqp[1], a, b, 1, high,
                                       f->chroma_format_idc == 2);
            }
        }
    }

    filter_mb_dir(h, sl, f, mb_x, mb_y, img_y, img_cb, img_cr, linesize,
                  uvlinesize, mb_xy, mb_type, mvy_limit,
                  first_vertical_edge_done, a, b, 0);
    filter_mb_dir(h, sl, f, mb_x, mb_y, img_y, img_cb, img_cr, linesize,
                  uvlinesize, mb_xy, mb_type, mvy_limit,
                  0, a, b, 1);
}

/* ------------------------------------------------------------------ */
/* row-incremental deblocking                                          */
/* ------------------------------------------------------------------ */
/* A horizontal edge between two MB rows belongs to the row below it, so
 * filtering MB row mb_y only reads and writes rows mb_y-1 and mb_y. The pass
 * can therefore follow the decoder one row at a time instead of running after
 * the whole picture, which is what lets another picture be motion
 * compensated from the rows that are already finished. */
static void deblock_mb(H264Decoder *h, H264Slice *sl, H264Frame *f,
                       uint8_t *const *data, int mb_x, int mb_y)
{
    const int mb_xy = mb_x + mb_y * h->mb_stride;
    const uint32_t mb_type = f->mb_type[mb_xy];
    const int bytes = f->bit_depth > 8 ? 2 : 1;
    int mb_field, linesize, uvlinesize;
    uint8_t *dy, *dcb, *dcr;

    if (f->slice_table[mb_xy] == 0xFFFF)
        return;
    sl->mb_xy = mb_xy;
    sl->mb_x = mb_x;
    sl->mb_y = mb_y;
    sl->slice_num = f->slice_table[mb_xy];
    sl->deblocking_filter = f->mb_deblock_idc[mb_xy];
    if (!sl->deblocking_filter)
        return;
    /* a field picture codes every macroblock as a field */
    mb_field = f->field_picture ? 1 : !!IS_INTERLACED(mb_type);
    sl->mb_field_decoding_flag = mb_field;
    sl->mb_mbaff = mb_field;
    sl->slice_alpha_c0_offset = f->mb_alpha[mb_xy];
    sl->slice_beta_offset = f->mb_beta[mb_xy];
    sl->chroma_qp[0] = h264_frame_get_chroma_qp(f, 0, f->qscale_table[mb_xy]);
    sl->chroma_qp[1] = h264_frame_get_chroma_qp(f, 1, f->qscale_table[mb_xy]);
    sl->list_count = f->list_counts[mb_xy];
    linesize = f->linesize[0] << mb_field;
    uvlinesize = f->linesize[1] << mb_field;
    dy = data[0] + mb_x * 16 * bytes + mb_y * f->linesize[0] * 16;
    {
        const int cidc = f->chroma_format_idc;
        if (cidc) {
            const int cw = cidc == 3 ? 16 : 8;
            const int ch = cidc == 1 ? 8 : 16;
            dcb = data[1] + mb_x * cw * bytes + mb_y * f->linesize[1] * ch;
            dcr = data[2] + mb_x * cw * bytes + mb_y * f->linesize[2] * ch;
        } else {
            dcb = dcr = NULL;
        }
    }
    if (mb_field && (mb_y & 1)) {
        const int ch = f->chroma_format_idc == 1 ? 7 : 15;
        dy -= f->linesize[0] * 15;
        if (f->chroma_format_idc) {
            dcb -= f->linesize[1] * ch;
            dcr -= f->linesize[2] * ch;
        }
    }
    if (fill_filter_caches(h, sl, f, mb_type))
        return;
    h264_filter_mb(h, sl, f, mb_x, mb_y, dy, dcb, dcr, linesize, uvlinesize);
}

/* one MB row, or the two MBs of an MBAFF pair column by column */
static void deblock_unit(H264Decoder *h, H264Slice *sl, H264Frame *f,
                         uint8_t *const *data, int mb_y, int pair)
{
    int mb_x, yy;
    for (mb_x = 0; mb_x < h->mb_width; mb_x++) {
        for (yy = mb_y; yy < mb_y + pair && yy < h->mb_height; yy++)
            deblock_mb(h, sl, f, data, mb_x, yy);
    }
}

void h264_deblock_upto(H264Decoder *h, H264Frame *f, int structure, int until)
{
    H264Slice *sl = f->filter_sl;   /* the caches travel with the picture, not the thread */
    uint8_t *data[3];
    const int step = (f->mbaff || f->field_picture) ? 2 : 1;
    const int pair = f->mbaff ? 2 : 1;
    const int parity = f->field_picture && structure == PICT_BOTTOM_FIELD;
    const int start = f->deblock_row[parity];
    int mb_y;

    if (until > h->mb_height)
        until = h->mb_height;
    if (h->param.disable_lf) {
        /* nothing is filtered, so every decoded row is already final */
        if (until > start)
            f->deblock_row[parity] = until - (until - start) % step;
        goto report;
    }

    /* The filter caches of the previous macroblock are reused, so they have to
     * survive from one row to the next: start a pass with cleared caches,
     * exactly like a single pass over the whole picture did. */
    if (start == parity)
        memset(sl, 0, sizeof(*sl));
    sl->h = h;
    sl->picture_structure = structure;
    data[0] = f->data[0];
    data[1] = f->data[1];
    data[2] = f->data[2];

    for (mb_y = start; mb_y < until; mb_y += step)
        deblock_unit(h, sl, f, data, mb_y, pair);
    f->deblock_row[parity] = mb_y;

report:
    /* Publish the last row whose samples can no longer change: the row that
     * was just filtered still shares its bottom edge with the row below, so
     * only the row above it is final. Other pictures wait on these lines
     * before motion compensating from them. */
    if (f->deblock_row[parity] != start) {
        const int last = f->deblock_row[parity] - step;
        h264_progress_report(h, f,
                             16 * (f->field_picture ? (last >> 1) : last) - 1,
                             parity);
    }
}
