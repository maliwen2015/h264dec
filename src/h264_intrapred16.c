/*
 * H.264 8-bit 4:2:0 intra prediction (all modes).
 * Algorithms from ITU-T H.264 and FFmpeg's h264pred_template.c (LGPL-2.1+).
 */
#include "h264_internal.h"

static inline uint16_t clip10(int v)
{
    return (uint16_t)(v < 0 ? 0 : v > 1023 ? 1023 : v);
}

static inline void fill_px(uint16_t *dst, uint16_t v, int n)
{
    while (n--) *dst++ = v;
}

/* ---------------- 4x4 ---------------- */
static void pred4x4_vertical(uint16_t *src, const uint16_t *topright, int stride)
{
    const uint16_t *t = src - stride;
    int i;
    for (i = 0; i < 4; i++)
        memcpy(src + i * stride, t, 8);
}

static void pred4x4_horizontal(uint16_t *src, const uint16_t *topright, int stride)
{
    int i;
    for (i = 0; i < 4; i++)
        fill_px(src + i * stride, src[-1 + i * stride], 4);
}

static void pred4x4_dc(uint16_t *src, const uint16_t *topright, int stride)
{
    int dc = (src[-stride] + src[1 - stride] + src[2 - stride] + src[3 - stride] +
              src[-1] + src[-1 + stride] + src[-1 + 2 * stride] + src[-1 + 3 * stride] + 4) >> 3;
    int i;
    for (i = 0; i < 4; i++)
        fill_px(src + i * stride, dc, 4);
}

static void pred4x4_left_dc(uint16_t *src, const uint16_t *topright, int stride)
{
    int dc = (src[-1] + src[-1 + stride] + src[-1 + 2 * stride] + src[-1 + 3 * stride] + 2) >> 2;
    int i;
    for (i = 0; i < 4; i++)
        fill_px(src + i * stride, dc, 4);
}

static void pred4x4_top_dc(uint16_t *src, const uint16_t *topright, int stride)
{
    int dc = (src[-stride] + src[1 - stride] + src[2 - stride] + src[3 - stride] + 2) >> 2;
    int i;
    for (i = 0; i < 4; i++)
        fill_px(src + i * stride, dc, 4);
}

static void pred4x4_128_dc(uint16_t *src, const uint16_t *topright, int stride)
{
    int i;
    for (i = 0; i < 4; i++)
        fill_px(src + i * stride, 512, 4);
}

static void pred4x4_down_right(uint16_t *src, const uint16_t *topright, int stride)
{
    const int lt = src[-1 - stride];
    const int t0 = src[-stride], t1 = src[1 - stride], t2 = src[2 - stride], t3 = src[3 - stride];
    const int l0 = src[-1], l1 = src[-1 + stride], l2 = src[-1 + 2 * stride], l3 = src[-1 + 3 * stride];

    src[0 + 3 * stride] = (l3 + 2 * l2 + l1 + 2) >> 2;
    src[0 + 2 * stride] = src[1 + 3 * stride] = (l2 + 2 * l1 + l0 + 2) >> 2;
    src[0 + 1 * stride] = src[1 + 2 * stride] = src[2 + 3 * stride] = (l1 + 2 * l0 + lt + 2) >> 2;
    src[0 + 0 * stride] = src[1 + 1 * stride] = src[2 + 2 * stride] = src[3 + 3 * stride] =
        (l0 + 2 * lt + t0 + 2) >> 2;
    src[1 + 0 * stride] = src[2 + 1 * stride] = src[3 + 2 * stride] = (lt + 2 * t0 + t1 + 2) >> 2;
    src[2 + 0 * stride] = src[3 + 1 * stride] = (t0 + 2 * t1 + t2 + 2) >> 2;
    src[3 + 0 * stride] = (t1 + 2 * t2 + t3 + 2) >> 2;
}

static void pred4x4_down_left(uint16_t *src, const uint16_t *topright, int stride)
{
    const int t0 = src[-stride], t1 = src[1 - stride], t2 = src[2 - stride], t3 = src[3 - stride];
    const int t4 = topright[0], t5 = topright[1], t6 = topright[2], t7 = topright[3];

    src[0 + 0 * stride] = (t0 + t2 + 2 * t1 + 2) >> 2;
    src[1 + 0 * stride] = src[0 + 1 * stride] = (t1 + t3 + 2 * t2 + 2) >> 2;
    src[2 + 0 * stride] = src[1 + 1 * stride] = src[0 + 2 * stride] = (t2 + t4 + 2 * t3 + 2) >> 2;
    src[3 + 0 * stride] = src[2 + 1 * stride] = src[1 + 2 * stride] = src[0 + 3 * stride] =
        (t3 + t5 + 2 * t4 + 2) >> 2;
    src[3 + 1 * stride] = src[2 + 2 * stride] = src[1 + 3 * stride] = (t4 + t6 + 2 * t5 + 2) >> 2;
    src[3 + 2 * stride] = src[2 + 3 * stride] = (t5 + t7 + 2 * t6 + 2) >> 2;
    src[3 + 3 * stride] = (t6 + 3 * t7 + 2) >> 2;
}

static void pred4x4_vertical_right(uint16_t *src, const uint16_t *topright, int stride)
{
    const int lt = src[-1 - stride];
    const int t0 = src[-stride], t1 = src[1 - stride], t2 = src[2 - stride], t3 = src[3 - stride];
    const int l0 = src[-1], l1 = src[-1 + stride], l2 = src[-1 + 2 * stride];

    src[0 + 0 * stride] = src[1 + 2 * stride] = (lt + t0 + 1) >> 1;
    src[1 + 0 * stride] = src[2 + 2 * stride] = (t0 + t1 + 1) >> 1;
    src[2 + 0 * stride] = src[3 + 2 * stride] = (t1 + t2 + 1) >> 1;
    src[3 + 0 * stride] = (t2 + t3 + 1) >> 1;
    src[0 + 1 * stride] = src[1 + 3 * stride] = (l0 + 2 * lt + t0 + 2) >> 2;
    src[1 + 1 * stride] = src[2 + 3 * stride] = (lt + 2 * t0 + t1 + 2) >> 2;
    src[2 + 1 * stride] = src[3 + 3 * stride] = (t0 + 2 * t1 + t2 + 2) >> 2;
    src[3 + 1 * stride] = (t1 + 2 * t2 + t3 + 2) >> 2;
    src[0 + 2 * stride] = (lt + 2 * l0 + l1 + 2) >> 2;
    src[0 + 3 * stride] = (l0 + 2 * l1 + l2 + 2) >> 2;
}

static void pred4x4_vertical_left(uint16_t *src, const uint16_t *topright, int stride)
{
    const int t0 = src[-stride], t1 = src[1 - stride], t2 = src[2 - stride], t3 = src[3 - stride];
    const int t4 = topright[0], t5 = topright[1], t6 = topright[2];

    src[0 + 0 * stride] = (t0 + t1 + 1) >> 1;
    src[1 + 0 * stride] = src[0 + 2 * stride] = (t1 + t2 + 1) >> 1;
    src[2 + 0 * stride] = src[1 + 2 * stride] = (t2 + t3 + 1) >> 1;
    src[3 + 0 * stride] = src[2 + 2 * stride] = (t3 + t4 + 1) >> 1;
    src[3 + 2 * stride] = (t4 + t5 + 1) >> 1;
    src[0 + 1 * stride] = (t0 + 2 * t1 + t2 + 2) >> 2;
    src[1 + 1 * stride] = src[0 + 3 * stride] = (t1 + 2 * t2 + t3 + 2) >> 2;
    src[2 + 1 * stride] = src[1 + 3 * stride] = (t2 + 2 * t3 + t4 + 2) >> 2;
    src[3 + 1 * stride] = src[2 + 3 * stride] = (t3 + 2 * t4 + t5 + 2) >> 2;
    src[3 + 3 * stride] = (t4 + 2 * t5 + t6 + 2) >> 2;
}

static void pred4x4_horizontal_up(uint16_t *src, const uint16_t *topright, int stride)
{
    const int l0 = src[-1], l1 = src[-1 + stride], l2 = src[-1 + 2 * stride], l3 = src[-1 + 3 * stride];

    src[0 + 0 * stride] = (l0 + l1 + 1) >> 1;
    src[1 + 0 * stride] = (l0 + 2 * l1 + l2 + 2) >> 2;
    src[2 + 0 * stride] = src[0 + 1 * stride] = (l1 + l2 + 1) >> 1;
    src[3 + 0 * stride] = src[1 + 1 * stride] = (l1 + 2 * l2 + l3 + 2) >> 2;
    src[2 + 1 * stride] = src[0 + 2 * stride] = (l2 + l3 + 1) >> 1;
    src[3 + 1 * stride] = src[1 + 2 * stride] = (l2 + 2 * l3 + l3 + 2) >> 2;
    src[3 + 2 * stride] = src[1 + 3 * stride] = src[0 + 3 * stride] =
    src[2 + 2 * stride] = src[2 + 3 * stride] = src[3 + 3 * stride] = l3;
}

static void pred4x4_horizontal_down(uint16_t *src, const uint16_t *topright, int stride)
{
    const int lt = src[-1 - stride];
    const int t0 = src[-stride], t1 = src[1 - stride], t2 = src[2 - stride];
    const int l0 = src[-1], l1 = src[-1 + stride], l2 = src[-1 + 2 * stride], l3 = src[-1 + 3 * stride];

    src[0 + 0 * stride] = src[2 + 1 * stride] = (lt + l0 + 1) >> 1;
    src[1 + 0 * stride] = src[3 + 1 * stride] = (l0 + 2 * lt + t0 + 2) >> 2;
    src[2 + 0 * stride] = (lt + 2 * t0 + t1 + 2) >> 2;
    src[3 + 0 * stride] = (t0 + 2 * t1 + t2 + 2) >> 2;
    src[0 + 1 * stride] = src[2 + 2 * stride] = (l0 + l1 + 1) >> 1;
    src[1 + 1 * stride] = src[3 + 2 * stride] = (lt + 2 * l0 + l1 + 2) >> 2;
    src[0 + 2 * stride] = src[2 + 3 * stride] = (l1 + l2 + 1) >> 1;
    src[1 + 2 * stride] = src[3 + 3 * stride] = (l0 + 2 * l1 + l2 + 2) >> 2;
    src[0 + 3 * stride] = (l2 + l3 + 1) >> 1;
    src[1 + 3 * stride] = (l1 + 2 * l2 + l3 + 2) >> 2;
}

/* ---------------- 8x8 luma (High profile I8x8) ---------------- */
#define SRC8(x, y) src[(x) + (y) * stride]

static void pred8x8l_vertical(uint16_t *src, int has_topleft, int has_topright, int stride)
{
    uint16_t t[8];
    int i, y;
    t[0] = (uint16_t)(((has_topleft ? src[-1 - stride] : src[-stride])
                      + 2 * src[-stride] + src[1 - stride] + 2) >> 2);
    for (i = 1; i <= 6; i++)
        t[i] = (uint16_t)((src[i - 1 - stride] + 2 * src[i - stride]
                          + src[i + 1 - stride] + 2) >> 2);
    t[7] = (uint16_t)(((has_topright ? src[8 - stride] : src[7 - stride])
                      + 2 * src[7 - stride] + src[6 - stride] + 2) >> 2);
    for (y = 0; y < 8; y++)
        memcpy(src + y * stride, t, 16);
}

static void pred8x8l_horizontal(uint16_t *src, int has_topleft, int has_topright, int stride)
{
    int l[8], i;
    l[0] = ((has_topleft ? src[-1 - stride] : src[-1]) + 2 * src[-1] + src[-1 + stride] + 2) >> 2;
    for (i = 1; i <= 6; i++)
        l[i] = (src[-1 + (i - 1) * stride] + 2 * src[-1 + i * stride] + src[-1 + (i + 1) * stride] + 2) >> 2;
    l[7] = (src[-1 + 6 * stride] + 3 * src[-1 + 7 * stride] + 2) >> 2;
    for (i = 0; i < 8; i++)
        fill_px(src + i * stride, l[i], 8);
}

static void pred8x8l_dc(uint16_t *src, int has_topleft, int has_topright, int stride)
{
    int t[8], l[8], i, dc;
    t[0] = ((has_topleft ? src[-1 - stride] : src[-stride]) + 2 * src[-stride] + src[1 - stride] + 2) >> 2;
    for (i = 1; i <= 6; i++)
        t[i] = (src[i - 1 - stride] + 2 * src[i - stride] + src[i + 1 - stride] + 2) >> 2;
    t[7] = ((has_topright ? src[8 - stride] : src[7 - stride]) + 2 * src[7 - stride] + src[6 - stride] + 2) >> 2;
    l[0] = ((has_topleft ? src[-1 - stride] : src[-1]) + 2 * src[-1] + src[-1 + stride] + 2) >> 2;
    for (i = 1; i <= 6; i++)
        l[i] = (src[-1 + (i - 1) * stride] + 2 * src[-1 + i * stride] + src[-1 + (i + 1) * stride] + 2) >> 2;
    l[7] = (src[-1 + 6 * stride] + 3 * src[-1 + 7 * stride] + 2) >> 2;
    dc = (l[0] + l[1] + l[2] + l[3] + l[4] + l[5] + l[6] + l[7] +
          t[0] + t[1] + t[2] + t[3] + t[4] + t[5] + t[6] + t[7] + 8) >> 4;
    for (i = 0; i < 8; i++)
        fill_px(src + i * stride, dc, 8);
}

static void pred8x8l_left_dc(uint16_t *src, int has_topleft, int has_topright, int stride)
{
    int l[8], i, dc;
    l[0] = ((has_topleft ? src[-1 - stride] : src[-1]) + 2 * src[-1] + src[-1 + stride] + 2) >> 2;
    for (i = 1; i <= 6; i++)
        l[i] = (src[-1 + (i - 1) * stride] + 2 * src[-1 + i * stride] + src[-1 + (i + 1) * stride] + 2) >> 2;
    l[7] = (src[-1 + 6 * stride] + 3 * src[-1 + 7 * stride] + 2) >> 2;
    dc = (l[0] + l[1] + l[2] + l[3] + l[4] + l[5] + l[6] + l[7] + 4) >> 3;
    for (i = 0; i < 8; i++)
        fill_px(src + i * stride, dc, 8);
}

static void pred8x8l_top_dc(uint16_t *src, int has_topleft, int has_topright, int stride)
{
    int t[8], i, dc;
    t[0] = ((has_topleft ? src[-1 - stride] : src[-stride]) + 2 * src[-stride] + src[1 - stride] + 2) >> 2;
    for (i = 1; i <= 6; i++)
        t[i] = (src[i - 1 - stride] + 2 * src[i - stride] + src[i + 1 - stride] + 2) >> 2;
    t[7] = ((has_topright ? src[8 - stride] : src[7 - stride]) + 2 * src[7 - stride] + src[6 - stride] + 2) >> 2;
    dc = (t[0] + t[1] + t[2] + t[3] + t[4] + t[5] + t[6] + t[7] + 4) >> 3;
    for (i = 0; i < 8; i++)
        fill_px(src + i * stride, dc, 8);
}

static void pred8x8l_128_dc(uint16_t *src, int has_topleft, int has_topright, int stride)
{
    int i;
    for (i = 0; i < 8; i++)
        fill_px(src + i * stride, 512, 8);
}

static void pred8x8l_down_left(uint16_t *src, int has_topleft, int has_topright, int stride)
{
    int t[16], i;
    t[0] = ((has_topleft ? src[-1 - stride] : src[-stride]) + 2 * src[-stride] + src[1 - stride] + 2) >> 2;
    for (i = 1; i <= 6; i++)
        t[i] = (src[i - 1 - stride] + 2 * src[i - stride] + src[i + 1 - stride] + 2) >> 2;
    t[7] = ((has_topright ? src[8 - stride] : src[7 - stride]) + 2 * src[7 - stride] + src[6 - stride] + 2) >> 2;
    if (has_topright) {
        for (i = 8; i <= 14; i++)
            t[i] = (src[i - 1 - stride] + 2 * src[i - stride] + src[i + 1 - stride] + 2) >> 2;
        t[15] = (src[14 - stride] + 3 * src[15 - stride] + 2) >> 2;
    } else {
        for (i = 8; i < 16; i++)
            t[i] = src[7 - stride];
    }
#define PUT_DL(x, y, a, b, c) do { int s = (a) + 2 * (b) + (c) + 2; SRC8(x, y) = (uint16_t)(s >> 2); } while (0)
    PUT_DL(0,0,t[0],t[1],t[2]);
    PUT_DL(1,0,t[1],t[2],t[3]); PUT_DL(0,1,t[1],t[2],t[3]);
    PUT_DL(2,0,t[2],t[3],t[4]); PUT_DL(1,1,t[2],t[3],t[4]); PUT_DL(0,2,t[2],t[3],t[4]);
    PUT_DL(3,0,t[3],t[4],t[5]); PUT_DL(2,1,t[3],t[4],t[5]); PUT_DL(1,2,t[3],t[4],t[5]); PUT_DL(0,3,t[3],t[4],t[5]);
    PUT_DL(4,0,t[4],t[5],t[6]); PUT_DL(3,1,t[4],t[5],t[6]); PUT_DL(2,2,t[4],t[5],t[6]); PUT_DL(1,3,t[4],t[5],t[6]); PUT_DL(0,4,t[4],t[5],t[6]);
    PUT_DL(5,0,t[5],t[6],t[7]); PUT_DL(4,1,t[5],t[6],t[7]); PUT_DL(3,2,t[5],t[6],t[7]); PUT_DL(2,3,t[5],t[6],t[7]); PUT_DL(1,4,t[5],t[6],t[7]); PUT_DL(0,5,t[5],t[6],t[7]);
    PUT_DL(6,0,t[6],t[7],t[8]); PUT_DL(5,1,t[6],t[7],t[8]); PUT_DL(4,2,t[6],t[7],t[8]); PUT_DL(3,3,t[6],t[7],t[8]); PUT_DL(2,4,t[6],t[7],t[8]); PUT_DL(1,5,t[6],t[7],t[8]); PUT_DL(0,6,t[6],t[7],t[8]);
    PUT_DL(7,0,t[7],t[8],t[9]); PUT_DL(6,1,t[7],t[8],t[9]); PUT_DL(5,2,t[7],t[8],t[9]); PUT_DL(4,3,t[7],t[8],t[9]); PUT_DL(3,4,t[7],t[8],t[9]); PUT_DL(2,5,t[7],t[8],t[9]); PUT_DL(1,6,t[7],t[8],t[9]); PUT_DL(0,7,t[7],t[8],t[9]);
    PUT_DL(7,1,t[8],t[9],t[10]); PUT_DL(6,2,t[8],t[9],t[10]); PUT_DL(5,3,t[8],t[9],t[10]); PUT_DL(4,4,t[8],t[9],t[10]); PUT_DL(3,5,t[8],t[9],t[10]); PUT_DL(2,6,t[8],t[9],t[10]); PUT_DL(1,7,t[8],t[9],t[10]);
    PUT_DL(7,2,t[9],t[10],t[11]); PUT_DL(6,3,t[9],t[10],t[11]); PUT_DL(5,4,t[9],t[10],t[11]); PUT_DL(4,5,t[9],t[10],t[11]); PUT_DL(3,6,t[9],t[10],t[11]); PUT_DL(2,7,t[9],t[10],t[11]);
    PUT_DL(7,3,t[10],t[11],t[12]); PUT_DL(6,4,t[10],t[11],t[12]); PUT_DL(5,5,t[10],t[11],t[12]); PUT_DL(4,6,t[10],t[11],t[12]); PUT_DL(3,7,t[10],t[11],t[12]);
    PUT_DL(7,4,t[11],t[12],t[13]); PUT_DL(6,5,t[11],t[12],t[13]); PUT_DL(5,6,t[11],t[12],t[13]); PUT_DL(4,7,t[11],t[12],t[13]);
    PUT_DL(7,5,t[12],t[13],t[14]); PUT_DL(6,6,t[12],t[13],t[14]); PUT_DL(5,7,t[12],t[13],t[14]);
    PUT_DL(7,6,t[13],t[14],t[15]); PUT_DL(6,7,t[13],t[14],t[15]);
    SRC8(7,7) = (uint16_t)((t[14] + 3 * t[15] + 2) >> 2);
#undef PUT_DL
}

static void pred8x8l_down_right(uint16_t *src, int has_topleft, int has_topright, int stride)
{
    int t[8], l[8], i, lt;
    t[0] = ((has_topleft ? src[-1 - stride] : src[-stride]) + 2 * src[-stride] + src[1 - stride] + 2) >> 2;
    for (i = 1; i <= 6; i++)
        t[i] = (src[i - 1 - stride] + 2 * src[i - stride] + src[i + 1 - stride] + 2) >> 2;
    t[7] = ((has_topright ? src[8 - stride] : src[7 - stride]) + 2 * src[7 - stride] + src[6 - stride] + 2) >> 2;
    l[0] = ((has_topleft ? src[-1 - stride] : src[-1]) + 2 * src[-1] + src[-1 + stride] + 2) >> 2;
    for (i = 1; i <= 6; i++)
        l[i] = (src[-1 + (i - 1) * stride] + 2 * src[-1 + i * stride] + src[-1 + (i + 1) * stride] + 2) >> 2;
    l[7] = (src[-1 + 6 * stride] + 3 * src[-1 + 7 * stride] + 2) >> 2;
    lt = (src[-1] + 2 * src[-1 - stride] + src[-stride] + 2) >> 2;

    SRC8(0,7) = (l[7] + 2 * l[6] + l[5] + 2) >> 2;
    SRC8(0,6) = SRC8(1,7) = (l[6] + 2 * l[5] + l[4] + 2) >> 2;
    SRC8(0,5) = SRC8(1,6) = SRC8(2,7) = (l[5] + 2 * l[4] + l[3] + 2) >> 2;
    SRC8(0,4) = SRC8(1,5) = SRC8(2,6) = SRC8(3,7) = (l[4] + 2 * l[3] + l[2] + 2) >> 2;
    SRC8(0,3) = SRC8(1,4) = SRC8(2,5) = SRC8(3,6) = SRC8(4,7) = (l[3] + 2 * l[2] + l[1] + 2) >> 2;
    SRC8(0,2) = SRC8(1,3) = SRC8(2,4) = SRC8(3,5) = SRC8(4,6) = SRC8(5,7) = (l[2] + 2 * l[1] + l[0] + 2) >> 2;
    SRC8(0,1) = SRC8(1,2) = SRC8(2,3) = SRC8(3,4) = SRC8(4,5) = SRC8(5,6) = SRC8(6,7) = (l[1] + 2 * l[0] + lt + 2) >> 2;
    SRC8(0,0) = SRC8(1,1) = SRC8(2,2) = SRC8(3,3) = SRC8(4,4) = SRC8(5,5) = SRC8(6,6) = SRC8(7,7) = (l[0] + 2 * lt + t[0] + 2) >> 2;
    SRC8(1,0) = SRC8(2,1) = SRC8(3,2) = SRC8(4,3) = SRC8(5,4) = SRC8(6,5) = SRC8(7,6) = (lt + 2 * t[0] + t[1] + 2) >> 2;
    SRC8(2,0) = SRC8(3,1) = SRC8(4,2) = SRC8(5,3) = SRC8(6,4) = SRC8(7,5) = (t[0] + 2 * t[1] + t[2] + 2) >> 2;
    SRC8(3,0) = SRC8(4,1) = SRC8(5,2) = SRC8(6,3) = SRC8(7,4) = (t[1] + 2 * t[2] + t[3] + 2) >> 2;
    SRC8(4,0) = SRC8(5,1) = SRC8(6,2) = SRC8(7,3) = (t[2] + 2 * t[3] + t[4] + 2) >> 2;
    SRC8(5,0) = SRC8(6,1) = SRC8(7,2) = (t[3] + 2 * t[4] + t[5] + 2) >> 2;
    SRC8(6,0) = SRC8(7,1) = (t[4] + 2 * t[5] + t[6] + 2) >> 2;
    SRC8(7,0) = (t[5] + 2 * t[6] + t[7] + 2) >> 2;
}

static void pred8x8l_vertical_right(uint16_t *src, int has_topleft, int has_topright, int stride)
{
    int t[8], l[8], i, lt;
    t[0] = ((has_topleft ? src[-1 - stride] : src[-stride]) + 2 * src[-stride] + src[1 - stride] + 2) >> 2;
    for (i = 1; i <= 6; i++)
        t[i] = (src[i - 1 - stride] + 2 * src[i - stride] + src[i + 1 - stride] + 2) >> 2;
    t[7] = ((has_topright ? src[8 - stride] : src[7 - stride]) + 2 * src[7 - stride] + src[6 - stride] + 2) >> 2;
    l[0] = ((has_topleft ? src[-1 - stride] : src[-1]) + 2 * src[-1] + src[-1 + stride] + 2) >> 2;
    for (i = 1; i <= 6; i++)
        l[i] = (src[-1 + (i - 1) * stride] + 2 * src[-1 + i * stride] + src[-1 + (i + 1) * stride] + 2) >> 2;
    l[7] = (src[-1 + 6 * stride] + 3 * src[-1 + 7 * stride] + 2) >> 2;
    lt = (src[-1] + 2 * src[-1 - stride] + src[-stride] + 2) >> 2;

    SRC8(0,6) = (l[5] + 2 * l[4] + l[3] + 2) >> 2;
    SRC8(0,7) = (l[6] + 2 * l[5] + l[4] + 2) >> 2;
    SRC8(0,4) = SRC8(1,6) = (l[3] + 2 * l[2] + l[1] + 2) >> 2;
    SRC8(0,5) = SRC8(1,7) = (l[4] + 2 * l[3] + l[2] + 2) >> 2;
    SRC8(0,2) = SRC8(1,4) = SRC8(2,6) = (l[1] + 2 * l[0] + lt + 2) >> 2;
    SRC8(0,3) = SRC8(1,5) = SRC8(2,7) = (l[2] + 2 * l[1] + l[0] + 2) >> 2;
    SRC8(0,1) = SRC8(1,3) = SRC8(2,5) = SRC8(3,7) = (l[0] + 2 * lt + t[0] + 2) >> 2;
    SRC8(0,0) = SRC8(1,2) = SRC8(2,4) = SRC8(3,6) = (lt + t[0] + 1) >> 1;
    SRC8(1,1) = SRC8(2,3) = SRC8(3,5) = SRC8(4,7) = (lt + 2 * t[0] + t[1] + 2) >> 2;
    SRC8(1,0) = SRC8(2,2) = SRC8(3,4) = SRC8(4,6) = (t[0] + t[1] + 1) >> 1;
    SRC8(2,1) = SRC8(3,3) = SRC8(4,5) = SRC8(5,7) = (t[0] + 2 * t[1] + t[2] + 2) >> 2;
    SRC8(2,0) = SRC8(3,2) = SRC8(4,4) = SRC8(5,6) = (t[1] + t[2] + 1) >> 1;
    SRC8(3,1) = SRC8(4,3) = SRC8(5,5) = SRC8(6,7) = (t[1] + 2 * t[2] + t[3] + 2) >> 2;
    SRC8(3,0) = SRC8(4,2) = SRC8(5,4) = SRC8(6,6) = (t[2] + t[3] + 1) >> 1;
    SRC8(4,1) = SRC8(5,3) = SRC8(6,5) = SRC8(7,7) = (t[2] + 2 * t[3] + t[4] + 2) >> 2;
    SRC8(4,0) = SRC8(5,2) = SRC8(6,4) = SRC8(7,6) = (t[3] + t[4] + 1) >> 1;
    SRC8(5,1) = SRC8(6,3) = SRC8(7,5) = (t[3] + 2 * t[4] + t[5] + 2) >> 2;
    SRC8(5,0) = SRC8(6,2) = SRC8(7,4) = (t[4] + t[5] + 1) >> 1;
    SRC8(6,1) = SRC8(7,3) = (t[4] + 2 * t[5] + t[6] + 2) >> 2;
    SRC8(6,0) = SRC8(7,2) = (t[5] + t[6] + 1) >> 1;
    SRC8(7,1) = (t[5] + 2 * t[6] + t[7] + 2) >> 2;
    SRC8(7,0) = (t[6] + t[7] + 1) >> 1;
}

static void pred8x8l_horizontal_down(uint16_t *src, int has_topleft, int has_topright, int stride)
{
    int t[8], l[8], i, lt;
    t[0] = ((has_topleft ? src[-1 - stride] : src[-stride]) + 2 * src[-stride] + src[1 - stride] + 2) >> 2;
    for (i = 1; i <= 6; i++)
        t[i] = (src[i - 1 - stride] + 2 * src[i - stride] + src[i + 1 - stride] + 2) >> 2;
    t[7] = ((has_topright ? src[8 - stride] : src[7 - stride]) + 2 * src[7 - stride] + src[6 - stride] + 2) >> 2;
    l[0] = ((has_topleft ? src[-1 - stride] : src[-1]) + 2 * src[-1] + src[-1 + stride] + 2) >> 2;
    for (i = 1; i <= 6; i++)
        l[i] = (src[-1 + (i - 1) * stride] + 2 * src[-1 + i * stride] + src[-1 + (i + 1) * stride] + 2) >> 2;
    l[7] = (src[-1 + 6 * stride] + 3 * src[-1 + 7 * stride] + 2) >> 2;
    lt = (src[-1] + 2 * src[-1 - stride] + src[-stride] + 2) >> 2;

    SRC8(0,7) = (l[6] + l[7] + 1) >> 1;
    SRC8(1,7) = (l[5] + 2 * l[6] + l[7] + 2) >> 2;
    SRC8(0,6) = SRC8(2,7) = (l[5] + l[6] + 1) >> 1;
    SRC8(1,6) = SRC8(3,7) = (l[4] + 2 * l[5] + l[6] + 2) >> 2;
    SRC8(0,5) = SRC8(2,6) = SRC8(4,7) = (l[4] + l[5] + 1) >> 1;
    SRC8(1,5) = SRC8(3,6) = SRC8(5,7) = (l[3] + 2 * l[4] + l[5] + 2) >> 2;
    SRC8(0,4) = SRC8(2,5) = SRC8(4,6) = SRC8(6,7) = (l[3] + l[4] + 1) >> 1;
    SRC8(1,4) = SRC8(3,5) = SRC8(5,6) = SRC8(7,7) = (l[2] + 2 * l[3] + l[4] + 2) >> 2;
    SRC8(0,3) = SRC8(2,4) = SRC8(4,5) = SRC8(6,6) = (l[2] + l[3] + 1) >> 1;
    SRC8(1,3) = SRC8(3,4) = SRC8(5,5) = SRC8(7,6) = (l[1] + 2 * l[2] + l[3] + 2) >> 2;
    SRC8(0,2) = SRC8(2,3) = SRC8(4,4) = SRC8(6,5) = (l[1] + l[2] + 1) >> 1;
    SRC8(1,2) = SRC8(3,3) = SRC8(5,4) = SRC8(7,5) = (l[0] + 2 * l[1] + l[2] + 2) >> 2;
    SRC8(0,1) = SRC8(2,2) = SRC8(4,3) = SRC8(6,4) = (l[0] + l[1] + 1) >> 1;
    SRC8(1,1) = SRC8(3,2) = SRC8(5,3) = SRC8(7,4) = (lt + 2 * l[0] + l[1] + 2) >> 2;
    SRC8(0,0) = SRC8(2,1) = SRC8(4,2) = SRC8(6,3) = (lt + l[0] + 1) >> 1;
    SRC8(1,0) = SRC8(3,1) = SRC8(5,2) = SRC8(7,3) = (l[0] + 2 * lt + t[0] + 2) >> 2;
    SRC8(2,0) = SRC8(4,1) = SRC8(6,2) = (t[1] + 2 * t[0] + lt + 2) >> 2;
    SRC8(3,0) = SRC8(5,1) = SRC8(7,2) = (t[2] + 2 * t[1] + t[0] + 2) >> 2;
    SRC8(4,0) = SRC8(6,1) = (t[3] + 2 * t[2] + t[1] + 2) >> 2;
    SRC8(5,0) = SRC8(7,1) = (t[4] + 2 * t[3] + t[2] + 2) >> 2;
    SRC8(6,0) = (t[5] + 2 * t[4] + t[3] + 2) >> 2;
    SRC8(7,0) = (t[6] + 2 * t[5] + t[4] + 2) >> 2;
}

static void pred8x8l_vertical_left(uint16_t *src, int has_topleft, int has_topright, int stride)
{
    int t[16], i;
    t[0] = ((has_topleft ? src[-1 - stride] : src[-stride]) + 2 * src[-stride] + src[1 - stride] + 2) >> 2;
    for (i = 1; i <= 6; i++)
        t[i] = (src[i - 1 - stride] + 2 * src[i - stride] + src[i + 1 - stride] + 2) >> 2;
    t[7] = ((has_topright ? src[8 - stride] : src[7 - stride]) + 2 * src[7 - stride] + src[6 - stride] + 2) >> 2;
    if (has_topright) {
        for (i = 8; i <= 14; i++)
            t[i] = (src[i - 1 - stride] + 2 * src[i - stride] + src[i + 1 - stride] + 2) >> 2;
        t[15] = (src[14 - stride] + 3 * src[15 - stride] + 2) >> 2;
    } else {
        for (i = 8; i < 16; i++)
            t[i] = src[7 - stride];
    }

    SRC8(0,0) = (t[0] + t[1] + 1) >> 1;
    SRC8(0,1) = (t[0] + 2 * t[1] + t[2] + 2) >> 2;
    SRC8(0,2) = SRC8(1,0) = (t[1] + t[2] + 1) >> 1;
    SRC8(0,3) = SRC8(1,1) = (t[1] + 2 * t[2] + t[3] + 2) >> 2;
    SRC8(0,4) = SRC8(1,2) = SRC8(2,0) = (t[2] + t[3] + 1) >> 1;
    SRC8(0,5) = SRC8(1,3) = SRC8(2,1) = (t[2] + 2 * t[3] + t[4] + 2) >> 2;
    SRC8(0,6) = SRC8(1,4) = SRC8(2,2) = SRC8(3,0) = (t[3] + t[4] + 1) >> 1;
    SRC8(0,7) = SRC8(1,5) = SRC8(2,3) = SRC8(3,1) = (t[3] + 2 * t[4] + t[5] + 2) >> 2;
    SRC8(1,6) = SRC8(2,4) = SRC8(3,2) = SRC8(4,0) = (t[4] + t[5] + 1) >> 1;
    SRC8(1,7) = SRC8(2,5) = SRC8(3,3) = SRC8(4,1) = (t[4] + 2 * t[5] + t[6] + 2) >> 2;
    SRC8(2,6) = SRC8(3,4) = SRC8(4,2) = SRC8(5,0) = (t[5] + t[6] + 1) >> 1;
    SRC8(2,7) = SRC8(3,5) = SRC8(4,3) = SRC8(5,1) = (t[5] + 2 * t[6] + t[7] + 2) >> 2;
    SRC8(3,6) = SRC8(4,4) = SRC8(5,2) = SRC8(6,0) = (t[6] + t[7] + 1) >> 1;
    SRC8(3,7) = SRC8(4,5) = SRC8(5,3) = SRC8(6,1) = (t[6] + 2 * t[7] + t[8] + 2) >> 2;
    SRC8(4,6) = SRC8(5,4) = SRC8(6,2) = SRC8(7,0) = (t[7] + t[8] + 1) >> 1;
    SRC8(4,7) = SRC8(5,5) = SRC8(6,3) = SRC8(7,1) = (t[7] + 2 * t[8] + t[9] + 2) >> 2;
    SRC8(5,6) = SRC8(6,4) = SRC8(7,2) = (t[8] + t[9] + 1) >> 1;
    SRC8(5,7) = SRC8(6,5) = SRC8(7,3) = (t[8] + 2 * t[9] + t[10] + 2) >> 2;
    SRC8(6,6) = SRC8(7,4) = (t[9] + t[10] + 1) >> 1;
    SRC8(6,7) = SRC8(7,5) = (t[9] + 2 * t[10] + t[11] + 2) >> 2;
    SRC8(7,6) = (t[10] + t[11] + 1) >> 1;
    SRC8(7,7) = (t[10] + 2 * t[11] + t[12] + 2) >> 2;
}

static void pred8x8l_horizontal_up(uint16_t *src, int has_topleft, int has_topright, int stride)
{
    int l[8], i;
    l[0] = ((has_topleft ? src[-1 - stride] : src[-1]) + 2 * src[-1] + src[-1 + stride] + 2) >> 2;
    for (i = 1; i <= 6; i++)
        l[i] = (src[-1 + (i - 1) * stride] + 2 * src[-1 + i * stride] + src[-1 + (i + 1) * stride] + 2) >> 2;
    l[7] = (src[-1 + 6 * stride] + 3 * src[-1 + 7 * stride] + 2) >> 2;

    SRC8(0,0) = (l[0] + l[1] + 1) >> 1;
    SRC8(1,0) = (l[0] + 2 * l[1] + l[2] + 2) >> 2;
    SRC8(0,1) = SRC8(2,0) = (l[1] + l[2] + 1) >> 1;
    SRC8(1,1) = SRC8(3,0) = (l[1] + 2 * l[2] + l[3] + 2) >> 2;
    SRC8(0,2) = SRC8(2,1) = SRC8(4,0) = (l[2] + l[3] + 1) >> 1;
    SRC8(1,2) = SRC8(3,1) = SRC8(5,0) = (l[2] + 2 * l[3] + l[4] + 2) >> 2;
    SRC8(0,3) = SRC8(2,2) = SRC8(4,1) = SRC8(6,0) = (l[3] + l[4] + 1) >> 1;
    SRC8(1,3) = SRC8(3,2) = SRC8(5,1) = SRC8(7,0) = (l[3] + 2 * l[4] + l[5] + 2) >> 2;
    SRC8(0,4) = SRC8(2,3) = SRC8(4,2) = SRC8(6,1) = (l[4] + l[5] + 1) >> 1;
    SRC8(1,4) = SRC8(3,3) = SRC8(5,2) = SRC8(7,1) = (l[4] + 2 * l[5] + l[6] + 2) >> 2;
    SRC8(0,5) = SRC8(2,4) = SRC8(4,3) = SRC8(6,2) = (l[5] + l[6] + 1) >> 1;
    SRC8(1,5) = SRC8(3,4) = SRC8(5,3) = SRC8(7,2) = (l[5] + 2 * l[6] + l[7] + 2) >> 2;
    SRC8(0,6) = SRC8(2,5) = SRC8(4,4) = SRC8(6,3) = (l[6] + l[7] + 1) >> 1;
    SRC8(1,6) = SRC8(3,5) = SRC8(5,4) = SRC8(7,3) = (l[6] + 3 * l[7] + 2) >> 2;
    SRC8(0,7) = SRC8(1,7) = SRC8(2,6) = SRC8(2,7) = SRC8(3,6) =
    SRC8(3,7) = SRC8(4,5) = SRC8(4,6) = SRC8(4,7) = SRC8(5,5) =
    SRC8(5,6) = SRC8(5,7) = SRC8(6,4) = SRC8(6,5) = SRC8(6,6) =
    SRC8(6,7) = SRC8(7,4) = SRC8(7,5) = SRC8(7,6) = SRC8(7,7) = (uint16_t)l[7];
}

/* ---------------- 8x8 chroma / 16x16 luma ---------------- */
static void pred8x8_vertical(uint16_t *src, int stride)
{
    int i;
    for (i = 0; i < 8; i++)
        memcpy(src + i * stride, src - stride, 16);
}

static void pred8x8_horizontal(uint16_t *src, int stride)
{
    int i;
    for (i = 0; i < 8; i++)
        fill_px(src + i * stride, src[-1 + i * stride], 8);
}

static void pred8x8_dc(uint16_t *src, int stride)
{
    int i;
    int dc0 = 0, dc1 = 0, dc2 = 0, dc3;
    for (i = 0; i < 4; i++) {
        dc0 += src[-1 + i * stride] + src[i - stride];
        dc1 += src[4 + i - stride];
        dc2 += src[-1 + (i + 4) * stride];
    }
    dc3 = (dc1 + dc2 + 4) >> 3;
    dc0 = (dc0 + 4) >> 3;
    dc1 = (dc1 + 2) >> 2;
    dc2 = (dc2 + 2) >> 2;
    for (i = 0; i < 4; i++) {
        fill_px(src + i * stride, dc0, 4);
        fill_px(src + i * stride + 4, dc1, 4);
    }
    for (i = 4; i < 8; i++) {
        fill_px(src + i * stride, dc2, 4);
        fill_px(src + i * stride + 4, dc3, 4);
    }
}

static void pred8x8_left_dc(uint16_t *src, int stride)
{
    int i, dc0 = 0, dc2 = 0;
    for (i = 0; i < 4; i++) {
        dc0 += src[-1 + i * stride];
        dc2 += src[-1 + (i + 4) * stride];
    }
    dc0 = (dc0 + 2) >> 2;
    dc2 = (dc2 + 2) >> 2;
    for (i = 0; i < 4; i++)
        fill_px(src + i * stride, dc0, 8);
    for (i = 4; i < 8; i++)
        fill_px(src + i * stride, dc2, 8);
}

static void pred8x8_top_dc(uint16_t *src, int stride)
{
    int i, dc0 = 0, dc1 = 0;
    for (i = 0; i < 4; i++) {
        dc0 += src[i - stride];
        dc1 += src[4 + i - stride];
    }
    dc0 = (dc0 + 2) >> 2;
    dc1 = (dc1 + 2) >> 2;
    for (i = 0; i < 8; i++) {
        fill_px(src + i * stride, dc0, 4);
        fill_px(src + i * stride + 4, dc1, 4);
    }
}

static void pred8x8_128_dc(uint16_t *src, int stride)
{
    int i;
    for (i = 0; i < 8; i++)
        fill_px(src + i * stride, 512, 8);
}

static void pred8x8_plane(uint16_t *src, int stride)
{
    int j, k, a;
    const uint16_t *src0 = src + 3 - stride;
    const uint16_t *src1 = src + 4 * stride - 1;
    const uint16_t *src2 = src1 - 2 * stride;
    int H = src0[1] - src0[-1];
    int V = src1[0] - src2[0];
    for (k = 2; k <= 4; k++) {
        src1 += stride;
        src2 -= stride;
        H += k * (src0[k] - src0[-k]);
        V += k * (src1[0] - src2[0]);
    }
    H = (17 * H + 16) >> 5;
    V = (17 * V + 16) >> 5;
    a = 16 * (src1[0] + src2[8] + 1) - 3 * (V + H);
    for (j = 0; j < 8; j++) {
        int b = a;
        a += V;
        for (k = 0; k < 8; k++)
            src[k] = clip10((b + k * H) >> 5);
        src += stride;
    }
}

/* ---------------- 8x16 chroma (4:2:2) ---------------- */
static void pred8x16_vertical(uint16_t *src, int stride)
{
    int i;
    for (i = 0; i < 16; i++)
        memcpy(src + i * stride, src - stride, 16);
}

static void pred8x16_horizontal(uint16_t *src, int stride)
{
    int i;
    for (i = 0; i < 16; i++)
        fill_px(src + i * stride, src[-1 + i * stride], 8);
}

static void pred8x16_dc(uint16_t *src, int stride)
{
    int i;
    int dc0 = 0, dc1 = 0, dc2 = 0, dc3 = 0, dc4 = 0;
    int p0, p1, p2, p3, p4, p5, p6, p7;

    for (i = 0; i < 4; i++) {
        dc0 += src[-1 + i * stride] + src[i - stride];
        dc1 += src[4 + i - stride];
        dc2 += src[-1 + (i + 4) * stride];
        dc3 += src[-1 + (i + 8) * stride];
        dc4 += src[-1 + (i + 12) * stride];
    }
    p0 = (dc0 + 4) >> 3;
    p1 = (dc1 + 2) >> 2;
    p2 = (dc2 + 2) >> 2;
    p3 = (dc1 + dc2 + 4) >> 3;
    p4 = (dc3 + 2) >> 2;
    p5 = (dc1 + dc3 + 4) >> 3;
    p6 = (dc4 + 2) >> 2;
    p7 = (dc1 + dc4 + 4) >> 3;
    for (i = 0; i < 4; i++) {
        fill_px(src + i * stride, p0, 4);
        fill_px(src + i * stride + 4, p1, 4);
    }
    for (i = 4; i < 8; i++) {
        fill_px(src + i * stride, p2, 4);
        fill_px(src + i * stride + 4, p3, 4);
    }
    for (i = 8; i < 12; i++) {
        fill_px(src + i * stride, p4, 4);
        fill_px(src + i * stride + 4, p5, 4);
    }
    for (i = 12; i < 16; i++) {
        fill_px(src + i * stride, p6, 4);
        fill_px(src + i * stride + 4, p7, 4);
    }
}

static void pred8x16_left_dc(uint16_t *src, int stride)
{
    int i, half, dc0, dc1;
    for (half = 0; half < 2; half++, src += 8 * stride) {
        dc0 = dc1 = 0;
        for (i = 0; i < 4; i++) {
            dc0 += src[-1 + i * stride];
            dc1 += src[-1 + (i + 4) * stride];
        }
        dc0 = (dc0 + 2) >> 2;
        dc1 = (dc1 + 2) >> 2;
        for (i = 0; i < 4; i++)
            fill_px(src + i * stride, dc0, 8);
        for (i = 4; i < 8; i++)
            fill_px(src + i * stride, dc1, 8);
    }
}

static void pred8x16_top_dc(uint16_t *src, int stride)
{
    int i, dc0 = 0, dc1 = 0;
    for (i = 0; i < 4; i++) {
        dc0 += src[i - stride];
        dc1 += src[4 + i - stride];
    }
    dc0 = (dc0 + 2) >> 2;
    dc1 = (dc1 + 2) >> 2;
    for (i = 0; i < 16; i++) {
        fill_px(src + i * stride, dc0, 4);
        fill_px(src + i * stride + 4, dc1, 4);
    }
}

static void pred8x16_128_dc(uint16_t *src, int stride)
{
    int i;
    for (i = 0; i < 16; i++)
        fill_px(src + i * stride, 512, 8);
}

static void pred8x16_plane(uint16_t *src, int stride)
{
    int j, k, a;
    const uint16_t *src0 = src + 3 - stride;
    const uint16_t *src1 = src + 8 * stride - 1;
    const uint16_t *src2 = src1 - 2 * stride;
    int H = src0[1] - src0[-1];
    int V = src1[0] - src2[0];

    for (k = 2; k <= 4; k++) {
        src1 += stride;
        src2 -= stride;
        H += k * (src0[k] - src0[-k]);
        V += k * (src1[0] - src2[0]);
    }
    for (; k <= 8; k++) {
        src1 += stride;
        src2 -= stride;
        V += k * (src1[0] - src2[0]);
    }
    H = (17 * H + 16) >> 5;
    V = (5 * V + 32) >> 6;
    a = 16 * (src1[0] + src2[8] + 1) - 7 * V - 3 * H;
    for (j = 0; j < 16; j++) {
        int b = a;
        a += V;
        for (k = 0; k < 8; k++)
            src[k] = clip10((b + k * H) >> 5);
        src += stride;
    }
}

static void pred16x16_vertical(uint16_t *src, int stride)
{
    int i;
    for (i = 0; i < 16; i++)
        memcpy(src + i * stride, src - stride, 32);
}

static void pred16x16_horizontal(uint16_t *src, int stride)
{
    int i;
    for (i = 0; i < 16; i++)
        fill_px(src + i * stride, src[-1 + i * stride], 16);
}

static void pred16x16_dc(uint16_t *src, int stride)
{
    int i, dc = 0;
    for (i = 0; i < 16; i++)
        dc += src[-1 + i * stride] + src[i - stride];
    dc = (dc + 16) >> 5;
    for (i = 0; i < 16; i++)
        fill_px(src + i * stride, dc, 16);
}

static void pred16x16_left_dc(uint16_t *src, int stride)
{
    int i, dc = 0;
    for (i = 0; i < 16; i++)
        dc += src[-1 + i * stride];
    dc = (dc + 8) >> 4;
    for (i = 0; i < 16; i++)
        fill_px(src + i * stride, dc, 16);
}

static void pred16x16_top_dc(uint16_t *src, int stride)
{
    int i, dc = 0;
    for (i = 0; i < 16; i++)
        dc += src[i - stride];
    dc = (dc + 8) >> 4;
    for (i = 0; i < 16; i++)
        fill_px(src + i * stride, dc, 16);
}

static void pred16x16_128_dc(uint16_t *src, int stride)
{
    int i;
    for (i = 0; i < 16; i++)
        fill_px(src + i * stride, 512, 16);
}

static void pred16x16_plane(uint16_t *src, int stride)
{
    int i, j, k;
    const uint16_t *src0 = src + 7 - stride;
    const uint16_t *src1 = src + 8 * stride - 1;
    const uint16_t *src2 = src1 - 2 * stride;
    int H = src0[1] - src0[-1];
    int V = src1[0] - src2[0];
    int a;
    for (k = 2; k <= 8; k++) {
        src1 += stride;
        src2 -= stride;
        H += k * (src0[k] - src0[-k]);
        V += k * (src1[0] - src2[0]);
    }
    H = (5 * H + 32) >> 6;
    V = (5 * V + 32) >> 6;
    a = 16 * (src1[0] + src2[16] + 1) - 7 * (V + H);
    for (j = 0; j < 16; j++) {
        int b = a;
        a += V;
        for (i = 0; i < 16; i++) {
            src[i] = clip10(b >> 5);
            b += H;
        }
        src += stride;
    }
}

/* ---------------- dispatch ---------------- */
void h264_pred4x4_dispatch_16(int mode, uint16_t *src, const uint16_t *topright, int stride)
{
    switch (mode) {
    case 0: pred4x4_vertical(src, topright, stride); break;
    case 1: pred4x4_horizontal(src, topright, stride); break;
    case 2: pred4x4_dc(src, topright, stride); break;
    case 3: pred4x4_down_left(src, topright, stride); break;
    case 4: pred4x4_down_right(src, topright, stride); break;
    case 5: pred4x4_vertical_right(src, topright, stride); break;
    case 6: pred4x4_horizontal_down(src, topright, stride); break;
    case 7: pred4x4_vertical_left(src, topright, stride); break;
    case 8: pred4x4_horizontal_up(src, topright, stride); break;
    case 9: pred4x4_left_dc(src, topright, stride); break;
    case 10: pred4x4_top_dc(src, topright, stride); break;
    default: pred4x4_128_dc(src, topright, stride); break;
    }
}

void h264_pred8x8l_dispatch_16(int mode, uint16_t *src, int has_topleft, int has_topright, int stride)
{
    switch (mode) {
    case 0: pred8x8l_vertical(src, has_topleft, has_topright, stride); break;
    case 1: pred8x8l_horizontal(src, has_topleft, has_topright, stride); break;
    case 2: pred8x8l_dc(src, has_topleft, has_topright, stride); break;
    case 3: pred8x8l_down_left(src, has_topleft, has_topright, stride); break;
    case 4: pred8x8l_down_right(src, has_topleft, has_topright, stride); break;
    case 5: pred8x8l_vertical_right(src, has_topleft, has_topright, stride); break;
    case 6: pred8x8l_horizontal_down(src, has_topleft, has_topright, stride); break;
    case 7: pred8x8l_vertical_left(src, has_topleft, has_topright, stride); break;
    case 8: pred8x8l_horizontal_up(src, has_topleft, has_topright, stride); break;
    case 9: pred8x8l_left_dc(src, has_topleft, has_topright, stride); break;
    case 10: pred8x8l_top_dc(src, has_topleft, has_topright, stride); break;
    default: pred8x8l_128_dc(src, has_topleft, has_topright, stride); break;
    }
}

void h264_pred8x8_dispatch_16(int mode, uint16_t *src, int stride)
{
    switch (mode) {
    case 0: pred8x8_dc(src, stride); break;
    case 1: pred8x8_horizontal(src, stride); break;
    case 2: pred8x8_vertical(src, stride); break;
    case 3: pred8x8_plane(src, stride); break;
    case 4: pred8x8_left_dc(src, stride); break;
    case 5: pred8x8_top_dc(src, stride); break;
    case 6: pred8x8_128_dc(src, stride); break;
    default:
        /* MBAFF constrained-intra "mad cow" modes */
        if (mode == 7) {          /* L0T: top DC everywhere, DC in top-left 4x4 */
            pred8x8_top_dc(src, stride);
            pred4x4_dc(src, NULL, stride);
        } else if (mode == 8) {   /* 0LT: full DC, top DC in top-left */
            pred8x8_dc(src, stride);
            pred4x4_top_dc(src, NULL, stride);
        } else if (mode == 9) {   /* L00: left DC, 128 in top-right and bottom-right */
            pred8x8_left_dc(src, stride);
            pred4x4_128_dc(src + 4, NULL, stride);
            pred4x4_128_dc(src + 4 + 4 * stride, NULL, stride);
        } else {                  /* 0L0: left DC, 128 in top-left and top-right */
            pred8x8_left_dc(src, stride);
            pred4x4_128_dc(src, NULL, stride);
            pred4x4_128_dc(src + 4, NULL, stride);
        }
        break;
    }
}

void h264_pred8x16_dispatch_16(int mode, uint16_t *src, int stride)
{
    switch (mode) {
    case 0: pred8x16_dc(src, stride); break;
    case 1: pred8x16_horizontal(src, stride); break;
    case 2: pred8x16_vertical(src, stride); break;
    case 3: pred8x16_plane(src, stride); break;
    case 4: pred8x16_left_dc(src, stride); break;
    case 5: pred8x16_top_dc(src, stride); break;
    case 6: pred8x16_128_dc(src, stride); break;
    default:
        if (mode == 7) {
            pred8x16_top_dc(src, stride);
            pred4x4_dc(src, NULL, stride);
        } else if (mode == 8) {
            pred8x16_dc(src, stride);
            pred4x4_top_dc(src, NULL, stride);
        } else if (mode == 9) {
            pred8x16_left_dc(src, stride);
            pred4x4_128_dc(src + 4, NULL, stride);
            pred4x4_128_dc(src + 4 + 4 * stride, NULL, stride);
        } else {
            pred8x16_left_dc(src, stride);
            pred4x4_128_dc(src, NULL, stride);
            pred4x4_128_dc(src + 4, NULL, stride);
        }
        break;
    }
}

void h264_pred16x16_dispatch_16(int mode, uint16_t *src, int stride)
{
    switch (mode) {
    case 0: pred16x16_dc(src, stride); break;
    case 1: pred16x16_horizontal(src, stride); break;
    case 2: pred16x16_vertical(src, stride); break;
    case 3: pred16x16_plane(src, stride); break;
    case 4: pred16x16_left_dc(src, stride); break;
    case 5: pred16x16_top_dc(src, stride); break;
    default: pred16x16_128_dc(src, stride); break;
    }
}

/* ---------------- availability checks ---------------- */
int h264_check_intra4x4_pred_mode_16(int8_t *pred_mode_cache, int top_samples_available,
                                  int left_samples_available)
{
    static const int8_t top[12] = {
        -1, 0, 9 /* LEFT_DC_PRED */, -1, -1, -1, -1, -1, 0
    };
    static const int8_t left[12] = {
        0, -1, 10 /* TOP_DC_PRED */, 0, -1, -1, -1, 0, -1, 11 /* DC_128_PRED */
    };
    int i;

    if (!(top_samples_available & 0x8000)) {
        for (i = 0; i < 4; i++) {
            int status = top[pred_mode_cache[h264_scan8[0] + i]];
            if (status < 0)
                return -1;
            if (status)
                pred_mode_cache[h264_scan8[0] + i] = (int8_t)status;
        }
    }
    if ((left_samples_available & 0x8888) != 0x8888) {
        static const int mask[4] = { 0x8000, 0x2000, 0x80, 0x20 };
        for (i = 0; i < 4; i++) {
            if (!(left_samples_available & mask[i])) {
                int status = left[pred_mode_cache[h264_scan8[0] + 8 * i]];
                if (status < 0)
                    return -1;
                if (status)
                    pred_mode_cache[h264_scan8[0] + 8 * i] = (int8_t)status;
            }
        }
    }
    return 0;
}

int h264_check_intra_pred_mode_16(int top_samples_available, int left_samples_available,
                               int mode, int is_chroma)
{
    static const int8_t top[4]  = { 4 /* LEFT_DC_PRED8x8 */, 1, -1, -1 };
    static const int8_t left[5] = { 5 /* TOP_DC_PRED8x8 */, -1, 2, -1, 6 /* DC_128_PRED8x8 */ };

    /* unsigned compare: a negative mode must fail the same way */
    if ((unsigned)mode > 3U)
        return -1;

    if (!(top_samples_available & 0x8000)) {
        mode = top[mode];
        if (mode < 0)
            return -1;
    }
    if ((left_samples_available & 0x8080) != 0x8080) {
        mode = left[mode];
        if (mode < 0)
            return -1;
        if (is_chroma && (left_samples_available & 0x8080)) {
            mode = 7 /* ALZHEIMER_DC_L0T */ +
                   (!(left_samples_available & 0x8000)) +
                   2 * (mode == 6 /* DC_128 */);
        }
    }
    return mode;
}

/* ---------------- lossless (transform_bypass) prediction + residual add ----------------
 * High 4:4:4 Predictive with x264 combined the modular residual add with the
 * vertical/horizontal prediction, and did so with the residual read in
 * column-major order; the reference decoder reproduces that, so we must too. */
static void pred4x4_vertical_add(uint16_t *pix, int32_t *block, int stride)
{
    int i;
    pix -= stride;
    for (i = 0; i < 4; i++) {
        uint16_t v = pix[0];
        pix[1 * stride] = v += block[0];
        pix[2 * stride] = v += block[4];
        pix[3 * stride] = v += block[8];
        pix[4 * stride] = v +  block[12];
        pix++;
        block++;
    }
    memset(block - 4, 0, 16 * sizeof(int32_t));
}

static void pred4x4_horizontal_add(uint16_t *pix, int32_t *block, int stride)
{
    int i;
    int32_t *orig = block;
    for (i = 0; i < 4; i++) {
        uint16_t v = pix[-1];
        pix[0] = v += block[0];
        pix[1] = v += block[1];
        pix[2] = v += block[2];
        pix[3] = v +  block[3];
        pix += stride;
        block += 4;
    }
    memset(orig, 0, 16 * sizeof(int32_t));
}

static void pred8x8l_vertical_add(uint16_t *pix, int32_t *block, int stride)
{
    int i;
    pix -= stride;
    for (i = 0; i < 8; i++) {
        uint16_t v = pix[0];
        pix[1 * stride] = v += block[0];
        pix[2 * stride] = v += block[8];
        pix[3 * stride] = v += block[16];
        pix[4 * stride] = v += block[24];
        pix[5 * stride] = v += block[32];
        pix[6 * stride] = v += block[40];
        pix[7 * stride] = v += block[48];
        pix[8 * stride] = v +  block[56];
        pix++;
        block++;
    }
    memset(block - 8, 0, 64 * sizeof(int32_t));
}

static void pred8x8l_horizontal_add(uint16_t *pix, int32_t *block, int stride)
{
    int i;
    int32_t *orig = block;
    for (i = 0; i < 8; i++) {
        uint16_t v = pix[-1];
        pix[0] = v += block[0];
        pix[1] = v += block[1];
        pix[2] = v += block[2];
        pix[3] = v += block[3];
        pix[4] = v += block[4];
        pix[5] = v += block[5];
        pix[6] = v += block[6];
        pix[7] = v +  block[7];
        pix += stride;
        block += 8;
    }
    memset(orig, 0, 64 * sizeof(int32_t));
}

static void pred8x8l_vertical_filter_add(uint16_t *src, int32_t *block, int has_topleft,
                                         int has_topright, int stride)
{
    uint16_t pix[8];
    int32_t *orig = block;
    int i;
    pix[0] = (uint16_t)(((has_topleft ? src[-1 - stride] : src[-stride])
                        + 2 * src[-stride] + src[1 - stride] + 2) >> 2);
    for (i = 1; i <= 6; i++)
        pix[i] = (uint16_t)((src[i - 1 - stride] + 2 * src[i - stride]
                            + src[i + 1 - stride] + 2) >> 2);
    pix[7] = (uint16_t)(((has_topright ? src[8 - stride] : src[7 - stride])
                        + 2 * src[7 - stride] + src[6 - stride] + 2) >> 2);
    for (i = 0; i < 8; i++) {
        uint16_t v = pix[i];
        src[0 * stride] = v += block[0];
        src[1 * stride] = v += block[8];
        src[2 * stride] = v += block[16];
        src[3 * stride] = v += block[24];
        src[4 * stride] = v += block[32];
        src[5 * stride] = v += block[40];
        src[6 * stride] = v += block[48];
        src[7 * stride] = v +  block[56];
        src++;
        block++;
    }
    memset(orig, 0, 64 * sizeof(int32_t));
}

static void pred8x8l_horizontal_filter_add(uint16_t *src, int32_t *block, int has_topleft,
                                           int has_topright, int stride)
{
    uint16_t pix[8];
    int32_t *orig = block;
    int i;
    pix[0] = (uint16_t)(((has_topleft ? src[-1 - stride] : src[-1])
                        + 2 * src[-1] + src[-1 + stride] + 2) >> 2);
    for (i = 1; i <= 6; i++)
        pix[i] = (uint16_t)((src[-1 + (i - 1) * stride] + 2 * src[-1 + i * stride]
                            + src[-1 + (i + 1) * stride] + 2) >> 2);
    pix[7] = (uint16_t)((src[-1 + 6 * stride] + 3 * src[-1 + 7 * stride] + 2) >> 2);
    for (i = 0; i < 8; i++) {
        uint16_t v = pix[i];
        src[0] = v += block[0];
        src[1] = v += block[1];
        src[2] = v += block[2];
        src[3] = v += block[3];
        src[4] = v += block[4];
        src[5] = v += block[5];
        src[6] = v += block[6];
        src[7] = v +  block[7];
        src += stride;
        block += 8;
    }
    memset(orig, 0, 64 * sizeof(int32_t));
}

void h264_pred4x4_add_16(int mode, uint16_t *pix, int32_t *block, int stride)
{
    if (mode == 0)
        pred4x4_vertical_add(pix, block, stride);
    else
        pred4x4_horizontal_add(pix, block, stride);
}

void h264_pred8x8l_add_16(int mode, uint16_t *pix, int32_t *block, int stride)
{
    if (mode == 0)
        pred8x8l_vertical_add(pix, block, stride);
    else
        pred8x8l_horizontal_add(pix, block, stride);
}

void h264_pred8x8l_filter_add_16(int mode, uint16_t *src, int32_t *block,
                              int has_topleft, int has_topright, int stride)
{
    if (mode == 0)
        pred8x8l_vertical_filter_add(src, block, has_topleft, has_topright, stride);
    else
        pred8x8l_horizontal_filter_add(src, block, has_topleft, has_topright, stride);
}

/* the 16x16 and chroma tables are indexed with the chroma/8x8 mode numbers
 * (DC=0, HOR=1, VERT=2), exactly as the reference does */
void h264_pred16x16_add_16(int mode, uint16_t *pix, const int *block_offset,
                       int32_t *block, int stride)
{
    int i;
    for (i = 0; i < 16; i++)
        h264_pred4x4_add_16(mode == 2 ? 0 : 1, pix + block_offset[i],
                         block + i * 16, stride);
}

void h264_pred8x8_add_16(int mode, uint16_t *pix, const int *block_offset,
                      int32_t *block, int stride)
{
    int i;
    for (i = 0; i < 4; i++)
        h264_pred4x4_add_16(mode == 2 ? 0 : 1, pix + block_offset[i],
                         block + i * 16, stride);
}
