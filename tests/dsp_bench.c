/*
 * Microbenchmark for the vector kernels: compares each SSE2 entry point
 * against the scalar one over a large block of data.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "h264_internal.h"

#define FIELD 256
#define OFF   16
#define ITER  4000

/* one deblocked MB row: 16 positions of a 4x4 edge group need 16 samples, and
 * the walk below moves one row per call */
#define DB_STRIDE 64
#define DB_ROWS   1024
#define DB_ITER   2000000

static uint8_t src[FIELD * FIELD];
static uint8_t dst_a[FIELD * FIELD], dst_b[FIELD * FIELD];
static uint8_t db_base[DB_STRIDE * DB_ROWS];

static double now_ms(void)
{
    return (double)clock() * 1000.0 / CLOCKS_PER_SEC;
}

typedef void (*mc_fn)(uint8_t *, const uint8_t *, int, int, int, int, int, int);

static double bench(mc_fn f, int w, int h, int x, int y, int avg)
{
    int i;
    double t0, t1;
    t0 = now_ms();
    for (i = 0; i < ITER; i++) {
        uint8_t *d = dst_a + OFF * FIELD + OFF + (i & 15) * FIELD;
        const uint8_t *s = src + OFF * FIELD + OFF + (i & 31) * FIELD;
        f(d, s, FIELD, w, h, x, y, avg);
    }
    t1 = now_ms();
    return t1 - t0;
}

/* the four deblocking kernels share two signatures */
typedef void (*lf_fn)(uint8_t *, int, int, int, int, int, const int8_t *);
typedef void (*lf_intra_fn)(uint8_t *, int, int, int, int, int);

static void h264_lf_luma_c(uint8_t *p, int xs, int ys, int it, int a, int b,
                           const int8_t *tc) { h264_loop_filter_luma_c(p, xs, ys, it, a, b, tc); }
static void h264_lf_luma_sse2(uint8_t *p, int xs, int ys, int it, int a, int b,
                              const int8_t *tc) { h264_loop_filter_luma_sse2(p, xs, ys, it, a, b, tc); }
static void h264_lf_chroma_c(uint8_t *p, int xs, int ys, int it, int a, int b,
                             const int8_t *tc) { h264_loop_filter_chroma_c(p, xs, ys, it, a, b, tc); }
static void h264_lf_chroma_sse2(uint8_t *p, int xs, int ys, int it, int a, int b,
                                const int8_t *tc) { h264_loop_filter_chroma_sse2(p, xs, ys, it, a, b, tc); }
static void h264_lf_luma_intra_c(uint8_t *p, int xs, int ys, int it, int a, int b)
{ h264_loop_filter_luma_intra_c(p, xs, ys, it, a, b); }
static void h264_lf_luma_intra_sse2(uint8_t *p, int xs, int ys, int it, int a, int b)
{ h264_loop_filter_luma_intra_sse2(p, xs, ys, it, a, b); }
static void h264_lf_chroma_intra_c(uint8_t *p, int xs, int ys, int it, int a, int b)
{ h264_loop_filter_chroma_intra_c(p, xs, ys, it, a, b); }
static void h264_lf_chroma_intra_sse2(uint8_t *p, int xs, int ys, int it, int a, int b)
{ h264_loop_filter_chroma_intra_sse2(p, xs, ys, it, a, b); }

/* Each call filters its own row of a fresh copy of the same field, so the
 * scalar and the SSE2 run over identical data and neither sees the other's
 * smoothing. The field is near-flat because a random one fails the alpha/beta
 * tests almost always, and a filter that is never entered measures nothing. */
static double bench_lf(lf_fn f, int form_b, int inner_iters, int alpha, int beta)
{
    static const int8_t tc[4] = { 3, 0, 4, 2 };
    double t0, t1;
    int i;

    memcpy(dst_a, db_base, sizeof(db_base));
    t0 = now_ms();
    for (i = 0; i < DB_ITER; i++) {
        uint8_t *p = dst_a + (i % (DB_ROWS - 24)) * DB_STRIDE + 8;
        f(p, form_b ? 1 : DB_STRIDE, form_b ? DB_STRIDE : 1,
          inner_iters, alpha, beta, tc);
    }
    t1 = now_ms();
    return t1 - t0;
}

static double bench_lf_intra(lf_intra_fn f, int form_b, int inner_iters,
                             int alpha, int beta)
{
    double t0, t1;
    int i;

    memcpy(dst_a, db_base, sizeof(db_base));
    t0 = now_ms();
    for (i = 0; i < DB_ITER; i++) {
        uint8_t *p = dst_a + (i % (DB_ROWS - 24)) * DB_STRIDE + 8;
        f(p, form_b ? 1 : DB_STRIDE, form_b ? DB_STRIDE : 1,
          inner_iters, alpha, beta);
    }
    t1 = now_ms();
    return t1 - t0;
}

static void db_report(const char *what, int form_b, int inner_iters, int intra,
                      int alpha, int beta, lf_fn c, lf_fn s,
                      lf_intra_fn ci, lf_intra_fn si)
{
    double a, b;

    if (intra) {
        a = bench_lf_intra(ci, form_b, inner_iters, alpha, beta);
        b = bench_lf_intra(si, form_b, inner_iters, alpha, beta);
    } else {
        a = bench_lf(c, form_b, inner_iters, alpha, beta);
        b = bench_lf(s, form_b, inner_iters, alpha, beta);
    }
    printf("%-13s %-10s inner=%d alpha=%3d beta=%2d: scalar %7.1f ms   "
           "sse2 %7.1f ms   %.2fx\n",
           what, form_b ? "xstride=1" : "ystride=1", inner_iters, alpha, beta,
           a, b, a / b);
}

static void deblock_section(void)
{
    int i;

    srand(4711);
    for (i = 0; i < (int)sizeof(db_base); i++)
        db_base[i] = (uint8_t)(120 + rand() % 9 - 4);

    /* alpha/beta 28/20 is what a qp 26 inter edge uses; 255/255 puts every
     * position inside the filter, which is the kernel's own worst case */
    db_report("luma inter", 0, 4, 0, 28, 20,
              h264_lf_luma_c, h264_lf_luma_sse2, NULL, NULL);
    db_report("luma inter", 1, 4, 0, 28, 20,
              h264_lf_luma_c, h264_lf_luma_sse2, NULL, NULL);
    db_report("chroma inter", 0, 2, 0, 28, 20,
              h264_lf_chroma_c, h264_lf_chroma_sse2, NULL, NULL);
    db_report("chroma inter", 1, 2, 0, 28, 20,
              h264_lf_chroma_c, h264_lf_chroma_sse2, NULL, NULL);
    db_report("luma intra", 0, 4, 1, 28, 20,
              NULL, NULL, h264_lf_luma_intra_c, h264_lf_luma_intra_sse2);
    db_report("luma intra", 1, 4, 1, 28, 20,
              NULL, NULL, h264_lf_luma_intra_c, h264_lf_luma_intra_sse2);
    db_report("chroma intra", 0, 2, 1, 28, 20,
              NULL, NULL, h264_lf_chroma_intra_c, h264_lf_chroma_intra_sse2);
    db_report("chroma intra", 1, 2, 1, 28, 20,
              NULL, NULL, h264_lf_chroma_intra_c, h264_lf_chroma_intra_sse2);
    db_report("luma inter", 0, 4, 0, 255, 255,
              h264_lf_luma_c, h264_lf_luma_sse2, NULL, NULL);
    db_report("luma intra", 1, 4, 1, 255, 255,
              NULL, NULL, h264_lf_luma_intra_c, h264_lf_luma_intra_sse2);
}

/* every fractional position and both put/avg modes, over ITER row sweeps */
static double bench_qpel(mc_fn f, int w, int h)
{
    int x, y, avg, i;
    double t0, t1;

    t0 = now_ms();
    for (i = 0; i < ITER; i++) {
        for (y = 0; y < 4; y++)
            for (x = 0; x < 4; x++)
                for (avg = 0; avg < 2; avg++) {
                    if (!x && !y)
                        continue;
                    f(dst_a + OFF * FIELD + OFF, src + OFF * FIELD + OFF,
                      FIELD, w, h, x, y, avg);
                }
    }
    t1 = now_ms();
    return t1 - t0;
}

static void avx_section(void)
{
    static const int hs[3] = { 16, 8, 4 };
    double tot_s = 0, tot_a = 0;
    int i;

    if (!h264_have_avx2) {
        printf("avx2: CPU has none, skipped\n");
        return;
    }
    for (i = 0; i < 3; i++) {
        double s = bench_qpel(h264_qpel_mc_sse2, 16, hs[i]);
        double a = bench_qpel(h264_qpel_mc_avx2, 16, hs[i]);

        tot_s += s; tot_a += a;
        printf("qpel 16x%-2d      sse2 %7.1f ms   avx2 %7.1f ms   %.2fx over sse2\n",
               hs[i], s, a, s / a);
    }
    printf("qpel 16-wide total sse2 %.1f ms  avx2 %.1f ms  (%.2fx)\n",
           tot_s, tot_a, tot_s / tot_a);
}

int main(void)
{
    int i, k;
    static const int sizes[6][2] = { { 4, 4 }, { 8, 4 }, { 4, 8 },
                                     { 8, 8 }, { 16, 8 }, { 16, 16 } };
    double q_c = 0, q_s = 0, c_c = 0, c_s = 0;

    srand(12345);
    for (i = 0; i < (int)sizeof(src); i++)
        src[i] = (uint8_t)rand();

    for (k = 0; k < 6; k++) {
        int w = sizes[k][0], h = sizes[k][1];
        int x, y, avg;
        double a = 0, b = 0;
        for (y = 0; y < 4; y++)
            for (x = 0; x < 4; x++)
                for (avg = 0; avg < 2; avg++) {
                    if (!x && !y) continue;
                    a += bench(h264_qpel_mc_c, w, h, x, y, avg);
                    b += bench(h264_qpel_mc_sse2, w, h, x, y, avg);
                }
        q_c += a; q_s += b;
        printf("qpel %2dx%-2d   scalar %7.1f ms   sse2 %7.1f ms   speedup %.2fx\n",
               w, h, a, b, a / b);
    }

    for (k = 0; k < 6; k++) {
        int w = sizes[k][0], h = sizes[k][1];
        int x, y, avg;
        double a = 0, b = 0;
        for (y = 0; y < 8; y++)
            for (x = 0; x < 8; x++)
                for (avg = 0; avg < 2; avg++) {
                    if (!x && !y) continue;
                    a += bench(h264_chroma_mc_c, w / 2, h / 2, x, y, avg);
                    b += bench(h264_chroma_mc_sse2, w / 2, h / 2, x, y, avg);
                }
        c_c += a; c_s += b;
        printf("chroma %2dx%-2d scalar %7.1f ms   sse2 %7.1f ms   speedup %.2fx\n",
               w / 2, h / 2, a, b, a / b);
    }

    printf("total qpel   scalar %.1f ms  sse2 %.1f ms  (%.2fx)\n", q_c, q_s, q_c / q_s);
    printf("total chroma scalar %.1f ms  sse2 %.1f ms  (%.2fx)\n", c_c, c_s, c_c / c_s);
    deblock_section();
    avx_section();
    (void)dst_b;
    return 0;
}
