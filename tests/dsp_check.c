/*
 * dsp_check - compare the scalar and SIMD DSP kernels sample by sample.
 *
 * The decoder is bit-exact against the reference, so a kernel that matches
 * the scalar C version here is as correct as the scalar version is. Build:
 *   cc -O2 -I. -o dsp_check tests/dsp_check.c libh264dec.a
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "h264_internal.h"

#define FIELD 96
#define OFF   8          /* source origin inside the field, gives margin */

static uint8_t field_[FIELD * FIELD];
static uint8_t dst_c[FIELD * FIELD], dst_s[FIELD * FIELD];
static unsigned rnd_state = 12345;

static unsigned rnd(void)
{
    rnd_state = rnd_state * 1103515245u + 12345u;
    return (rnd_state >> 16) & 0xff;
}

static int fails;
static int cur_x = -1, cur_y = -1;
static int dump_all;

static void compare(const char *what, int w, int h, int avg)
{
    const int pitch = FIELD;
    const int base = OFF * FIELD + OFF;
    int x, y, any = 0;
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            uint8_t c = dst_c[base + y * pitch + x];
            uint8_t s = dst_s[base + y * pitch + x];
            if (c == s)
                continue;
            any = 1;
            if (fails < 20 || dump_all)
                printf("%s w%d h%d xy%d%d avg%d: out(%d,%d) c=%d sse=%d\n",
                       what, w, h, cur_x, cur_y, avg, x, y, c, s);
            if (!dump_all) {
                fails++;
                return;
            }
        }
    }
    fails += any;
}

static void prep(int seed)
{
    unsigned i;
    rnd_state = seed;
    for (i = 0; i < sizeof(field_); i++)
        field_[i] = (uint8_t)rnd();
    memset(dst_c, 0x5a, sizeof(dst_c));
    memset(dst_s, 0x5a, sizeof(dst_s));
}

static int idct_block(int w, int range, unsigned seed)
{
    int16_t bc[64], bs[64];
    int i, r, c, bad = 0;
    const int pitch = FIELD;
    const int area = w * w;

    rnd_state = seed;
    for (i = 0; i < area; i++) {
        int v = (int)(((rnd() << 8) | rnd()) % (unsigned)(2 * range + 1)) - range;
        bc[i] = (int16_t)v;
        bs[i] = (int16_t)v;
    }
    for (i = 0; i < FIELD * FIELD; i++) {
        uint8_t v = (uint8_t)rnd();
        dst_c[i] = v;
        dst_s[i] = v;
    }
    if (w == 8) {
        h264_idct8_dc_add_c(dst_c + OFF * pitch + OFF, bc, pitch);
        h264_idct8_dc_add_simd(dst_s + OFF * pitch + OFF, bs, pitch);
    } else {
        h264_idct_dc_add_c(dst_c + OFF * pitch + OFF, bc, pitch);
        h264_idct_dc_add_simd(dst_s + OFF * pitch + OFF, bs, pitch);
    }
    for (r = 0; r < w; r++)
        for (c = 0; c < w; c++)
            if (dst_c[(OFF + r) * pitch + OFF + c] != dst_s[(OFF + r) * pitch + OFF + c])
                bad++;
    if (bc[0] != bs[0])
        bad++;
    return bad;
}

static int idct_full_case(int range, unsigned seed)
{
    int16_t bc[16], bs[16];
    int i, r, c, bad = 0;
    const int pitch = FIELD;

    rnd_state = seed;
    for (i = 0; i < 16; i++) {
        int v = (int)(((rnd() << 8) | rnd()) % (unsigned)(2 * range + 1)) - range;
        bc[i] = (int16_t)v;
        bs[i] = (int16_t)v;
    }
    for (i = 0; i < FIELD * FIELD; i++) {
        uint8_t v = (uint8_t)rnd();
        dst_c[i] = v;
        dst_s[i] = v;
    }
    h264_idct_add_c(dst_c + OFF * pitch + OFF, bc, pitch);
    h264_idct_add_simd(dst_s + OFF * pitch + OFF, bs, pitch);
    for (r = 0; r < 4; r++)
        for (c = 0; c < 4; c++)
            if (dst_c[(OFF + r) * pitch + OFF + c] != dst_s[(OFF + r) * pitch + OFF + c])
                bad++;
    for (i = 0; i < 16; i++)
        if (bc[i] != bs[i])
            bad++;
    return bad;
}

static int idct_section(void)
{
    static const int ranges[] = { 1, 63, 255, 4095, 32767 };
    int ri, k, bad, blocks, total = 0;

    for (ri = 0; ri < 5; ri++) {
        bad = 0; blocks = 0;
        for (k = 0; k < 400; k++) {
            bad += idct_full_case(ranges[ri], 900u + (unsigned)ri * 31 + (unsigned)k);
            blocks++;
        }
        printf("idct_add    range %5d: %d blocks, %d mismatching samples\n",
               ranges[ri], blocks, bad);
        total += bad;
    }
    for (ri = 0; ri < 5; ri++) {
        bad = 0;
        for (k = 0; k < 200; k++) {
            bad += idct_block(4, ranges[ri], 700u + (unsigned)ri * 17 + (unsigned)k);
            bad += idct_block(8, ranges[ri], 500u + (unsigned)ri * 13 + (unsigned)k);
        }
        printf("idct_dc_add range %5d: %d mismatching samples\n", ranges[ri], bad);
        total += bad;
    }
    return total;
}

/* One deblocking case: the same pixels filtered by the C kernel and by the
 * SSE2 one, then the whole buffer compared so that a write outside the
 * filtered taps shows up as loudly as a wrong value. form_b picks the layout
 * the decoder uses for the two edge directions: xstride==1 (taps adjacent,
 * positions a row apart) or ystride==1 (the other way round). A narrow buffer
 * keeps the sweep cheap; positions then run across row ends, which is what a
 * real picture plane looks like to the filter anyway. */
#define DBF 64
#define DBO 16

static int deblock_case(int chroma, int intra, int form_b, int inner_iters, int stride,
                        unsigned seed, int *modified)
{
    static const uint8_t levels[8] = { 0, 1, 2, 4, 8, 16, 30, 255 };
    static const int8_t tc_levels[9] = { -1, -1, 0, 1, 2, 3, 4, 5, 7 };
    static uint8_t db_c[DBF * DBF], db_s[DBF * DBF], db_p[DBF * DBF];
#if defined(H264_SIMD_X86)
    /* the AVX2 luma kernel only takes the ystride==1 layout, so it gets its
     * own copy; the comparison below includes it only when it actually ran */
    static uint8_t db_a[DBF * DBF];
#endif
    int8_t tc[4];
    uint8_t *pc, *ps;
    int i, bad = 0, alpha, beta;
#if defined(H264_SIMD_X86)
    int avx2_run = 0;
#endif
    const int xstride = form_b ? 1 : stride;
    const int ystride = form_b ? stride : 1;

    *modified = 0;
    rnd_state = seed;
    alpha = levels[rnd() % 8];
    beta = levels[rnd() % 8];
    for (i = 0; i < 4; i++)
        tc[i] = (int8_t)tc_levels[rnd() % 9];

    for (i = 0; i < DBF * DBF; i++)
        /* half the cases are near-flat: with uncorrelated bytes the alpha and
         * beta tests almost never pass, and the sweep would compare buffers
         * that neither kernel touched */
        db_c[i] = (seed & 1) ? (uint8_t)(128 + rnd() % 7) : (uint8_t)rnd();
    memcpy(db_p, db_c, sizeof(db_p));
    memcpy(db_s, db_c, sizeof(db_s));
#if defined(H264_SIMD_X86)
    memcpy(db_a, db_c, sizeof(db_a));
#endif

    pc = db_c + DBO * stride + DBO;
    ps = db_s + DBO * stride + DBO;
    if (intra) {
        if (chroma) {
            h264_loop_filter_chroma_intra_c(pc, xstride, ystride, inner_iters, alpha, beta);
            h264_loop_filter_chroma_intra_simd(ps, xstride, ystride, inner_iters, alpha, beta);
        } else {
            h264_loop_filter_luma_intra_c(pc, xstride, ystride, inner_iters, alpha, beta);
            h264_loop_filter_luma_intra_simd(ps, xstride, ystride, inner_iters, alpha, beta);
        }
    } else if (chroma) {
        h264_loop_filter_chroma_c(pc, xstride, ystride, inner_iters, alpha, beta, tc);
        h264_loop_filter_chroma_simd(ps, xstride, ystride, inner_iters, alpha, beta, tc);
    } else {
        h264_loop_filter_luma_c(pc, xstride, ystride, inner_iters, alpha, beta, tc);
        h264_loop_filter_luma_simd(ps, xstride, ystride, inner_iters, alpha, beta, tc);
#if defined(H264_SIMD_X86)
        if (h264_have_avx2 && xstride != 1 && inner_iters == 4) {
            h264_loop_filter_luma_avx2(db_a + DBO * stride + DBO, xstride, ystride,
                                       inner_iters, alpha, beta, tc);
            avx2_run = 1;
        }
#endif
    }

    for (i = 0; i < DBF * DBF; i++) {
        /* a case where nothing was filtered proves nothing, so count it */
        if (db_c[i] != db_p[i])
            (*modified)++;
        if (db_c[i] != db_s[i])
            bad++;
#if defined(H264_SIMD_X86)
        if (avx2_run && db_c[i] != db_a[i])
            bad++;
#endif
    }
    return bad;
}

static int deblock_section(void)
{
    static const int strides[3] = { DBF, 40, 17 };
    int total = 0;
    int chroma, intra, form_b, it, si, k;

    for (intra = 0; intra < 2; intra++)
    for (chroma = 0; chroma < 2; chroma++) {
        for (form_b = 0; form_b < 2; form_b++) {
            for (it = 0; it < 3; it++) {
                const int inner_iters = 1 << it;
                int bad = 0, cases = 0, modified = 0;

                for (si = 0; si < 3; si++)
                    for (k = 0; k < 120; k++) {
                        int mod = 0;
                        bad += deblock_case(chroma, intra, form_b, inner_iters, strides[si],
                                            4242u + (unsigned)si * 977u +
                                            (unsigned)k * 31u + (unsigned)it * 7u +
                                            (unsigned)form_b * 3u + (unsigned)chroma * 13u +
                                            (unsigned)intra * 29u,
                                            &mod);
                        modified += mod;
                        cases++;
                    }
                printf("deblock %-5s %-6s %-10s inner=%d: %d cases, %d samples filtered, "
                       "%d mismatching\n",
                       intra ? "intra" : "inter", chroma ? "chroma" : "luma",
                       form_b ? "xstride=1" : "ystride=1",
                       inner_iters, cases, modified, bad);
                total += bad;
                if (modified == 0) {
                    printf("deblock: no case filtered anything - the sweep is broken\n");
                    total++;
                }
            }
        }
    }
    return total;
}

/* the dispatcher only routes 16-wide luma blocks to AVX2, so that is the width
 * compared here; full-pel (x=y=0) stays on the copy path and is skipped */
static int qpel_avx2_section(void)
{
#if defined(H264_SIMD_X86)
    static const int heights[3] = { 16, 8, 4 };
    int hi, x, y, avg, before, bad = 0;

    if (!h264_have_avx2) {
        printf("qpel_avx2: CPU has no AVX2, skipped\n");
        return 0;
    }
    for (hi = 0; hi < 3; hi++) {
        const int h = heights[hi];
        for (y = 0; y < 4; y++) {
            for (x = 0; x < 4; x++) {
                if (!x && !y)
                    continue;
                for (avg = 0; avg < 2; avg++) {
                    const uint8_t *src = field_ + OFF * FIELD + OFF;
                    prep(991u + (unsigned)hi * 97u + (unsigned)y * 7u + (unsigned)x);
                    h264_qpel_mc_c(dst_c + OFF * FIELD + OFF, src, FIELD,
                                   16, h, x, y, avg);
                    h264_qpel_mc_avx2(dst_s + OFF * FIELD + OFF, src, FIELD,
                                      16, h, x, y, avg);
                    before = fails;
                    cur_x = x; cur_y = y;
                    compare("qpel_avx2", 16, h, avg);
                    bad += fails - before;
                }
            }
        }
        printf("qpel_avx2 h%-2d: %d mismatching samples\n", h, bad);
    }
    return bad;
#else
    /* the second half of the vector register file is an x86 extension; NEON has
     * no counterpart to compare here, and the kernel is not even linked in */
    printf("qpel_avx2: not an x86 target, skipped\n");
    return 0;
#endif
}

/* integer motion: the scalar reference is the x=y=0 case of h264_qpel_mc_c,
 * and the average mode has to be covered because pavgusb is its rounding */
static int hpel_section(void)
{
    static const int sizes[][2] = { { 4, 4 }, { 8, 4 }, { 4, 8 }, { 8, 8 },
                                    { 16, 8 }, { 8, 16 }, { 16, 16 },
                                    { 16, 4 }, { 4, 16 } };
    size_t si;
    int avg, before, bad = 0;

    for (si = 0; si < sizeof(sizes) / sizeof(sizes[0]); si++) {
        const int w = sizes[si][0], h = sizes[si][1];

        for (avg = 0; avg < 2; avg++) {
            const uint8_t *src = field_ + OFF * FIELD + OFF;
            prep(1227u + (unsigned)si * 53u + (unsigned)avg);
            h264_qpel_mc_c(dst_c + OFF * FIELD + OFF, src, FIELD, w, h, 0, 0, avg);
            h264_hpel_mc_simd(dst_s + OFF * FIELD + OFF, src, FIELD, w, h, avg);
            before = fails;
            cur_x = 0; cur_y = 0;
            compare("hpel", w, h, avg);
            bad += fails - before;
        }
    }
    printf("hpel: %d mismatching samples over %d block shapes\n",
           bad, (int)(sizeof(sizes) / sizeof(sizes[0])) * 2);
    return bad;
}

/* weighted prediction rewrites the block it reads, so both buffers have to
 * start from the same varied content rather than the memset pattern; the
 * source of the biweighted case is taken a row over so it differs */
static void wp_buffers(int h)
{
    uint8_t *dc = dst_c + OFF * FIELD + OFF;
    uint8_t *ds = dst_s + OFF * FIELD + OFF;
    const uint8_t *sp = field_ + OFF * FIELD + OFF + FIELD + 3;
    int y;

    for (y = 0; y < h; y++) {
        memcpy(dc + y * FIELD, sp + y * FIELD, FIELD);
        memcpy(ds + y * FIELD, sp + y * FIELD, FIELD);
    }
}

static const int wp_sizes[][2] = { { 16, 16 }, { 16, 8 }, { 8, 16 }, { 8, 8 },
                                   { 16, 4 }, { 8, 4 }, { 4, 8 }, { 4, 4 },
                                   { 2, 2 }, { 1, 2 }, { 12, 3 }, { 9, 5 } };
static const int wp_denoms[] = { 0, 1, 2, 3, 5, 6, 7 };
static const int wp_weights[] = { -200, -129, -128, -64, -1, 0, 1, 16, 32,
                                  63, 64, 127, 128, 129, 200 };
static const int wp_offsets[] = { -300, -256, -129, -1, 0, 1, 16, 32, 127,
                                  255, 256, 300 };

/* the dispatcher is what the decoder runs, so it is what is compared against
 * the scalar reference: the identity shortcut, the int16 gate and the fallback
 * are all part of what has to agree */
static int wp_single_section(void)
{
    int bad = 0, cases = 0, vector = 0, identity = 0, si, di, wi, oi, before;
    long pixels = 0;

    for (si = 0; si < (int)(sizeof(wp_sizes) / sizeof(wp_sizes[0])); si++) {
        const int w = wp_sizes[si][0], h = wp_sizes[si][1];

        for (di = 0; di < (int)(sizeof(wp_denoms) / sizeof(int)); di++) {
            const int d = wp_denoms[di];

            for (wi = 0; wi < (int)(sizeof(wp_weights) / sizeof(int)); wi++) {
                const int weight = wp_weights[wi];

                for (oi = 0; oi < (int)(sizeof(wp_offsets) / sizeof(int)); oi++) {
                    const int offset = wp_offsets[oi];
                    uint8_t *dc = dst_c + OFF * FIELD + OFF;
                    uint8_t *ds = dst_s + OFF * FIELD + OFF;
                    long off = ((long)offset << d) + (d ? (long)1 << (d - 1) : 0);

                    prep(31337u + (unsigned)si * 7u + (unsigned)d * 131u +
                         (unsigned)wi * 17u + (unsigned)oi);
                    wp_buffers(h);
                    if (weight == (1 << d) && offset == 0)
                        identity++;
                    else if (d <= 7 && weight >= -128 && weight <= 128 &&
                             off >= -32768 && off <= 32767)
                        vector++;
                    h264_weight_pixels_c(dc, FIELD, w, h, d, weight, offset);
                    h264_weight_pixels(ds, FIELD, w, h, d, weight, offset);
                    before = fails;
                    cur_x = weight;
                    cur_y = d;
                    compare("weight", w, h, offset);
                    bad += fails - before;
                    cases++;
                    pixels += (long)w * h;
                }
            }
        }
    }
    printf("weight: %d cases (%ld pixels, %d to the vector kernel, %d identity), "
           "%d mismatching\n", cases, pixels, vector, identity, bad);
    if (!vector || !identity || pixels < 1000) {
        printf("weight: vacuous sweep (vector=%d identity=%d pixels=%ld)\n",
               vector, identity, pixels);
        bad++;
    }
    return bad;
}

static int wp_bi_section(void)
{
    int bad = 0, cases = 0, vector = 0, identity = 0, si, di, wi, oi, before;
    long pixels = 0;

    for (si = 0; si < (int)(sizeof(wp_sizes) / sizeof(wp_sizes[0])); si++) {
        const int w = wp_sizes[si][0], h = wp_sizes[si][1];

        for (di = 0; di < (int)(sizeof(wp_denoms) / sizeof(int)); di++) {
            const int d = wp_denoms[di];

            for (wi = 0; wi < (int)(sizeof(wp_weights) / sizeof(int)); wi++) {
                const int wd = wp_weights[wi];
                const int ws = 64 - wd;     /* implicit weighting keeps w0+w1=64 */

                for (oi = 0; oi < (int)(sizeof(wp_offsets) / sizeof(int)); oi++) {
                    const int offset = wp_offsets[oi];
                    uint8_t *dc = dst_c + OFF * FIELD + OFF;
                    uint8_t *ds = dst_s + OFF * FIELD + OFF;
                    const uint8_t *src = field_ + OFF * FIELD + OFF + FIELD + 3;
                    long off;

                    prep(41177u + (unsigned)si * 11u + (unsigned)d * 137u +
                         (unsigned)wi * 19u + (unsigned)oi);
                    wp_buffers(h);
                    off = (long)((offset + 1) | 1) << d;
                    if (wd == ws && wd == (1 << d) && offset == 0)
                        identity++;
                    else if (d <= 6 && wd >= -128 && wd <= 128 &&
                             ws >= -128 && ws <= 128 && off >= -32768 &&
                             off <= 32767)
                        vector++;
                    h264_biweight_pixels_c(dc, src, FIELD, w, h, d, wd, ws,
                                           offset);
                    h264_biweight_pixels(ds, src, FIELD, w, h, d, wd, ws,
                                         offset);
                    before = fails;
                    cur_x = wd;
                    cur_y = d;
                    compare("biweight", w, h, offset);
                    bad += fails - before;
                    cases++;
                    pixels += (long)w * h;
                }
            }
        }
    }
    printf("biweight: %d cases (%ld pixels, %d to the vector kernel, %d identity)"
           ", %d mismatching\n", cases, pixels, vector, identity, bad);
    if (!vector || !identity) {
        printf("biweight: sweep never reached the vector or the identity path"
               " - the check is broken\n");
        bad++;
    }
    return bad;
}

/* the whole-macroblock loop-filter skip rests on one number: that alpha and tc0
 * are still zero where it says they are. Check it against the tables, and check
 * that the check can fail. */
static int deblock_inert_section(void)
{
    const int r = h264_deblock_check_inert_index();

    if (r) {
        printf("deblock inert index: BROKEN (%d) - the skip in h264_deblock.c no "
               "longer matches the alpha/tc0 tables\n", r);
        return 1;
    }
    printf("deblock inert index: alpha and tc0 zero through index 15, as assumed\n");
    return 0;
}

/* The chroma dispatcher takes a copy when a block does not move. That is only
 * sound while the bilinear sum at (0, 0) really is the sample itself, so check
 * the dispatcher against the filter it replaces at every width the decoder can
 * ask for - including the 2-wide block it must not hand to the vector copy. */
static int chroma_int_section(void)
{
    static const int sizes[][2] = { { 8, 8 }, { 8, 4 }, { 4, 8 }, { 4, 4 },
                                    { 2, 2 }, { 2, 4 }, { 4, 2 } };
    const int n = (int)(sizeof(sizes) / sizeof(sizes[0]));
    int si, avg, bad = 0;

    for (si = 0; si < n; si++) {
        for (avg = 0; avg < 2; avg++) {
            const uint8_t *src = field_ + OFF * FIELD + OFF;
            int before;
            prep(4242 + si * 5 + avg);
            h264_chroma_mc(dst_c + OFF * FIELD + OFF, src, FIELD,
                           sizes[si][0], sizes[si][1], 0, 0, avg);
            h264_chroma_mc_c(dst_s + OFF * FIELD + OFF, src, FIELD,
                             sizes[si][0], sizes[si][1], 0, 0, avg);
            before = fails;
            cur_x = cur_y = 0;
            compare("chroma-int", sizes[si][0], sizes[si][1], avg);
            bad += fails - before;
        }
    }
    printf("chroma-int: %d cases, dispatcher at xy00 against the filter it "
           "replaces\n", n * 2);
    return bad;
}

int main(int argc, char **argv)
{
    static const int sizes[][2] = { { 4, 4 }, { 8, 4 }, { 4, 8 }, { 8, 8 },
                                    { 16, 8 }, { 8, 16 }, { 16, 16 },
                                    { 16, 4 }, { 4, 16 } };
    size_t si;
    int x, y, avg;
    int nbad = 0;

    dump_all = getenv("DSP_DUMP_ALL") != NULL;

    if (argc > 1 && !strcmp(argv[1], "dump")) {
        /* single case, print both blocks: dsp_check dump <w> <x> <y> <avg> */
        int w = atoi(argv[2]), xx = atoi(argv[3]);
        int yy = atoi(argv[4]), aa = atoi(argv[5]);
        int r, c;
        const uint8_t *src = field_ + OFF * FIELD + OFF;
        prep(777 + yy * 7 + xx);
        h264_qpel_mc_c(dst_c + OFF * FIELD + OFF, src, FIELD, w, w, xx, yy, aa);
        h264_qpel_mc_simd(dst_s + OFF * FIELD + OFF, src, FIELD, w, w, xx, yy, aa);
        printf("src rows:\n");
        for (r = -2; r < w + 4; r++) {
            printf(" r%3d:", r);
            for (c = -2; c < w + 2; c++)
                printf(" %3d", field_[(OFF + r) * FIELD + OFF + c]);
            printf("\n");
        }
        for (r = 0; r < w; r++) {
            printf("c:");
            for (c = 0; c < w; c++)
                printf(" %3d", dst_c[(OFF + r) * FIELD + OFF + c]);
            printf("\n s:");
            for (c = 0; c < w; c++)
                printf(" %3d", dst_s[(OFF + r) * FIELD + OFF + c]);
            printf("\n");
        }
        return 0;
    }

    for (si = 0; si < sizeof(sizes) / sizeof(sizes[0]); si++) {
        int w = sizes[si][0], h = sizes[si][1];
        for (y = 0; y < 4; y++) {
            for (x = 0; x < 4; x++) {
                if (!x && !y)
                    continue;
                for (avg = 0; avg < 2; avg++) {
                    const uint8_t *src = field_ + OFF * FIELD + OFF;
                    prep(777 + (int)si * 31 + y * 7 + x);
                    h264_qpel_mc_c(dst_c + OFF * FIELD + OFF, src, FIELD,
                                   w, h, x, y, avg);
                    h264_qpel_mc_simd(dst_s + OFF * FIELD + OFF, src, FIELD,
                                      w, h, x, y, avg);
                    {
                        int before = fails;
                        cur_x = x; cur_y = y;
                        compare("qpel", w, h, avg);
                        nbad += fails - before;
                    }
                }
            }
        }
    }

    for (si = 0; si < sizeof(sizes) / sizeof(sizes[0]); si++) {
        int w = sizes[si][0] / 2, h = sizes[si][1] / 2;
        if (w < 2)
            w = 2;
        if (h < 2)
            h = 2;
        for (y = 0; y < 8; y++) {
            for (x = 0; x < 8; x++) {
                for (avg = 0; avg < 2; avg++) {
                    const uint8_t *src = field_ + OFF * FIELD + OFF;
                    prep(11 + (int)si * 17 + y * 5 + x);
                    h264_chroma_mc_c(dst_c + OFF * FIELD + OFF, src, FIELD,
                                     w, h, x, y, avg);
                    h264_chroma_mc_simd(dst_s + OFF * FIELD + OFF, src, FIELD,
                                        w, h, x, y, avg);
                    {
                        int before = fails;
                        compare("chroma", w, h, avg);
                        nbad += fails - before;
                    }
                }
            }
        }
    }


    nbad += idct_section();
    nbad += deblock_section();
    nbad += deblock_inert_section();
    nbad += chroma_int_section();
    nbad += qpel_avx2_section();
    nbad += hpel_section();
    nbad += wp_single_section();
    nbad += wp_bi_section();
    printf("dsp_check: %d mismatching cases\n", nbad);
    return nbad != 0;
}
