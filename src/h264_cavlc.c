/*
 * H.264 CAVLC decoding (8-bit 4:2:0).
 * Ported from FFmpeg's h264_cavlc.c (LGPL-2.1+).
 */
#include "h264_internal.h"

/* ------------------------------------------------------------------ */
/* VLC tables                                                          */
/* ------------------------------------------------------------------ */
static const uint8_t chroma_dc_coeff_token_len[4 * 5] = {
    2, 0, 0, 0,
    6, 1, 0, 0,
    6, 6, 3, 0,
    6, 7, 7, 6,
    6, 8, 8, 7,
};
static const uint8_t chroma_dc_coeff_token_bits[4 * 5] = {
    1, 0, 0, 0,
    7, 1, 0, 0,
    4, 6, 1, 0,
    3, 3, 2, 5,
    2, 3, 2, 0,
};

static const uint8_t coeff_token_len[4][4 * 17] = {
{
    1, 0, 0, 0,
    6, 2, 0, 0,     8, 6, 3, 0,     9, 8, 7, 5,    10, 9, 8, 6,
    11,10, 9, 7,    13,11,10, 8,    13,13,11, 9,    13,13,13,10,
    14,14,13,11,    14,14,14,13,    15,15,14,14,    15,15,15,14,
    16,15,15,15,    16,16,16,15,    16,16,16,16,    16,16,16,16,
},
{
    2, 0, 0, 0,
    6, 2, 0, 0,     6, 5, 3, 0,     7, 6, 6, 4,     8, 6, 6, 4,
    8, 7, 7, 5,     9, 8, 8, 6,    11, 9, 9, 6,    11,11,11, 7,
    12,11,11, 9,    12,12,12,11,    12,12,12,11,    13,13,13,12,
    13,13,13,13,    13,14,13,13,    14,14,14,13,    14,14,14,14,
},
{
    4, 0, 0, 0,
    6, 4, 0, 0,     6, 5, 4, 0,     6, 5, 5, 4,     7, 5, 5, 4,
    7, 5, 5, 4,     7, 6, 6, 4,     7, 6, 6, 4,     8, 7, 7, 5,
    8, 8, 7, 6,     9, 8, 8, 7,     9, 9, 8, 8,     9, 9, 9, 8,
    10, 9, 9, 9,    10,10,10,10,    10,10,10,10,    10,10,10,10,
},
{
    6, 0, 0, 0,
    6, 6, 0, 0,     6, 6, 6, 0,     6, 6, 6, 6,     6, 6, 6, 6,
    6, 6, 6, 6,     6, 6, 6, 6,     6, 6, 6, 6,     6, 6, 6, 6,
    6, 6, 6, 6,     6, 6, 6, 6,     6, 6, 6, 6,     6, 6, 6, 6,
    6, 6, 6, 6,     6, 6, 6, 6,     6, 6, 6, 6,     6, 6, 6, 6,
}
};

static const uint8_t coeff_token_bits[4][4 * 17] = {
{
    1, 0, 0, 0,
    5, 1, 0, 0,     7, 4, 1, 0,     7, 6, 5, 3,     7, 6, 5, 3,
    7, 6, 5, 4,    15, 6, 5, 4,    11,14, 5, 4,     8,10,13, 4,
    15,14, 9, 4,    11,10,13,12,    15,14, 9,12,    11,10,13, 8,
    15, 1, 9,12,    11,14,13, 8,     7,10, 9,12,     4, 6, 5, 8,
},
{
    3, 0, 0, 0,
    11, 2, 0, 0,     7, 7, 3, 0,     7,10, 9, 5,     7, 6, 5, 4,
    4, 6, 5, 6,     7, 6, 5, 8,    15, 6, 5, 4,    11,14,13, 4,
    15,10, 9, 4,    11,14,13,12,     8,10, 9, 8,    15,14,13,12,
    11,10, 9,12,     7,11, 6, 8,     9, 8,10, 1,     7, 6, 5, 4,
},
{
    15, 0, 0, 0,
    15,14, 0, 0,    11,15,13, 0,     8,12,14,12,    15,10,11,11,
    11, 8, 9,10,     9,14,13, 9,     8,10, 9, 8,    15,14,13,13,
    11,14,10,12,    15,10,13,12,    11,14, 9,12,     8,10,13, 8,
    13, 7, 9,12,     9,12,11,10,     5, 8, 7, 6,     1, 4, 3, 2,
},
{
    3, 0, 0, 0,
    0, 1, 0, 0,     4, 5, 6, 0,     8, 9,10,11,    12,13,14,15,
    16,17,18,19,    20,21,22,23,    24,25,26,27,    28,29,30,31,
    32,33,34,35,    36,37,38,39,    40,41,42,43,    44,45,46,47,
    48,49,50,51,    52,53,54,55,    56,57,58,59,    60,61,62,63,
}
};

static const uint8_t total_zeros_len[16][16] = {
    {1,3,3,4,4,5,5,6,6,7,7,8,8,9,9,9},
    {3,3,3,3,3,4,4,4,4,5,5,6,6,6,6},
    {4,3,3,3,4,4,3,3,4,5,5,6,5,6},
    {5,3,4,4,3,3,3,4,3,4,5,5,5},
    {4,4,4,3,3,3,3,3,4,5,4,5},
    {6,5,3,3,3,3,3,3,4,3,6},
    {6,5,3,3,3,2,3,4,3,6},
    {6,4,5,3,2,2,3,3,6},
    {6,6,4,2,2,3,2,5},
    {5,5,3,2,2,2,4},
    {4,4,3,3,1,3},
    {4,4,2,1,3},
    {3,3,1,2},
    {2,2,1},
    {1,1},
};

static const uint8_t total_zeros_bits[16][16] = {
    {1,3,2,3,2,3,2,3,2,3,2,3,2,3,2,1},
    {7,6,5,4,3,5,4,3,2,3,2,3,2,1,0},
    {5,7,6,5,4,3,4,3,2,3,2,1,1,0},
    {3,7,5,4,6,5,4,3,3,2,2,1,0},
    {5,4,3,7,6,5,4,3,2,1,1,0},
    {1,1,7,6,5,4,3,2,1,1,0},
    {1,1,5,4,3,3,2,1,1,0},
    {1,1,1,3,3,2,2,1,0},
    {1,0,1,3,2,1,1,1},
    {1,0,1,3,2,1,1},
    {0,1,1,2,1,3},
    {0,1,1,1,1},
    {0,1,1,1},
    {0,1,1},
    {0,1},
};

static const uint8_t chroma_dc_total_zeros_len[3][4] = {
    { 1, 2, 3, 3,},
    { 1, 2, 2, 0,},
    { 1, 1, 0, 0,},
};
static const uint8_t chroma_dc_total_zeros_bits[3][4] = {
    { 1, 1, 1, 0,},
    { 1, 1, 0, 0,},
    { 1, 0, 0, 0,},
};

static const uint8_t chroma422_dc_coeff_token_len[4 * 9] = {
    1,  0,  0,  0,
    7,  2,  0,  0,
    7,  7,  3,  0,
    9,  7,  7,  5,
    9,  9,  7,  6,
    10, 10, 9,  7,
    11, 11, 10, 7,
    12, 12, 11, 10,
    13, 12, 12, 11,
};

static const uint8_t chroma422_dc_coeff_token_bits[4 * 9] = {
    1,  0,  0,  0,
    15, 1,  0,  0,
    14, 13, 1,  0,
    7,  12, 11, 1,
    6,  5,  10, 1,
    7,  6,  4,  9,
    7,  6,  5,  8,
    7,  6,  5,  4,
    7,  5,  4,  4,
};

static const uint8_t chroma422_dc_total_zeros_len[7][8] = {
    { 1, 3, 3, 4, 4, 4, 5, 5 },
    { 3, 2, 3, 3, 3, 3, 3 },
    { 3, 3, 2, 2, 3, 3 },
    { 3, 2, 2, 2, 3 },
    { 2, 2, 2, 2 },
    { 2, 2, 1 },
    { 1, 1 },
};

static const uint8_t chroma422_dc_total_zeros_bits[7][8] = {
    { 1, 2, 3, 2, 3, 1, 1, 0 },
    { 0, 1, 1, 4, 5, 6, 7 },
    { 0, 1, 1, 2, 6, 7 },
    { 6, 0, 1, 2, 7 },
    { 0, 1, 2, 3 },
    { 0, 1, 1 },
    { 0, 1 },
};

static const uint8_t run_len[7][16] = {
    {1,1},
    {1,2,2},
    {2,2,2,2},
    {2,2,2,3,3},
    {2,2,3,3,3,3},
    {2,3,3,3,3,3,3},
    {3,3,3,3,3,3,3,4,5,6,7,8,9,10,11},
};
static const uint8_t run_bits[7][16] = {
    {1,0},
    {1,1,0},
    {3,2,1,0},
    {3,2,1,1,0},
    {3,2,3,2,1,0},
    {3,0,1,3,2,5,4},
    {7,6,5,4,3,2,1,1,1,1,1,1,1,1,1},
};

typedef struct H264VLCEntry {
    uint8_t len;
    uint8_t sym;
} H264VLCEntry;

typedef struct H264VLC {
    H264VLCEntry *table;
    int bits;
} H264VLC;

#define COEFF_TOKEN_BITS 16
#define CHROMA_DC_TOKEN_BITS 8
#define CHROMA422_DC_TOKEN_BITS 13
#define TOTAL_ZEROS_BITS 9
#define CHROMA_DC_ZEROS_BITS 3
#define CHROMA422_DC_ZEROS_BITS 5
#define RUN_BITS 6
#define RUN7_BITS 11

static H264VLCEntry coeff_token_lut[4][1 << COEFF_TOKEN_BITS];
static H264VLCEntry chroma_dc_token_lut[1 << CHROMA_DC_TOKEN_BITS];
static H264VLCEntry chroma422_dc_token_lut[1 << CHROMA422_DC_TOKEN_BITS];
static H264VLCEntry total_zeros_lut[16][1 << TOTAL_ZEROS_BITS];
static H264VLCEntry chroma_dc_zeros_lut[4][1 << CHROMA_DC_ZEROS_BITS];
static H264VLCEntry chroma422_dc_zeros_lut[8][1 << CHROMA422_DC_ZEROS_BITS];
static H264VLCEntry run_lut[7][1 << RUN7_BITS];
static int cavlc_initialized;

static int build_vlc(H264VLCEntry *table, int bits, const uint8_t *lens,
                     const uint8_t *codes, int count)
{
    int i;
    memset(table, 0, sizeof(H264VLCEntry) << bits);
    for (i = 0; i < count; i++) {
        int len = lens[i];
        int code = codes[i];
        int shift, j;
        if (!len)
            continue;
        if (len > bits)
            return -1;
        shift = bits - len;
        for (j = 0; j < (1 << shift); j++) {
            H264VLCEntry *e = &table[(code << shift) | j];
            e->len = (uint8_t)len;
            e->sym = (uint8_t)i;
        }
    }
    return 0;
}

static void init_cavlc_level_tab(void);

void h264_cavlc_init(void)
{
    int i, expected = 0;

    /* The tables are process-wide and identical for every decoder, but two
     * threads may open a decoder at the same time. The first one builds them
     * with the flag set to -1, and the others wait for the release store that
     * publishes the finished tables. */
    if (__atomic_load_n(&cavlc_initialized, __ATOMIC_ACQUIRE))
        return;
    if (!__atomic_compare_exchange_n(&cavlc_initialized, &expected, -1, 0,
                                     __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
        while (__atomic_load_n(&cavlc_initialized, __ATOMIC_ACQUIRE) != 1)
            ;
        return;
    }
    for (i = 0; i < 4; i++)
        build_vlc(coeff_token_lut[i], COEFF_TOKEN_BITS,
                  coeff_token_len[i], coeff_token_bits[i], 4 * 17);
    build_vlc(chroma_dc_token_lut, CHROMA_DC_TOKEN_BITS,
              chroma_dc_coeff_token_len, chroma_dc_coeff_token_bits, 4 * 5);
    build_vlc(chroma422_dc_token_lut, CHROMA422_DC_TOKEN_BITS,
              chroma422_dc_coeff_token_len, chroma422_dc_coeff_token_bits, 4 * 9);
    for (i = 0; i < 15; i++)
        build_vlc(total_zeros_lut[i + 1], TOTAL_ZEROS_BITS,
                  total_zeros_len[i], total_zeros_bits[i], 16);
    for (i = 0; i < 3; i++)
        build_vlc(chroma_dc_zeros_lut[i + 1], CHROMA_DC_ZEROS_BITS,
                  chroma_dc_total_zeros_len[i], chroma_dc_total_zeros_bits[i], 4);
    for (i = 0; i < 7; i++)
        build_vlc(chroma422_dc_zeros_lut[i + 1], CHROMA422_DC_ZEROS_BITS,
                  chroma422_dc_total_zeros_len[i], chroma422_dc_total_zeros_bits[i], 8);
    for (i = 0; i < 7; i++)
        build_vlc(run_lut[i], RUN7_BITS, run_len[i], run_bits[i], 16);
    init_cavlc_level_tab();
    __atomic_store_n(&cavlc_initialized, 1, __ATOMIC_RELEASE);
}

/* ------------------------------------------------------------------ */
/* bit helpers                                                         */
/* ------------------------------------------------------------------ */
static unsigned peek_bits(H264BitReader *br, int n)
{
    unsigned v = 0;
    int i;
    int pos = br->pos;
    for (i = 0; i < n; i++) {
        int p = pos + i;
        v <<= 1;
        if (p < br->size_bits)
            v |= (br->buffer[p >> 3] >> (7 - (p & 7))) & 1;
    }
    return v;
}

static int get_vlc(H264BitReader *br, const H264VLCEntry *table, int bits, const char *what)
{
    unsigned peek = peek_bits(br, bits);
    H264VLCEntry e = table[peek];
    if (!e.len) {
        br->error = 1;
        fprintf(stderr, "[h264dec] invalid VLC (%s)\n", what);
        return -1;
    }
    h264_br_skip(br, e.len);
    return e.sym;
}

static int av_log2_u(unsigned v)
{
    int n = 0;
    while (v >>= 1)
        n++;
    return n;
}

/* level decoding table (matches FFmpeg's cavlc_level_tab) */
#define LEVEL_TAB_BITS 8
static int8_t cavlc_level_tab[7][1 << LEVEL_TAB_BITS][2];

static void init_cavlc_level_tab(void)
{
    int suffix_length;
    unsigned int i;
    for (suffix_length = 0; suffix_length < 7; suffix_length++) {
        for (i = 0; i < (1 << LEVEL_TAB_BITS); i++) {
            int prefix = LEVEL_TAB_BITS - av_log2_u(2 * i);
            if (prefix + 1 + suffix_length <= LEVEL_TAB_BITS) {
                int level_code = (prefix << suffix_length) +
                    (i >> (av_log2_u(i) - suffix_length)) - (1 << suffix_length);
                int mask = -(level_code & 1);
                level_code = (((2 + level_code) >> 1) ^ mask) - mask;
                cavlc_level_tab[suffix_length][i][0] = (int8_t)level_code;
                cavlc_level_tab[suffix_length][i][1] = (int8_t)(prefix + 1 + suffix_length);
            } else if (prefix + 1 <= LEVEL_TAB_BITS) {
                cavlc_level_tab[suffix_length][i][0] = (int8_t)(prefix + 100);
                cavlc_level_tab[suffix_length][i][1] = (int8_t)(prefix + 1);
            } else {
                cavlc_level_tab[suffix_length][i][0] = LEVEL_TAB_BITS + 100;
                cavlc_level_tab[suffix_length][i][1] = LEVEL_TAB_BITS;
            }
        }
    }
}

static int get_level_prefix(H264BitReader *br)
{
    int prefix = 0;
    while (!h264_br_get_bits1(br)) {
        prefix++;
        if (prefix > 31) {
            br->error = 1;
            return 31;
        }
    }
    return prefix;
}

static int pred_non_zero_count(H264Slice *sl, int n)
{
    const int index8 = h264_scan8[n];
    const int left = sl->non_zero_count_cache[index8 - 1];
    const int top = sl->non_zero_count_cache[index8 - 8];
    int i = left + top;
    if (i < 64)
        i = (i + 1) >> 1;
    return i & 31;
}

/* ------------------------------------------------------------------ */
/* residual                                                            */
/* ------------------------------------------------------------------ */
/* a damaged level can be large enough that level * qmul overflows int; the
 * dequantisation is exact in 64 bits and the stored result is truncated to
 * the block's sample type either way */
static inline int cavlc_dequant(int level, int qmul)
{
    return (int)((int32_t)((uint32_t)level * (uint32_t)qmul + 32u) >> 6);
}

static int decode_residual(H264Decoder *h, H264Slice *sl, int16_t *block, int n,
                           const uint8_t *scantable, const uint32_t *qmul,
                           int max_coeff)
{
    int level[16];
    int zeros_left, coeff_token, total_coeff, i, trailing_ones, run_before;
    /* High 10 writes 32-bit coefficients into the same scratch array; the
     * caller has already advanced the pointer by 16 samples per block. */
#define PUT_COEF(j_, v_) do { \
        if (sl->pixel_shift) \
            ((int32_t *)block)[j_] = (int32_t)(v_); \
        else \
            block[j_] = (int16_t)(v_); \
    } while (0)
    H264BitReader *br = &sl->gb;

    if (max_coeff <= 8) {
        if (max_coeff == 4)
            coeff_token = get_vlc(br, chroma_dc_token_lut, CHROMA_DC_TOKEN_BITS, "chroma dc token");
        else
            coeff_token = get_vlc(br, chroma422_dc_token_lut, CHROMA422_DC_TOKEN_BITS,
                                  "chroma422 dc token");
    } else {
        int nc = pred_non_zero_count(sl, n >= 48 ? (n - 48) * 16 : n);
        static const uint8_t coeff_token_tab_index[17] = {
            0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3
        };
        coeff_token = get_vlc(br, coeff_token_lut[coeff_token_tab_index[nc]],
                              COEFF_TOKEN_BITS, "coeff token");
    }
    if (coeff_token < 0)
        return -1;
    total_coeff = coeff_token >> 2;
    sl->non_zero_count_cache[h264_scan8[n]] = (uint8_t)total_coeff;

    if (total_coeff == 0) {
        return 0;
    }
    if (total_coeff > max_coeff) {
        fprintf(stderr, "[h264dec] corrupted MB %d %d (total_coeff=%d)\n",
                sl->mb_x, sl->mb_y, total_coeff);
        return -1;
    }

    trailing_ones = coeff_token & 3;
    i = (int)peek_bits(br, 3);
    h264_br_skip(br, trailing_ones);
    level[0] = 1 - ((i & 4) >> 1);
    level[1] = 1 - ((i & 2));
    level[2] = 1 - ((i & 1) << 1);

    if (trailing_ones < total_coeff) {
        int mask, prefix;
        int suffix_length = (total_coeff > 10) & (trailing_ones < 3);
        int bitsi = (int)peek_bits(br, LEVEL_TAB_BITS);
        int level_code = cavlc_level_tab[suffix_length][bitsi][0];

        h264_br_skip(br, cavlc_level_tab[suffix_length][bitsi][1]);
        if (level_code >= 100) {
            prefix = level_code - 100;
            if (prefix == LEVEL_TAB_BITS)
                prefix += get_level_prefix(br);

            if (prefix < 14) {
                if (suffix_length)
                    level_code = (prefix << 1) + (int)h264_br_get_bits1(br);
                else
                    level_code = prefix;
            } else if (prefix == 14) {
                if (suffix_length)
                    level_code = (prefix << 1) + (int)h264_br_get_bits1(br);
                else
                    level_code = prefix + (int)h264_br_get_bits(br, 4);
            } else {
                level_code = 30;
                if (prefix >= 16) {
                    if (prefix > 25 + 3) {
                        fprintf(stderr, "[h264dec] invalid level prefix\n");
                        return -1;
                    }
                    level_code += (1 << (prefix - 3)) - 4096;
                }
                level_code += (int)h264_br_get_bits(br, prefix - 3);
            }
            if (trailing_ones < 3)
                level_code += 2;
            suffix_length = 2;
            mask = -(level_code & 1);
            level[trailing_ones] = (((2 + level_code) >> 1) ^ mask) - mask;
        } else {
            level_code += ((level_code >> 31) | 1) & -(trailing_ones < 3);
            suffix_length = 1 + (level_code + 3U > 6U);
            level[trailing_ones] = level_code;
        }

        for (i = trailing_ones + 1; i < total_coeff; i++) {
            static const unsigned suffix_limit[7] = { 0, 3, 6, 12, 24, 48, 0x7fffffff };
            int bitsi = (int)peek_bits(br, LEVEL_TAB_BITS);
            level_code = cavlc_level_tab[suffix_length][bitsi][0];
            h264_br_skip(br, cavlc_level_tab[suffix_length][bitsi][1]);
            if (level_code >= 100) {
                prefix = level_code - 100;
                if (prefix == LEVEL_TAB_BITS)
                    prefix += get_level_prefix(br);
                if (prefix < 15) {
                    level_code = (prefix << suffix_length) +
                                 (int)h264_br_get_bits(br, suffix_length);
                } else {
                    level_code = 15 << suffix_length;
                    if (prefix >= 16) {
                        if (prefix > 25 + 3) {
                            fprintf(stderr, "[h264dec] invalid level prefix\n");
                            return -1;
                        }
                        level_code += (1 << (prefix - 3)) - 4096;
                    }
                    level_code += (int)h264_br_get_bits(br, prefix - 3);
                }
                mask = -(level_code & 1);
                level_code = (((2 + level_code) >> 1) ^ mask) - mask;
            }
            level[i] = level_code;
            suffix_length += suffix_limit[suffix_length] + level_code >
                            2U * suffix_limit[suffix_length];
        }
    }

    if (total_coeff == max_coeff) {
        zeros_left = 0;
    } else {
        if (max_coeff <= 8) {
            if (max_coeff == 4)
                zeros_left = get_vlc(br, chroma_dc_zeros_lut[total_coeff],
                                     CHROMA_DC_ZEROS_BITS, "chroma dc zeros");
            else
                zeros_left = get_vlc(br, chroma422_dc_zeros_lut[total_coeff],
                                     CHROMA422_DC_ZEROS_BITS, "chroma422 dc zeros");
        } else {
            zeros_left = get_vlc(br, total_zeros_lut[total_coeff],
                                 TOTAL_ZEROS_BITS, "total zeros");
        }
        if (zeros_left < 0)
            return -1;
    }

    scantable += zeros_left + total_coeff - 1;
    if (n >= 48) {
        PUT_COEF(*scantable, level[0]);
        for (i = 1; i < total_coeff && zeros_left > 0; i++) {
            if (zeros_left < 7)
                run_before = get_vlc(br, run_lut[zeros_left - 1], RUN7_BITS, "run");
            else
                run_before = get_vlc(br, run_lut[6], RUN7_BITS, "run7");
            if (run_before < 0)
                return -1;
            zeros_left -= run_before;
            scantable -= 1 + run_before;
            PUT_COEF(*scantable, level[i]);
        }
        for (; i < total_coeff; i++) {
            scantable--;
            PUT_COEF(*scantable, level[i]);
        }
    } else {
        PUT_COEF(*scantable, cavlc_dequant(level[0], (int)qmul[*scantable]));
        for (i = 1; i < total_coeff && zeros_left > 0; i++) {
            if (zeros_left < 7)
                run_before = get_vlc(br, run_lut[zeros_left - 1], RUN7_BITS, "run");
            else
                run_before = get_vlc(br, run_lut[6], RUN7_BITS, "run7");
            if (run_before < 0)
                return -1;
            zeros_left -= run_before;
            scantable -= 1 + run_before;
            PUT_COEF(*scantable, cavlc_dequant(level[i], (int)qmul[*scantable]));
        }
        for (; i < total_coeff; i++) {
            scantable--;
            PUT_COEF(*scantable, cavlc_dequant(level[i], (int)qmul[*scantable]));
        }
    }

    if (zeros_left < 0) {
        fprintf(stderr, "[h264dec] negative zeros left\n");
        return -1;
    }
    return br->error ? -1 : 0;
}

static int decode_luma_residual(H264Decoder *h, H264Slice *sl, const uint8_t *scan,
                                const uint8_t *scan8x8, uint32_t mb_type, int cbp,
                                int p)
{
    int i4x4, i8x8;
    const int qscale = p == 0 ? sl->qscale : sl->chroma_qp[p - 1];
    H264PPS *pps = h->active_pps;
    const int nnz_off = 16 * p;
    const int coef_off = p * 256;

    if (IS_INTRA16x16(mb_type)) {
        memset(sl->mb_luma_dc[p], 0, sizeof(sl->mb_luma_dc[p]));
        if (decode_residual(h, sl, sl->mb_luma_dc[p], 48 + p, scan, NULL, 16) < 0)
            return -1;
        if (cbp & 15) {
            for (i8x8 = 0; i8x8 < 4; i8x8++) {
                for (i4x4 = 0; i4x4 < 4; i4x4++) {
                    const int index = i4x4 + 4 * i8x8;
                    if (decode_residual(h, sl,
                                        sl->mb + (((16 * index) + coef_off) << sl->pixel_shift),
                                        index + nnz_off, scan + 1,
                                        pps->dequant4_coeff[p][qscale], 15) < 0)
                        return -1;
                }
            }
            return 0xf;
        } else {
            memset(&sl->non_zero_count_cache[h264_scan8[nnz_off]], 0, 4);
            memset(&sl->non_zero_count_cache[h264_scan8[nnz_off] + 8], 0, 4);
            memset(&sl->non_zero_count_cache[h264_scan8[nnz_off] + 16], 0, 4);
            memset(&sl->non_zero_count_cache[h264_scan8[nnz_off] + 24], 0, 4);
            return 0;
        }
    } else {
        const int cqm = (IS_INTRA(mb_type) ? 0 : 3) + p;
        int new_cbp = 0;
        for (i8x8 = 0; i8x8 < 4; i8x8++) {
            if (cbp & (1 << i8x8)) {
                if (IS_8x8DCT(mb_type)) {
                    int16_t *buf = &sl->mb[((64 * i8x8) + coef_off) << sl->pixel_shift];
                    uint8_t *nnz;
                    for (i4x4 = 0; i4x4 < 4; i4x4++) {
                        const int index = i4x4 + 4 * i8x8;
                        if (decode_residual(h, sl, buf, index + nnz_off, scan8x8 + 16 * i4x4,
                                            pps->dequant8_coeff[cqm][qscale], 16) < 0)
                            return -1;
                    }
                    nnz = &sl->non_zero_count_cache[h264_scan8[4 * i8x8 + nnz_off]];
                    nnz[0] = (uint8_t)(nnz[0] + nnz[1] + nnz[8] + nnz[9]);
                    new_cbp |= !!nnz[0] << i8x8;
                } else {
                    for (i4x4 = 0; i4x4 < 4; i4x4++) {
                        const int index = i4x4 + 4 * i8x8;
                        if (decode_residual(h, sl,
                                            sl->mb + (((16 * index) + coef_off) << sl->pixel_shift),
                                            index + nnz_off, scan,
                                            pps->dequant4_coeff[cqm][qscale], 16) < 0)
                            return -1;
                        new_cbp |= sl->non_zero_count_cache[h264_scan8[index + nnz_off]] << i8x8;
                    }
                }
            } else {
                uint8_t *const nnz = &sl->non_zero_count_cache[h264_scan8[4 * i8x8 + nnz_off]];
                nnz[0] = nnz[1] = nnz[8] = nnz[9] = 0;
            }
        }
        return new_cbp;
    }
}

static int get_dct8x8_allowed(H264Decoder *h, H264Slice *sl)
{
    uint32_t v = (uint32_t)sl->sub_mb_type[0] | ((uint32_t)sl->sub_mb_type[1] << 16);
    uint32_t v2 = (uint32_t)sl->sub_mb_type[2] | ((uint32_t)sl->sub_mb_type[3] << 16);
    uint32_t mask = h->active_sps->direct_8x8_inference_flag ?
        (MB_TYPE_16x8 | MB_TYPE_8x16 | MB_TYPE_8x8) :
        (MB_TYPE_16x8 | MB_TYPE_8x16 | MB_TYPE_8x8 | MB_TYPE_DIRECT2);
    mask |= mask << 16;
    return !((v | v2) & mask);
}

/* ------------------------------------------------------------------ */
/* macroblock                                                          */
/* ------------------------------------------------------------------ */
int h264_decode_mb_cavlc(H264Decoder *h, H264Slice *sl)
{
    H264Frame *f = h->cur_pic;
    const int mb_xy = sl->mb_xy;
    int partition_count;
    uint32_t mb_type;
    int cbp = 0;
    int dct8x8_allowed = h->active_pps->transform_8x8_mode;
    const int max_qp = 51 + 6 * (h->active_sps->bit_depth_luma - 8);
    H264BitReader *br = &sl->gb;

    if (sl->slice_type_nos != H264_SLICE_I) {
        if (sl->mb_skip_run < 0) {
            sl->mb_skip_run = (int)h264_br_get_ue(br);
            if (br->error)
                return -1;
        }
        if (sl->mb_skip_run--) {
            if (f->mbaff && (sl->mb_y & 1) == 0) {
                if (sl->mb_skip_run == 0)
                    sl->mb_mbaff = sl->mb_field_decoding_flag = (int)h264_br_get_bits1(br);
            }
            h264_decode_mb_skip(h, sl);
            return 0;
        }
    }
    if (f->mbaff && (sl->mb_y & 1) == 0)
        sl->mb_mbaff = sl->mb_field_decoding_flag = (int)h264_br_get_bits1(br);

    sl->prev_mb_skipped = 0;

    mb_type = h264_br_get_ue(br);
    if (sl->slice_type_nos == H264_SLICE_B) {
        if (mb_type < 23) {
            partition_count = h264_b_mb_type_info[mb_type].partition_count;
            mb_type = h264_b_mb_type_info[mb_type].type;
        } else {
            mb_type -= 23;
            goto decode_intra_mb;
        }
    } else if (sl->slice_type_nos == H264_SLICE_P) {
        if (mb_type < 5) {
            partition_count = h264_p_mb_type_info[mb_type].partition_count;
            mb_type = h264_p_mb_type_info[mb_type].type;
        } else {
            mb_type -= 5;
            goto decode_intra_mb;
        }
    } else {
decode_intra_mb:
        if (mb_type > 25)
            return -1;
        partition_count = 0;
        cbp = h264_i_mb_type_info[mb_type].cbp;
        sl->intra16x16_pred_mode = h264_i_mb_type_info[mb_type].pred_mode;
        mb_type = h264_i_mb_type_info[mb_type].type;
    }

    if (MB_FIELD(sl))
        mb_type |= MB_TYPE_INTERLACED;

    f->slice_table[mb_xy] = (uint16_t)sl->slice_num;
    f->mb_skip[mb_xy] = 0;

    if (IS_INTRA_PCM(mb_type)) {
        const int cidc = h->active_sps->chroma_format_idc;
        const int samples = cidc == 3 ? 768 : cidc == 2 ? 512 : cidc ? 384 : 256;
        const int mb_size = samples * h->active_sps->bit_depth_luma;
        sl->intra_pcm_ptr = br->buffer + ((br->pos + 7) >> 3);
        h264_br_align(br);
        sl->intra_pcm_ptr = br->buffer + (br->pos >> 3);
        if (h264_br_bits_left(br) < mb_size)
            return -1;
        h264_br_skip(br, mb_size);
        f->qscale_table[mb_xy] = 0;
        memset(f->nnz[mb_xy], 16, 48);
        f->mb_type[mb_xy] = mb_type;
        f->intra_only = 1;
        return 0;
    }

    h264_fill_decode_neighbors(h, sl, mb_type);
    h264_fill_decode_caches(h, sl, mb_type);

    if (IS_INTRA(mb_type)) {
        if (IS_INTRA4x4(mb_type)) {
            int i;
            int di = 1;
            if (dct8x8_allowed && h264_br_get_bits1(br)) {
                mb_type |= MB_TYPE_8x8DCT;
                di = 4;
            }
            for (i = 0; i < 16; i += di) {
                int mode = 2;
                int left = sl->intra4x4_pred_mode_cache[h264_scan8[i] - 1];
                int top = sl->intra4x4_pred_mode_cache[h264_scan8[i] - 8];
                mode = H264_MIN(left, top);
                if (mode < 0)
                    mode = 2;
                if (!h264_br_get_bits1(br)) {
                    const int rem_mode = h264_br_get_bits(br, 3);
                    mode = rem_mode + (rem_mode >= mode);
                }
                if (di == 4) {
                    sl->intra4x4_pred_mode_cache[h264_scan8[i]] = (int8_t)mode;
                    sl->intra4x4_pred_mode_cache[h264_scan8[i] + 1] = (int8_t)mode;
                    sl->intra4x4_pred_mode_cache[h264_scan8[i] + 8] = (int8_t)mode;
                    sl->intra4x4_pred_mode_cache[h264_scan8[i] + 9] = (int8_t)mode;
                } else {
                    sl->intra4x4_pred_mode_cache[h264_scan8[i]] = (int8_t)mode;
                }
            }
            h264_write_back_intra_pred_mode(h, sl);
            if (h264_check_intra4x4_pred_mode(sl->intra4x4_pred_mode_cache,
                                              sl->top_samples_available,
                                              sl->left_samples_available) < 0)
                return -1;
        } else {
            sl->intra16x16_pred_mode = h264_check_intra_pred_mode(
                sl->top_samples_available, sl->left_samples_available,
                sl->intra16x16_pred_mode, 0);
            if (sl->intra16x16_pred_mode < 0)
                return -1;
        }
        if (h->active_sps->chroma_format_idc == 1 ||
            h->active_sps->chroma_format_idc == 2) {
            sl->chroma_pred_mode = h264_check_intra_pred_mode(
                sl->top_samples_available, sl->left_samples_available,
                (int)h264_br_get_ue(br), 1);
            if (sl->chroma_pred_mode < 0)
                return -1;
            f->chroma_pred_mode[mb_xy] = (uint8_t)sl->chroma_pred_mode;
        } else {
            sl->chroma_pred_mode = DC_128_PRED8x8;
            f->chroma_pred_mode[mb_xy] = DC_128_PRED8x8;
        }
    } else if (partition_count == 4) {
        int i, j, sub_partition_count[4], list, ref[2][4];

        if (sl->slice_type_nos == H264_SLICE_B) {
            for (i = 0; i < 4; i++) {
                sl->sub_mb_type[i] = (uint16_t)h264_br_get_ue(br);
                if (sl->sub_mb_type[i] >= 13)
                    return -1;
                sub_partition_count[i] = h264_b_sub_mb_type_info[sl->sub_mb_type[i]].partition_count;
                sl->sub_mb_type[i] = h264_b_sub_mb_type_info[sl->sub_mb_type[i]].type;
            }
            if (IS_DIRECT(sl->sub_mb_type[0] | sl->sub_mb_type[1] |
                          sl->sub_mb_type[2] | sl->sub_mb_type[3])) {
                h264_pred_direct_motion(h, sl, &mb_type);
                sl->ref_cache[0][h264_scan8[4]] = sl->ref_cache[1][h264_scan8[4]] =
                sl->ref_cache[0][h264_scan8[12]] = sl->ref_cache[1][h264_scan8[12]] =
                    PART_NOT_AVAILABLE;
            }
        } else {
            for (i = 0; i < 4; i++) {
                sl->sub_mb_type[i] = (uint16_t)h264_br_get_ue(br);
                if (sl->sub_mb_type[i] >= 4)
                    return -1;
                sub_partition_count[i] = h264_p_sub_mb_type_info[sl->sub_mb_type[i]].partition_count;
                sl->sub_mb_type[i] = h264_p_sub_mb_type_info[sl->sub_mb_type[i]].type;
            }
        }

        for (list = 0; list < sl->list_count; list++) {
            int ref_count = IS_REF0(mb_type) ? 1 : sl->ref_count[list] << MB_MBAFF(sl);
            for (i = 0; i < 4; i++) {
                if (IS_DIRECT(sl->sub_mb_type[i]))
                    continue;
                if (IS_DIR(sl->sub_mb_type[i], 0, list)) {
                    unsigned int tmp;
                    if (ref_count == 1) {
                        tmp = 0;
                    } else if (ref_count == 2) {
                        tmp = h264_br_get_bits1(br) ^ 1;
                    } else {
                        tmp = h264_br_get_ue(br);
                        if ((int)tmp >= ref_count)
                            return -1;
                    }
                    ref[list][i] = (int)tmp;
                } else {
                    ref[list][i] = -1;
                }
            }
        }

        if (dct8x8_allowed)
            dct8x8_allowed = get_dct8x8_allowed(h, sl);

        for (list = 0; list < sl->list_count; list++) {
            for (i = 0; i < 4; i++) {
                if (IS_DIRECT(sl->sub_mb_type[i])) {
                    sl->ref_cache[list][h264_scan8[4 * i]] =
                        sl->ref_cache[list][h264_scan8[4 * i] + 1];
                    continue;
                }
                sl->ref_cache[list][h264_scan8[4 * i]] =
                sl->ref_cache[list][h264_scan8[4 * i] + 1] =
                sl->ref_cache[list][h264_scan8[4 * i] + 8] =
                sl->ref_cache[list][h264_scan8[4 * i] + 9] = (int8_t)ref[list][i];

                if (IS_DIR(sl->sub_mb_type[i], 0, list)) {
                    const int sub_mb_type = sl->sub_mb_type[i];
                    const int block_width = (sub_mb_type & (MB_TYPE_16x16 | MB_TYPE_16x8)) ? 2 : 1;
                    for (j = 0; j < sub_partition_count[i]; j++) {
                        int mx, my;
                        const int index = 4 * i + block_width * j;
                        int16_t (*mv_cache)[2] = &sl->mv_cache[list][h264_scan8[index]];
                        h264_pred_motion(h, sl, index, block_width, list,
                                         sl->ref_cache[list][h264_scan8[index]], &mx, &my);
                        mx += h264_br_get_se(br);
                        my += h264_br_get_se(br);
                        if (IS_SUB_8X8(sub_mb_type)) {
                            mv_cache[1][0] = mv_cache[8][0] = mv_cache[9][0] = (int16_t)mx;
                            mv_cache[1][1] = mv_cache[8][1] = mv_cache[9][1] = (int16_t)my;
                        } else if (IS_SUB_8X4(sub_mb_type)) {
                            mv_cache[1][0] = (int16_t)mx;
                            mv_cache[1][1] = (int16_t)my;
                        } else if (IS_SUB_4X8(sub_mb_type)) {
                            mv_cache[8][0] = (int16_t)mx;
                            mv_cache[8][1] = (int16_t)my;
                        }
                        mv_cache[0][0] = (int16_t)mx;
                        mv_cache[0][1] = (int16_t)my;
                    }
                } else {
                    uint32_t *p = (uint32_t *)&sl->mv_cache[list][h264_scan8[4 * i]][0];
                    p[0] = p[1] = p[8] = p[9] = 0;
                }
            }
        }
    } else if (IS_DIRECT(mb_type)) {
        h264_pred_direct_motion(h, sl, &mb_type);
        dct8x8_allowed &= h->active_sps->direct_8x8_inference_flag;
    } else {
        int list, mx, my, i;
        if (IS_16X16(mb_type)) {
            for (list = 0; list < sl->list_count; list++) {
                unsigned int val;
                if (IS_DIR(mb_type, 0, list)) {
                    int rc = sl->ref_count[list] << MB_MBAFF(sl);
                    if (rc == 1) val = 0;
                    else if (rc == 2) val = h264_br_get_bits1(br) ^ 1;
                    else {
                        val = h264_br_get_ue(br);
                        if (val >= (unsigned)rc)
                            return -1;
                    }
                    /* 4x4 grid of cache slots, stride 8 (not contiguous!) */
                    for (int k = 0; k < 4; k++) {
                        int8_t *r = &sl->ref_cache[list][h264_scan8[0] + 8 * k];
                        r[0] = r[1] = r[2] = r[3] = (int8_t)val;
                    }
                }
            }
            for (list = 0; list < sl->list_count; list++) {
                if (IS_DIR(mb_type, 0, list)) {
                    h264_pred_motion(h, sl, 0, 4, list,
                                     sl->ref_cache[list][h264_scan8[0]], &mx, &my);
                    mx += h264_br_get_se(br);
                    my += h264_br_get_se(br);
                    for (i = 0; i < 4; i++) {
                        int16_t (*dst)[2] = &sl->mv_cache[list][h264_scan8[0] + 8 * i];
                        int k;
                        for (k = 0; k < 4; k++) {
                            dst[k][0] = (int16_t)mx;
                            dst[k][1] = (int16_t)my;
                        }
                    }
                }
            }
        } else if (IS_16X8(mb_type)) {
            for (list = 0; list < sl->list_count; list++) {
                for (i = 0; i < 2; i++) {
                    unsigned int val;
                    if (IS_DIR(mb_type, i, list)) {
                        int rc = sl->ref_count[list] << MB_MBAFF(sl);
                        if (rc == 1) val = 0;
                        else if (rc == 2) val = h264_br_get_bits1(br) ^ 1;
                        else {
                            val = h264_br_get_ue(br);
                            if (val >= (unsigned)rc)
                                return -1;
                        }
                    } else {
                        val = (unsigned)LIST_NOT_USED & 0xFF;
                    }
                    for (int k = 0; k < 2; k++) {
                        int8_t *r = &sl->ref_cache[list][h264_scan8[0] + 16 * i + 8 * k];
                        r[0] = r[1] = r[2] = r[3] = (int8_t)val;
                    }
                }
            }
            for (list = 0; list < sl->list_count; list++) {
                for (i = 0; i < 2; i++) {
                    if (IS_DIR(mb_type, i, list)) {
                        h264_pred_16x8_motion(h, sl, 8 * i, list,
                                              sl->ref_cache[list][h264_scan8[0] + 16 * i],
                                              &mx, &my);
                        mx += h264_br_get_se(br);
                        my += h264_br_get_se(br);
                    } else {
                        mx = my = 0;
                    }
                    for (int k = 0; k < 2; k++) {
                        int16_t (*dst)[2] = &sl->mv_cache[list][h264_scan8[0] + 16 * i + 8 * k];
                        int j;
                        for (j = 0; j < 4; j++) {
                            dst[j][0] = (int16_t)mx;
                            dst[j][1] = (int16_t)my;
                        }
                    }
                }
            }
        } else {
            for (list = 0; list < sl->list_count; list++) {
                for (i = 0; i < 2; i++) {
                    unsigned int val;
                    if (IS_DIR(mb_type, i, list)) {
                        int rc = sl->ref_count[list] << MB_MBAFF(sl);
                        if (rc == 1) val = 0;
                        else if (rc == 2) val = h264_br_get_bits1(br) ^ 1;
                        else {
                            val = h264_br_get_ue(br);
                            if (val >= (unsigned)rc)
                                return -1;
                        }
                    } else {
                        val = (unsigned)LIST_NOT_USED & 0xFF;
                    }
                    for (int k = 0; k < 4; k++) {
                        int8_t *r = &sl->ref_cache[list][h264_scan8[0] + 2 * i + 8 * k];
                        r[0] = r[1] = (int8_t)val;
                    }
                }
            }
            for (list = 0; list < sl->list_count; list++) {
                for (i = 0; i < 2; i++) {
                    if (IS_DIR(mb_type, i, list)) {
                        h264_pred_8x16_motion(h, sl, i * 4, list,
                                              sl->ref_cache[list][h264_scan8[0] + 2 * i],
                                              &mx, &my);
                        mx += h264_br_get_se(br);
                        my += h264_br_get_se(br);
                    } else {
                        mx = my = 0;
                    }
                    for (int k = 0; k < 4; k++) {
                        int16_t (*dst)[2] = &sl->mv_cache[list][h264_scan8[0] + 2 * i + 8 * k];
                        int j;
                        for (j = 0; j < 2; j++) {
                            dst[j][0] = (int16_t)mx;
                            dst[j][1] = (int16_t)my;
                        }
                    }
                }
            }
        }
    }

    if (IS_INTER(mb_type))
        h264_write_back_motion(h, sl, mb_type);

    if (!IS_INTRA16x16(mb_type)) {
        cbp = (int)h264_br_get_ue(br);
        if (h->active_sps->chroma_format_idc == 1 ||
            h->active_sps->chroma_format_idc == 2) {
            if (cbp > 47)
                return -1;
            cbp = IS_INTRA4x4(mb_type) ? h264_golomb_to_intra4x4_cbp[cbp]
                                       : h264_golomb_to_inter_cbp[cbp];
        } else {
            if (cbp > 15)
                return -1;
            cbp = IS_INTRA4x4(mb_type) ? h264_golomb_to_intra4x4_cbp_gray[cbp]
                                       : h264_golomb_to_inter_cbp_gray[cbp];
        }
    }

    if (dct8x8_allowed && (cbp & 15) && !IS_INTRA(mb_type))
        mb_type |= MB_TYPE_8x8DCT * h264_br_get_bits1(br);

    sl->cbp = cbp;
    f->cbp_table[mb_xy] = (uint16_t)cbp;
    f->mb_type[mb_xy] = mb_type;

    if (cbp || IS_INTRA16x16(mb_type)) {
        int dquant = h264_br_get_se(br);
        const uint8_t *scan, *scan8x8;
        int ret;

        sl->qscale += dquant;
        if (((unsigned)sl->qscale) > (unsigned)max_qp) {
            if (sl->qscale < 0) sl->qscale += max_qp + 1;
            else sl->qscale -= max_qp + 1;
            if (((unsigned)sl->qscale) > (unsigned)max_qp)
                return -1;
        }
        sl->chroma_qp[0] = h264_get_chroma_qp(h->active_pps, 0, sl->qscale);
        sl->chroma_qp[1] = h264_get_chroma_qp(h->active_pps, 1, sl->qscale);

        if (IS_INTERLACED(mb_type)) {
            scan = sl->qscale ? h->field_scan : h->field_scan_q0;
            scan8x8 = sl->qscale ? h->field_scan8x8_cavlc : h->field_scan8x8_cavlc_q0;
        } else {
            scan = sl->qscale ? h->zigzag_scan : h->zigzag_scan_q0;
            scan8x8 = sl->qscale ? h->zigzag_scan8x8_cavlc : h->zigzag_scan8x8_cavlc_q0;
        }

        if ((ret = decode_luma_residual(h, sl, scan, scan8x8, mb_type, cbp, 0)) < 0)
            return -1;
        f->cbp_table[mb_xy] |= (uint16_t)(ret << 12);

        if (h->active_sps->chroma_format_idc == 3) {
            if (decode_luma_residual(h, sl, scan, scan8x8, mb_type, cbp, 1) < 0)
                return -1;
            if (decode_luma_residual(h, sl, scan, scan8x8, mb_type, cbp, 2) < 0)
                return -1;
        } else {
            const int cidc = h->active_sps->chroma_format_idc;
            const int num_c8x8 = cidc;
            if (cbp & 0x30) {
                int chroma_idx;
                for (chroma_idx = 0; chroma_idx < 2; chroma_idx++) {
                    if (decode_residual(h, sl, sl->mb + ((256 + 256 * chroma_idx) << sl->pixel_shift),
                                        49 + chroma_idx,
                                        cidc == 2 ? h264_chroma422_dc_scan : h264_chroma_dc_scan,
                                        NULL, 4 * num_c8x8) < 0)
                        return -1;
                }
            }
            if (cbp & 0x20) {
                int chroma_idx, i8x8, i4x4;
                for (chroma_idx = 0; chroma_idx < 2; chroma_idx++) {
                    const uint32_t *qmul = h->active_pps->dequant4_coeff[
                        chroma_idx + 1 + (IS_INTRA(mb_type) ? 0 : 3)][sl->chroma_qp[chroma_idx]];
                    int16_t *mb = sl->mb + ((256 + 256 * chroma_idx) << sl->pixel_shift);
                    for (i8x8 = 0; i8x8 < num_c8x8; i8x8++) {
                        for (i4x4 = 0; i4x4 < 4; i4x4++) {
                            const int index = 16 + 16 * chroma_idx + 8 * i8x8 + i4x4;
                            if (decode_residual(h, sl, mb, index, scan + 1, qmul, 15) < 0)
                                return -1;
                            mb += 16 << sl->pixel_shift;
                        }
                    }
                }
            } else {
                memset(&sl->non_zero_count_cache[h264_scan8[16]], 0, 4);
                memset(&sl->non_zero_count_cache[h264_scan8[16] + 8], 0, 4);
                memset(&sl->non_zero_count_cache[h264_scan8[16] + 16], 0, 4);
                memset(&sl->non_zero_count_cache[h264_scan8[16] + 24], 0, 4);
                memset(&sl->non_zero_count_cache[h264_scan8[32]], 0, 4);
                memset(&sl->non_zero_count_cache[h264_scan8[32] + 8], 0, 4);
                memset(&sl->non_zero_count_cache[h264_scan8[32] + 16], 0, 4);
                memset(&sl->non_zero_count_cache[h264_scan8[32] + 24], 0, 4);
            }
        }
    } else {
        memset(&sl->non_zero_count_cache[h264_scan8[0]], 0, 4);
        memset(&sl->non_zero_count_cache[h264_scan8[0] + 8], 0, 4);
        memset(&sl->non_zero_count_cache[h264_scan8[0] + 16], 0, 4);
        memset(&sl->non_zero_count_cache[h264_scan8[0] + 24], 0, 4);
        memset(&sl->non_zero_count_cache[h264_scan8[16]], 0, 4);
        memset(&sl->non_zero_count_cache[h264_scan8[16] + 8], 0, 4);
        memset(&sl->non_zero_count_cache[h264_scan8[16] + 16], 0, 4);
        memset(&sl->non_zero_count_cache[h264_scan8[16] + 24], 0, 4);
        memset(&sl->non_zero_count_cache[h264_scan8[32]], 0, 4);
        memset(&sl->non_zero_count_cache[h264_scan8[32] + 8], 0, 4);
        memset(&sl->non_zero_count_cache[h264_scan8[32] + 16], 0, 4);
        memset(&sl->non_zero_count_cache[h264_scan8[32] + 24], 0, 4);
    }

    f->qscale_table[mb_xy] = (int8_t)sl->qscale;
    if (!IS_INTRA(mb_type))
        f->intra_only = 0;
    h264_write_back_non_zero_count(h, sl);

    return br->error ? -1 : 0;
}
