/*
 * Constant tables for the H.264 decoder.
 * Values from ITU-T H.264 and FFmpeg's h264data.c (LGPL-2.1+).
 */
#include "h264_internal.h"

const uint8_t h264_golomb_to_intra4x4_cbp[48] = {
    47, 31, 15, 0,  23, 27, 29, 30, 7,  11, 13, 14, 39, 43, 45, 46,
    16, 3,  5,  10, 12, 19, 21, 26, 28, 35, 37, 42, 44, 1,  2,  4,
    8,  17, 18, 20, 24, 6,  9,  22, 25, 32, 33, 34, 36, 40, 38, 41
};

const uint8_t h264_golomb_to_inter_cbp[48] = {
    0,  16, 1,  2,  4,  8,  32, 3,  5,  10, 12, 15, 47, 7,  11, 13,
    14, 6,  9,  31, 35, 37, 42, 44, 33, 34, 36, 40, 39, 43, 45, 46,
    17, 18, 20, 24, 19, 21, 26, 28, 23, 27, 29, 30, 22, 25, 38, 41
};

const uint8_t h264_golomb_to_inter_cbp_gray[16] = {
    0, 1, 2, 4, 8, 3, 5, 10, 12, 15, 7, 11, 13, 14, 6, 9,
};

const uint8_t h264_golomb_to_intra4x4_cbp_gray[16] = {
    15, 0, 7, 11, 13, 14, 3, 5, 10, 12, 1, 2, 4, 8, 6, 9,
};

const uint8_t h264_chroma_dc_scan[4] = {
    (0 + 0 * 2) * 16, (1 + 0 * 2) * 16,
    (0 + 1 * 2) * 16, (1 + 1 * 2) * 16,
};

const H264IMbInfo h264_i_mb_type_info[26] = {
    { MB_TYPE_INTRA4x4,  -1,  -1 },
    { MB_TYPE_INTRA16x16, 2,   0 },
    { MB_TYPE_INTRA16x16, 1,   0 },
    { MB_TYPE_INTRA16x16, 0,   0 },
    { MB_TYPE_INTRA16x16, 3,   0 },
    { MB_TYPE_INTRA16x16, 2,  16 },
    { MB_TYPE_INTRA16x16, 1,  16 },
    { MB_TYPE_INTRA16x16, 0,  16 },
    { MB_TYPE_INTRA16x16, 3,  16 },
    { MB_TYPE_INTRA16x16, 2,  32 },
    { MB_TYPE_INTRA16x16, 1,  32 },
    { MB_TYPE_INTRA16x16, 0,  32 },
    { MB_TYPE_INTRA16x16, 3,  32 },
    { MB_TYPE_INTRA16x16, 2,  15 +  0 },
    { MB_TYPE_INTRA16x16, 1,  15 +  0 },
    { MB_TYPE_INTRA16x16, 0,  15 +  0 },
    { MB_TYPE_INTRA16x16, 3,  15 +  0 },
    { MB_TYPE_INTRA16x16, 2,  15 + 16 },
    { MB_TYPE_INTRA16x16, 1,  15 + 16 },
    { MB_TYPE_INTRA16x16, 0,  15 + 16 },
    { MB_TYPE_INTRA16x16, 3,  15 + 16 },
    { MB_TYPE_INTRA16x16, 2,  15 + 32 },
    { MB_TYPE_INTRA16x16, 1,  15 + 32 },
    { MB_TYPE_INTRA16x16, 0,  15 + 32 },
    { MB_TYPE_INTRA16x16, 3,  15 + 32 },
    { MB_TYPE_INTRA_PCM,  -1, -1 },
};

const H264PMbInfo h264_p_mb_type_info[5] = {
    { MB_TYPE_16x16 | MB_TYPE_P0L0,                               1 },
    { MB_TYPE_16x8  | MB_TYPE_P0L0 | MB_TYPE_P1L0,                2 },
    { MB_TYPE_8x16  | MB_TYPE_P0L0 | MB_TYPE_P1L0,                2 },
    { MB_TYPE_8x8   | MB_TYPE_P0L0 | MB_TYPE_P1L0,                4 },
    { MB_TYPE_8x8   | MB_TYPE_P0L0 | MB_TYPE_P1L0 | MB_TYPE_REF0, 4 },
};

const H264PMbInfo h264_p_sub_mb_type_info[4] = {
    { MB_TYPE_16x16 | MB_TYPE_P0L0, 1 },
    { MB_TYPE_16x8  | MB_TYPE_P0L0, 2 },
    { MB_TYPE_8x16  | MB_TYPE_P0L0, 2 },
    { MB_TYPE_8x8   | MB_TYPE_P0L0, 4 },
};

const H264PMbInfo h264_b_mb_type_info[23] = {
    { MB_TYPE_DIRECT2 | MB_TYPE_L0L1,                                              1, },
    { MB_TYPE_16x16   | MB_TYPE_P0L0,                                              1, },
    { MB_TYPE_16x16   | MB_TYPE_P0L1,                                              1, },
    { MB_TYPE_16x16   | MB_TYPE_P0L0 | MB_TYPE_P0L1,                               1, },
    { MB_TYPE_16x8    | MB_TYPE_P0L0 | MB_TYPE_P1L0,                               2, },
    { MB_TYPE_8x16    | MB_TYPE_P0L0 | MB_TYPE_P1L0,                               2, },
    { MB_TYPE_16x8    | MB_TYPE_P0L1 | MB_TYPE_P1L1,                               2, },
    { MB_TYPE_8x16    | MB_TYPE_P0L1 | MB_TYPE_P1L1,                               2, },
    { MB_TYPE_16x8    | MB_TYPE_P0L0 | MB_TYPE_P1L1,                               2, },
    { MB_TYPE_8x16    | MB_TYPE_P0L0 | MB_TYPE_P1L1,                               2, },
    { MB_TYPE_16x8    | MB_TYPE_P0L1 | MB_TYPE_P1L0,                               2, },
    { MB_TYPE_8x16    | MB_TYPE_P0L1 | MB_TYPE_P1L0,                               2, },
    { MB_TYPE_16x8    | MB_TYPE_P0L0 | MB_TYPE_P1L0 | MB_TYPE_P1L1,                2, },
    { MB_TYPE_8x16    | MB_TYPE_P0L0 | MB_TYPE_P1L0 | MB_TYPE_P1L1,                2, },
    { MB_TYPE_16x8    | MB_TYPE_P0L1 | MB_TYPE_P1L0 | MB_TYPE_P1L1,                2, },
    { MB_TYPE_8x16    | MB_TYPE_P0L1 | MB_TYPE_P1L0 | MB_TYPE_P1L1,                2, },
    { MB_TYPE_16x8    | MB_TYPE_P0L0 | MB_TYPE_P0L1 | MB_TYPE_P1L0,                2, },
    { MB_TYPE_8x16    | MB_TYPE_P0L0 | MB_TYPE_P0L1 | MB_TYPE_P1L0,                2, },
    { MB_TYPE_16x8    | MB_TYPE_P0L0 | MB_TYPE_P0L1 | MB_TYPE_P1L1,                2, },
    { MB_TYPE_8x16    | MB_TYPE_P0L0 | MB_TYPE_P0L1 | MB_TYPE_P1L1,                2, },
    { MB_TYPE_16x8    | MB_TYPE_P0L0 | MB_TYPE_P0L1 | MB_TYPE_P1L0 | MB_TYPE_P1L1, 2, },
    { MB_TYPE_8x16    | MB_TYPE_P0L0 | MB_TYPE_P0L1 | MB_TYPE_P1L0 | MB_TYPE_P1L1, 2, },
    { MB_TYPE_8x8     | MB_TYPE_P0L0 | MB_TYPE_P0L1 | MB_TYPE_P1L0 | MB_TYPE_P1L1, 4, },
};

const H264PMbInfo h264_b_sub_mb_type_info[13] = {
    { MB_TYPE_DIRECT2,                                                           1, },
    { MB_TYPE_16x16 | MB_TYPE_P0L0,                                              1, },
    { MB_TYPE_16x16 | MB_TYPE_P0L1,                                              1, },
    { MB_TYPE_16x16 | MB_TYPE_P0L0 | MB_TYPE_P0L1,                               1, },
    { MB_TYPE_16x8  | MB_TYPE_P0L0 | MB_TYPE_P1L0,                               2, },
    { MB_TYPE_8x16  | MB_TYPE_P0L0 | MB_TYPE_P1L0,                               2, },
    { MB_TYPE_16x8  | MB_TYPE_P0L1 | MB_TYPE_P1L1,                               2, },
    { MB_TYPE_8x16  | MB_TYPE_P0L1 | MB_TYPE_P1L1,                               2, },
    { MB_TYPE_16x8  | MB_TYPE_P0L0 | MB_TYPE_P0L1 | MB_TYPE_P1L0 | MB_TYPE_P1L1, 2, },
    { MB_TYPE_8x16  | MB_TYPE_P0L0 | MB_TYPE_P0L1 | MB_TYPE_P1L0 | MB_TYPE_P1L1, 2, },
    { MB_TYPE_8x8   | MB_TYPE_P0L0 | MB_TYPE_P1L0,                               4, },
    { MB_TYPE_8x8   | MB_TYPE_P0L1 | MB_TYPE_P1L1,                               4, },
    { MB_TYPE_8x8   | MB_TYPE_P0L0 | MB_TYPE_P0L1 | MB_TYPE_P1L0 | MB_TYPE_P1L1, 4, },
};

const uint8_t h264_dequant4_coeff_init[6][3] = {
    { 10, 13, 16 },
    { 11, 14, 18 },
    { 13, 16, 20 },
    { 14, 18, 23 },
    { 16, 20, 25 },
    { 18, 23, 29 },
};

const uint8_t h264_dequant8_coeff_init_scan[16] = {
    0, 3, 4, 3, 3, 1, 5, 1, 4, 5, 2, 5, 3, 1, 5, 1
};

const uint8_t h264_dequant8_coeff_init[6][6] = {
    { 20, 18, 32, 19, 25, 24 },
    { 22, 19, 35, 21, 28, 26 },
    { 26, 23, 42, 24, 33, 31 },
    { 28, 25, 45, 26, 35, 33 },
    { 32, 28, 51, 30, 40, 38 },
    { 36, 32, 58, 34, 46, 43 },
};

const uint8_t h264_quant_rem6[QP_MAX_NUM + 1] = {
    0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2,
    3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5,
    0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2,
    3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5,
    0, 1, 2, 3,
};

const uint8_t h264_quant_div6[QP_MAX_NUM + 1] = {
    0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3,  3,  3,
    3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6,  6,  6,
    7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 10, 10, 10,
    10, 10, 10, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13,
    14, 14, 14, 14,
};

#define QP(qP, depth) ((qP) + 6 * ((depth) - 8))
#define CHROMA_QP_TABLE_END(d)                                          \
    QP(0,  d), QP(1,  d), QP(2,  d), QP(3,  d), QP(4,  d), QP(5,  d),   \
    QP(6,  d), QP(7,  d), QP(8,  d), QP(9,  d), QP(10, d), QP(11, d),   \
    QP(12, d), QP(13, d), QP(14, d), QP(15, d), QP(16, d), QP(17, d),   \
    QP(18, d), QP(19, d), QP(20, d), QP(21, d), QP(22, d), QP(23, d),   \
    QP(24, d), QP(25, d), QP(26, d), QP(27, d), QP(28, d), QP(29, d),   \
    QP(29, d), QP(30, d), QP(31, d), QP(32, d), QP(32, d), QP(33, d),   \
    QP(34, d), QP(34, d), QP(35, d), QP(35, d), QP(36, d), QP(36, d),   \
    QP(37, d), QP(37, d), QP(37, d), QP(38, d), QP(38, d), QP(38, d),   \
    QP(39, d), QP(39, d), QP(39, d), QP(39, d)

const uint8_t h264_chroma_qp[7][QP_MAX_NUM + 1] = {
    { CHROMA_QP_TABLE_END(8) },
    { 0, 1, 2, 3, 4, 5,
      CHROMA_QP_TABLE_END(9) },
    { 0, 1, 2, 3,  4,  5,
      6, 7, 8, 9, 10, 11,
      CHROMA_QP_TABLE_END(10) },
    { 0,  1, 2, 3,  4,  5,
      6,  7, 8, 9, 10, 11,
      12,13,14,15, 16, 17,
      CHROMA_QP_TABLE_END(11) },
    { 0,  1, 2, 3,  4,  5,
      6,  7, 8, 9, 10, 11,
      12,13,14,15, 16, 17,
      18,19,20,21, 22, 23,
      CHROMA_QP_TABLE_END(12) },
    { 0,  1, 2, 3,  4,  5,
      6,  7, 8, 9, 10, 11,
      12,13,14,15, 16, 17,
      18,19,20,21, 22, 23,
      24,25,26,27, 28, 29,
      CHROMA_QP_TABLE_END(13) },
    { 0,  1, 2, 3,  4,  5,
      6,  7, 8, 9, 10, 11,
      12,13,14,15, 16, 17,
      18,19,20,21, 22, 23,
      24,25,26,27, 28, 29,
      30,31,32,33, 34, 35,
      CHROMA_QP_TABLE_END(14) },
};

const uint8_t h264_zigzag_scan[17] = {
    0, 1, 4, 8, 5, 2, 3, 6, 9, 12, 13, 10, 7, 11, 14, 15, 0
};

const uint8_t h264_zigzag_direct[64] = {
     0,  1,  8, 16,  9,  2,  3, 10,
    17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34,
    27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36,
    29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46,
    53, 60, 61, 54, 47, 55, 62, 63,
};

const uint8_t h264_field_scan[17] = {
    0 + 0 * 4, 0 + 1 * 4, 1 + 0 * 4, 0 + 2 * 4,
    0 + 3 * 4, 1 + 1 * 4, 1 + 2 * 4, 1 + 3 * 4,
    2 + 0 * 4, 2 + 1 * 4, 2 + 2 * 4, 2 + 3 * 4,
    3 + 0 * 4, 3 + 1 * 4, 3 + 2 * 4, 3 + 3 * 4,
    0
};

const uint8_t h264_field_scan8x8[65] = {
    0 + 0 * 8, 0 + 1 * 8, 0 + 2 * 8, 1 + 0 * 8,
    1 + 1 * 8, 0 + 3 * 8, 0 + 4 * 8, 1 + 2 * 8,
    2 + 0 * 8, 1 + 3 * 8, 0 + 5 * 8, 0 + 6 * 8,
    0 + 7 * 8, 1 + 4 * 8, 2 + 1 * 8, 3 + 0 * 8,
    2 + 2 * 8, 1 + 5 * 8, 1 + 6 * 8, 1 + 7 * 8,
    2 + 3 * 8, 3 + 1 * 8, 4 + 0 * 8, 3 + 2 * 8,
    2 + 4 * 8, 2 + 5 * 8, 2 + 6 * 8, 2 + 7 * 8,
    3 + 3 * 8, 4 + 1 * 8, 5 + 0 * 8, 4 + 2 * 8,
    3 + 4 * 8, 3 + 5 * 8, 3 + 6 * 8, 3 + 7 * 8,
    4 + 3 * 8, 5 + 1 * 8, 6 + 0 * 8, 5 + 2 * 8,
    4 + 4 * 8, 4 + 5 * 8, 4 + 6 * 8, 4 + 7 * 8,
    5 + 3 * 8, 6 + 1 * 8, 6 + 2 * 8, 5 + 4 * 8,
    5 + 5 * 8, 5 + 6 * 8, 5 + 7 * 8, 6 + 3 * 8,
    7 + 0 * 8, 7 + 1 * 8, 6 + 4 * 8, 6 + 5 * 8,
    6 + 6 * 8, 6 + 7 * 8, 7 + 2 * 8, 7 + 3 * 8,
    7 + 4 * 8, 7 + 5 * 8, 7 + 6 * 8, 7 + 7 * 8,
    0
};

const uint8_t h264_field_scan8x8_cavlc[65] = {
    0 + 0 * 8, 1 + 1 * 8, 2 + 0 * 8, 0 + 7 * 8,
    2 + 2 * 8, 2 + 3 * 8, 2 + 4 * 8, 3 + 3 * 8,
    3 + 4 * 8, 4 + 3 * 8, 4 + 4 * 8, 5 + 3 * 8,
    5 + 5 * 8, 7 + 0 * 8, 6 + 6 * 8, 7 + 4 * 8,
    0 + 1 * 8, 0 + 3 * 8, 1 + 3 * 8, 1 + 4 * 8,
    1 + 5 * 8, 3 + 1 * 8, 2 + 5 * 8, 4 + 1 * 8,
    3 + 5 * 8, 5 + 1 * 8, 4 + 5 * 8, 6 + 1 * 8,
    5 + 6 * 8, 7 + 1 * 8, 6 + 7 * 8, 7 + 5 * 8,
    0 + 2 * 8, 0 + 4 * 8, 0 + 5 * 8, 2 + 1 * 8,
    1 + 6 * 8, 4 + 0 * 8, 2 + 6 * 8, 5 + 0 * 8,
    3 + 6 * 8, 6 + 0 * 8, 4 + 6 * 8, 6 + 2 * 8,
    5 + 7 * 8, 6 + 4 * 8, 7 + 2 * 8, 7 + 6 * 8,
    1 + 0 * 8, 1 + 2 * 8, 0 + 6 * 8, 3 + 0 * 8,
    1 + 7 * 8, 3 + 2 * 8, 2 + 7 * 8, 4 + 2 * 8,
    3 + 7 * 8, 5 + 2 * 8, 4 + 7 * 8, 5 + 4 * 8,
    6 + 3 * 8, 6 + 5 * 8, 7 + 3 * 8, 7 + 7 * 8,
    0
};

const uint8_t h264_zigzag_scan8x8_cavlc[65] = {
    0 + 0 * 8, 1 + 1 * 8, 1 + 2 * 8, 2 + 2 * 8,
    4 + 1 * 8, 0 + 5 * 8, 3 + 3 * 8, 7 + 0 * 8,
    3 + 4 * 8, 1 + 7 * 8, 5 + 3 * 8, 6 + 3 * 8,
    2 + 7 * 8, 6 + 4 * 8, 5 + 6 * 8, 7 + 5 * 8,
    1 + 0 * 8, 2 + 0 * 8, 0 + 3 * 8, 3 + 1 * 8,
    3 + 2 * 8, 0 + 6 * 8, 4 + 2 * 8, 6 + 1 * 8,
    2 + 5 * 8, 2 + 6 * 8, 6 + 2 * 8, 5 + 4 * 8,
    3 + 7 * 8, 7 + 3 * 8, 4 + 7 * 8, 7 + 6 * 8,
    0 + 1 * 8, 3 + 0 * 8, 0 + 4 * 8, 4 + 0 * 8,
    2 + 3 * 8, 1 + 5 * 8, 5 + 1 * 8, 5 + 2 * 8,
    1 + 6 * 8, 3 + 5 * 8, 7 + 1 * 8, 4 + 5 * 8,
    4 + 6 * 8, 7 + 4 * 8, 5 + 7 * 8, 6 + 7 * 8,
    0 + 2 * 8, 2 + 1 * 8, 1 + 3 * 8, 5 + 0 * 8,
    1 + 4 * 8, 2 + 4 * 8, 6 + 0 * 8, 4 + 3 * 8,
    0 + 7 * 8, 4 + 4 * 8, 7 + 2 * 8, 3 + 6 * 8,
    5 + 5 * 8, 6 + 5 * 8, 6 + 6 * 8, 7 + 7 * 8,
    0
};

const uint8_t h264_scan8[16 * 3 + 3] = {
    4 +  1 * 8, 5 +  1 * 8, 4 +  2 * 8, 5 +  2 * 8,
    6 +  1 * 8, 7 +  1 * 8, 6 +  2 * 8, 7 +  2 * 8,
    4 +  3 * 8, 5 +  3 * 8, 4 +  4 * 8, 5 +  4 * 8,
    6 +  3 * 8, 7 +  3 * 8, 6 +  4 * 8, 7 +  4 * 8,
    4 +  6 * 8, 5 +  6 * 8, 4 +  7 * 8, 5 +  7 * 8,
    6 +  6 * 8, 7 +  6 * 8, 6 +  7 * 8, 7 +  7 * 8,
    4 +  8 * 8, 5 +  8 * 8, 4 +  9 * 8, 5 +  9 * 8,
    6 +  8 * 8, 7 +  8 * 8, 6 +  9 * 8, 7 +  9 * 8,
    4 + 11 * 8, 5 + 11 * 8, 4 + 12 * 8, 5 + 12 * 8,
    6 + 11 * 8, 7 + 11 * 8, 6 + 12 * 8, 7 + 12 * 8,
    4 + 13 * 8, 5 + 13 * 8, 4 + 14 * 8, 5 + 14 * 8,
    6 + 13 * 8, 7 + 13 * 8, 6 + 14 * 8, 7 + 14 * 8,
    0 +  0 * 8, 0 +  5 * 8, 0 + 10 * 8
};

static uint8_t transpose4(uint8_t x);
static uint8_t transpose8(uint8_t x);

static uint8_t transpose4(uint8_t x)
{
    return (uint8_t)((x >> 2) | ((x << 2) & 0xF));
}

static uint8_t transpose8(uint8_t x)
{
    return (uint8_t)((x >> 3) | ((x & 7) << 3));
}

H264Dbg h264_dbg_cfg;

void h264_dbg_init(void)
{
    static int loaded;
    int expected = 0;
    const char *s;

    /* one compare-exchange per open, and the environment block runs once per
     * process: the winner fills the struct with the flag at -1, and everyone
     * else waits for the release store that publishes the finished block */
    if (__atomic_load_n(&loaded, __ATOMIC_ACQUIRE))
        return;
    if (!__atomic_compare_exchange_n(&loaded, &expected, -1, 0, __ATOMIC_ACQ_REL,
                                     __ATOMIC_ACQUIRE)) {
        while (__atomic_load_n(&loaded, __ATOMIC_ACQUIRE) != 1)
            ;
        return;
    }

    h264_dbg_cfg.mv   = getenv("H264_TRACE_MV") != NULL;
    h264_dbg_cfg.mc   = getenv("H264_TRACE_MC") != NULL;
    h264_dbg_cfg.pred = getenv("H264_TRACE_PRED") != NULL;
    h264_dbg_cfg.dm   = getenv("H264_TRACE_DM") != NULL;
    h264_dbg_cfg.mb   = getenv("H264_DEBUG_MB") != NULL;
    h264_dbg_cfg.lf   = getenv("H264_TRACE_LF") != NULL;
    h264_dbg_cfg.bits = getenv("H264_DEBUG_BITS") != NULL;
    h264_dbg_cfg.coef = getenv("H264_TRACE_COEF") != NULL;
    h264_dbg_cfg.ps   = getenv("H264_TRACE_PS") != NULL;
    s = getenv("H264_TRACE_MBX");
    h264_dbg_cfg.mbx = s ? atoi(s) : 0;
    s = getenv("H264_TRACE_MBY");
    h264_dbg_cfg.mby = s ? atoi(s) : 0;
    s = getenv("H264_TRACE_FN");
    h264_dbg_cfg.fn = s ? atoi(s) : -1;
    s = getenv("H264_DEBUG_LAG");
    h264_dbg_cfg.lag = s ? atoi(s) : 0;
    __atomic_store_n(&loaded, 1, __ATOMIC_RELEASE);
}

void h264_init_scan_tables(H264Decoder *h)
{
    int i;

    for (i = 0; i < 16; i++) {
        h->zigzag_scan[i] = transpose4(h264_zigzag_scan[i]);
        h->field_scan[i]  = transpose4(h264_field_scan[i]);
    }
    for (i = 0; i < 64; i++) {
        h->zigzag_scan8x8[i] = transpose8(h264_zigzag_direct[i]);
        h->field_scan8x8[i]  = transpose8(h264_field_scan8x8[i]);
        h->zigzag_scan8x8_cavlc[i] = transpose8(h264_zigzag_scan8x8_cavlc[i]);
        h->field_scan8x8_cavlc[i]  = transpose8(h264_field_scan8x8_cavlc[i]);
    }
    if (h->active_sps && h->active_sps->transform_bypass) {
        memcpy(h->zigzag_scan_q0,    h264_zigzag_scan,               16);
        memcpy(h->field_scan_q0,     h264_field_scan,                16);
        memcpy(h->zigzag_scan8x8_q0, h264_zigzag_direct,             64);
        memcpy(h->field_scan8x8_q0,  h264_field_scan8x8,             64);
        memcpy(h->zigzag_scan8x8_cavlc_q0, h264_zigzag_scan8x8_cavlc, 64);
        memcpy(h->field_scan8x8_cavlc_q0,  h264_field_scan8x8_cavlc,  64);
    } else {
        memcpy(h->zigzag_scan_q0,    h->zigzag_scan,      16);
        memcpy(h->field_scan_q0,     h->field_scan,       16);
        memcpy(h->zigzag_scan8x8_q0, h->zigzag_scan8x8,   64);
        memcpy(h->field_scan8x8_q0,  h->field_scan8x8,    64);
        memcpy(h->zigzag_scan8x8_cavlc_q0, h->zigzag_scan8x8_cavlc, 64);
        memcpy(h->field_scan8x8_cavlc_q0,  h->field_scan8x8_cavlc,  64);
    }
}

const uint8_t h264_chroma422_dc_scan[8] = {
    0, 32, 16, 64, 96, 48, 80, 112
};
