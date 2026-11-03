/*
 * B-slice direct motion prediction (8-bit 4:2:0).
 * Algorithms from ITU-T H.264, ported from FFmpeg's h264_direct.c (LGPL-2.1+).
 */
#include "h264_internal.h"

static int clip_int8(int v)
{
    return v < -128 ? -128 : v > 127 ? 127 : v;
}

static int get_scale_factor(H264Slice *sl, int poc, int poc1, int i)
{
    int poc0 = sl->ref_list[0][i].poc;
    int64_t pocdiff = poc1 - (int64_t)poc0;
    int td = clip_int8((int)pocdiff);

    if (td == 0 || sl->ref_list[0][i].parent->long_ref)
        return 256;

    {
        int64_t pocdiff0 = poc - (int64_t)poc0;
        int tb = clip_int8((int)pocdiff0);
        int tx = (16384 + (H264_ABS(td) >> 1)) / td;
        int v = (int)((tb * (int64_t)tx + 32) >> 6);
        return H264_CLIP(v, -1024, 1023);
    }
}

void h264_direct_dist_scale_factor(H264Decoder *h, H264Slice *sl)
{
    H264Frame *f = h->cur_pic;
    const int poc = f->field_picture ?
        f->field_poc[h->picture_structure == PICT_BOTTOM_FIELD] : f->poc;
    const int poc1 = sl->ref_list[1][0].poc;
    int i;

    if (f->mbaff) {
        int field;
        for (field = 0; field < 2; field++) {
            const int fpoc = f->field_poc[field];
            const int fpoc1 = sl->ref_list[1][0].parent->field_poc[field];
            for (i = 0; i < 2 * sl->ref_count[0]; i++)
                sl->dist_scale_factor_field[field][i ^ field] =
                    get_scale_factor(sl, fpoc, fpoc1, i + 16);
        }
    }
    for (i = 0; i < sl->ref_count[0]; i++)
        sl->dist_scale_factor[i] = get_scale_factor(sl, poc, poc1, i);
}

/* map a colocated reference index onto our list0 */
static void fill_colmap(H264Decoder *h, H264Slice *sl, int map[2][16 + 32],
                        int list, int field, int colfield, int mbafi)
{
    const H264Frame *ref1 = sl->ref_list[1][0].parent;
    int j, old_ref, rfield;
    const int start = mbafi ? 16 : 0;
    const int end = mbafi ? 16 + 2 * sl->ref_count[0] : sl->ref_count[0];
    const int interl = mbafi || h->picture_structure != PICT_FRAME;

    memset(map[list], 0, sizeof(map[list]));

    for (rfield = 0; rfield < 2; rfield++) {
        for (old_ref = 0; old_ref < ref1->ref_list_count[colfield][list]; old_ref++) {
            int poc = ref1->ref_poc[colfield][list][old_ref];

            if (!interl)
                poc |= 3;
            else if ((poc & 3) == 3)
                poc = (poc & ~3) + rfield + 1;

            for (j = start; j < end; j++) {
                if (4 * sl->ref_list[0][j].parent->frame_num +
                    (sl->ref_list[0][j].reference & 3) == poc) {
                    int cur_ref = mbafi ? ((j - 16) ^ field) : j;
                    if (ref1->mbaff)
                        map[list][2 * old_ref + (rfield ^ field) + 16] = cur_ref;
                    if (rfield == field || !interl)
                        map[list][old_ref] = cur_ref;
                    break;
                }
            }
        }
    }
}

void h264_direct_ref_list_init(H264Decoder *h, H264Slice *sl)
{
    H264Ref *ref1 = &sl->ref_list[1][0];
    H264Frame *cur = h->cur_pic;
    int list, field, j;
    int sidx = (h->picture_structure & 1) ^ 1;
    int ref1sidx = (ref1->reference & 1) ^ 1;

    for (list = 0; list < sl->list_count; list++) {
        cur->ref_list_count[sidx][list] = sl->ref_count[list];
        for (j = 0; j < sl->ref_count[list]; j++)
            cur->ref_poc[sidx][list][j] =
                4 * sl->ref_list[list][j].parent->frame_num +
                (sl->ref_list[list][j].reference & 3);
    }

    if (h->picture_structure == PICT_FRAME) {
        memcpy(cur->ref_list_count[1], cur->ref_list_count[0],
               sizeof(cur->ref_list_count[0]));
        memcpy(cur->ref_poc[1], cur->ref_poc[0], sizeof(cur->ref_poc[0]));
    }

    sl->col_fieldoff = 0;

    if (sl->list_count != 2 || !sl->ref_count[1])
        return;

    if (h->picture_structure == PICT_FRAME) {
        const int cur_poc = cur->poc;
        const int *col_poc = sl->ref_list[1][0].parent->field_poc;
        if (col_poc[0] == INT32_MAX && col_poc[1] == INT32_MAX) {
            fprintf(stderr, "[h264dec] co located POCs unavailable\n");
            sl->col_parity = 1;
        } else {
            sl->col_parity =
                H264_ABS(col_poc[0] - (int64_t)cur_poc) >=
                H264_ABS(col_poc[1] - (int64_t)cur_poc);
        }
        ref1sidx = sidx = sl->col_parity;
    } else if (!(h->picture_structure & sl->ref_list[1][0].reference) &&
               !sl->ref_list[1][0].parent->mbaff) {
        sl->col_fieldoff = 2 * sl->ref_list[1][0].reference - 3;
    }

    if (sl->slice_type_nos != H264_SLICE_B || sl->direct_spatial_mv_pred)
        return;

    for (list = 0; list < 2; list++) {
        fill_colmap(h, sl, sl->map_col_to_list0, list, sidx, ref1sidx, 0);
        if (cur->mbaff) {
            for (field = 0; field < 2; field++)
                fill_colmap(h, sl, sl->map_col_to_list0_field[field], list,
                            field, field, 1);
        }
    }
}

#define MB_TYPE_16x16_OR_INTRA (MB_TYPE_16x16 | MB_TYPE_INTRA4x4 | \
                                MB_TYPE_INTRA16x16 | MB_TYPE_INTRA_PCM)

/* The macroblock data of the collocated picture is produced by its own
 * worker, so the row that this MB maps to has to be finished first. */
static void await_colocated(H264Decoder *h, H264Slice *sl, const H264Ref *ref)
{
    const H264Frame *pic = ref->parent;
    int ref_field, field_picture, ref_height, row;

    if (!h->threaded || !pic || pic == h->cur_pic)
        return;
    ref_field = ref->reference - 1;
    field_picture = pic->field_picture;
    ref_height = 16 * h->mb_height >> field_picture;
    row = H264_MIN(16 * sl->mb_y >> field_picture, ref_height - 1);
    h264_progress_await(h, pic, row, field_picture && ref_field);
    /* a frame waits on its field pair as a whole, while the colocated data may
     * come from either field, decoded by different workers */
    if (field_picture && h->picture_structure == PICT_FRAME)
        h264_progress_await(h, pic, row, 0);
}

static void pred_spatial_direct_motion(H264Decoder *h, H264Slice *sl,
                                       uint32_t *mb_type)
{
    H264Frame *f = h->cur_pic;
    const H264Frame *col = sl->ref_list[1][0].parent;
    await_colocated(h, sl, &sl->ref_list[1][0]);
    int b8_stride = 2;
    int b4_stride = f->b_stride;
    int mb_xy = sl->mb_xy, mb_y = sl->mb_y;
    int mb_type_col[2];
    const int16_t (*l1mv0)[2], (*l1mv1)[2];
    const int8_t *l1ref0, *l1ref1;
    const int is_b8x8 = IS_8X8(*mb_type) != 0;
    unsigned int sub_mb_type = MB_TYPE_L0L1;
    int i8, i4, list;
    int ref[2], mv[2];

    /* ref = min(neighbours) */
    for (list = 0; list < 2; list++) {
        int left_ref = sl->ref_cache[list][h264_scan8[0] - 1];
        int top_ref = sl->ref_cache[list][h264_scan8[0] - 8];
        int refc = sl->ref_cache[list][h264_scan8[0] - 8 + 4];
        const int16_t *C = sl->mv_cache[list][h264_scan8[0] - 8 + 4];
        if (refc == PART_NOT_AVAILABLE) {
            refc = sl->ref_cache[list][h264_scan8[0] - 8 - 1];
            C = sl->mv_cache[list][h264_scan8[0] - 8 - 1];
        }
        ref[list] = (int)H264_MIN3((unsigned)left_ref, (unsigned)top_ref,
                                   (unsigned)refc);
        if (h264_dbg()->dm)
            fprintf(stderr, "SDT fn %d %d %d L%d l %d t %d c %d r %d\n",
                    h->cur_pic->frame_num, sl->mb_x, sl->mb_y, list,
                    left_ref, top_ref, refc, ref[list]);
        if (ref[list] >= 0) {
            const int16_t *A = sl->mv_cache[list][h264_scan8[0] - 1];
            const int16_t *B = sl->mv_cache[list][h264_scan8[0] - 8];
            int match_count = (left_ref == ref[list]) +
                              (top_ref == ref[list]) +
                              (refc == ref[list]);
            if (match_count > 1) {
                mv[list] = H264_PACK16TO32(h264_mid_pred(A[0], B[0], C[0]),
                                           h264_mid_pred(A[1], B[1], C[1]));
            } else if (left_ref == ref[list]) {
                mv[list] = (int)H264_MV32(A);
            } else if (top_ref == ref[list]) {
                mv[list] = (int)H264_MV32(B);
            } else {
                mv[list] = (int)H264_MV32(C);
            }
        } else {
            unsigned int mask = ~(MB_TYPE_L0 << (2 * list));
            mv[list] = 0;
            ref[list] = -1;
            if (!is_b8x8)
                *mb_type &= mask;
            sub_mb_type &= mask;
        }
    }
    if (ref[0] < 0 && ref[1] < 0) {
        ref[0] = ref[1] = 0;
        if (!is_b8x8)
            *mb_type |= MB_TYPE_L0L1;
        sub_mb_type |= MB_TYPE_L0L1;
    }

    if (!(is_b8x8 | mv[0] | mv[1])) {
        H264_FILL_RECT(&sl->ref_cache[0][h264_scan8[0]], 4, 4, 8, ref[0], int8_t);
        H264_FILL_RECT(&sl->ref_cache[1][h264_scan8[0]], 4, 4, 8, ref[1], int8_t);
        H264_FILL_RECT(&sl->mv_cache[0][h264_scan8[0]], 4, 4, 8, 0, int32_t);
        H264_FILL_RECT(&sl->mv_cache[1][h264_scan8[0]], 4, 4, 8, 0, int32_t);
        *mb_type = (*mb_type & ~(MB_TYPE_8x8 | MB_TYPE_16x8 | MB_TYPE_8x16 |
                                 MB_TYPE_P1L0 | MB_TYPE_P1L1)) |
                   MB_TYPE_16x16 | MB_TYPE_DIRECT2;
        return;
    }

    if (col->field_picture || IS_INTERLACED(col->mb_type[mb_xy])) {
        if (!IS_INTERLACED(*mb_type)) {
            mb_y = (sl->mb_y & ~1) + sl->col_parity;
            mb_xy = sl->mb_x + ((sl->mb_y & ~1) + sl->col_parity) * h->mb_stride;
            b8_stride = 0;
        } else {
            mb_y += sl->col_fieldoff;
            mb_xy += h->mb_stride * sl->col_fieldoff;
        }
        goto single_col;
    } else {
        if (IS_INTERLACED(*mb_type)) {
            mb_y = sl->mb_y & ~1;
            mb_xy = (sl->mb_y & ~1) * h->mb_stride + sl->mb_x;
            mb_type_col[0] = col->mb_type[mb_xy];
            mb_type_col[1] = col->mb_type[mb_xy + h->mb_stride];
            b8_stride = 2 + 4 * h->mb_stride;
            b4_stride *= 6;
            if (IS_INTERLACED(mb_type_col[0]) != IS_INTERLACED(mb_type_col[1])) {
                mb_type_col[0] &= ~MB_TYPE_INTERLACED;
                mb_type_col[1] &= ~MB_TYPE_INTERLACED;
            }
            sub_mb_type |= MB_TYPE_16x16 | MB_TYPE_DIRECT2; /* B_SUB_8x8 */
            if ((mb_type_col[0] & MB_TYPE_16x16_OR_INTRA) &&
                (mb_type_col[1] & MB_TYPE_16x16_OR_INTRA) && !is_b8x8)
                *mb_type |= MB_TYPE_16x8 | MB_TYPE_DIRECT2; /* B_16x8 */
            else
                *mb_type |= MB_TYPE_8x8;
        } else {
single_col:
            mb_type_col[0] = mb_type_col[1] = col->mb_type[mb_xy];
            sub_mb_type |= MB_TYPE_16x16 | MB_TYPE_DIRECT2; /* B_SUB_8x8 */
            if (!is_b8x8 && (mb_type_col[0] & MB_TYPE_16x16_OR_INTRA))
                *mb_type |= MB_TYPE_16x16 | MB_TYPE_DIRECT2; /* B_16x16 */
            else if (!is_b8x8 &&
                     (mb_type_col[0] & (MB_TYPE_16x8 | MB_TYPE_8x16)))
                *mb_type |= MB_TYPE_DIRECT2 |
                            (mb_type_col[0] & (MB_TYPE_16x8 | MB_TYPE_8x16));
            else {
                if (!h->active_sps->direct_8x8_inference_flag)
                    sub_mb_type += MB_TYPE_8x8 - MB_TYPE_16x16; /* B_SUB_4x4 */
                *mb_type |= MB_TYPE_8x8;
            }
        }
    }

    (void)mb_y;
    l1mv0 = col->motion_val[0] + h->mb2b_xy[mb_xy];
    l1mv1 = col->motion_val[1] + h->mb2b_xy[mb_xy];
    l1ref0 = &col->ref_index[0][4 * mb_xy];
    l1ref1 = &col->ref_index[1][4 * mb_xy];
    if (!b8_stride) {
        if (sl->mb_y & 1) {
            l1ref0 += 2;
            l1ref1 += 2;
            l1mv0 += 2 * b4_stride;
            l1mv1 += 2 * b4_stride;
        }
    }

    if (IS_INTERLACED(*mb_type) != IS_INTERLACED(mb_type_col[0])) {
        int n = 0;
        for (i8 = 0; i8 < 4; i8++) {
            int x8 = i8 & 1;
            int y8 = i8 >> 1;
            int xy8 = x8 + y8 * b8_stride;
            int xy4 = x8 * 3 + y8 * b4_stride;
            int a, b;

            if (is_b8x8 && !IS_DIRECT(sl->sub_mb_type[i8]))
                continue;
            sl->sub_mb_type[i8] = (uint16_t)sub_mb_type;

            H264_FILL_RECT(&sl->ref_cache[0][h264_scan8[i8 * 4]], 2, 2, 8, ref[0], int8_t);
            H264_FILL_RECT(&sl->ref_cache[1][h264_scan8[i8 * 4]], 2, 2, 8, ref[1], int8_t);
            if (!IS_INTRA(mb_type_col[y8]) && !col->long_ref &&
                ((l1ref0[xy8] == 0 &&
                  H264_ABS(l1mv0[xy4][0]) <= 1 &&
                  H264_ABS(l1mv0[xy4][1]) <= 1) ||
                 (l1ref0[xy8] < 0 &&
                  l1ref1[xy8] == 0 &&
                  H264_ABS(l1mv1[xy4][0]) <= 1 &&
                  H264_ABS(l1mv1[xy4][1]) <= 1))) {
                a = b = 0;
                if (ref[0] > 0)
                    a = mv[0];
                if (ref[1] > 0)
                    b = mv[1];
                n++;
            } else {
                a = mv[0];
                b = mv[1];
            }
            H264_FILL_RECT(&sl->mv_cache[0][h264_scan8[i8 * 4]], 2, 2, 8, a, int32_t);
            H264_FILL_RECT(&sl->mv_cache[1][h264_scan8[i8 * 4]], 2, 2, 8, b, int32_t);
        }
        if (!is_b8x8 && !(n & 3))
            *mb_type = (*mb_type & ~(MB_TYPE_8x8 | MB_TYPE_16x8 | MB_TYPE_8x16 |
                                     MB_TYPE_P1L0 | MB_TYPE_P1L1)) |
                       MB_TYPE_16x16 | MB_TYPE_DIRECT2;
    } else if (IS_16X16(*mb_type)) {
        int a, b;

        H264_FILL_RECT(&sl->ref_cache[0][h264_scan8[0]], 4, 4, 8, ref[0], int8_t);
        H264_FILL_RECT(&sl->ref_cache[1][h264_scan8[0]], 4, 4, 8, ref[1], int8_t);
        if (!IS_INTRA(mb_type_col[0]) && !col->long_ref &&
            ((l1ref0[0] == 0 &&
              H264_ABS(l1mv0[0][0]) <= 1 &&
              H264_ABS(l1mv0[0][1]) <= 1) ||
             (l1ref0[0] < 0 && !l1ref1[0] &&
              H264_ABS(l1mv1[0][0]) <= 1 &&
              H264_ABS(l1mv1[0][1]) <= 1 &&
              (unsigned)h->x264_build > 33U))) {
            a = b = 0;
            if (ref[0] > 0)
                a = mv[0];
            if (ref[1] > 0)
                b = mv[1];
        } else {
            a = mv[0];
            b = mv[1];
        }
        H264_FILL_RECT(&sl->mv_cache[0][h264_scan8[0]], 4, 4, 8, a, int32_t);
        H264_FILL_RECT(&sl->mv_cache[1][h264_scan8[0]], 4, 4, 8, b, int32_t);
    } else {
        int n = 0;
        for (i8 = 0; i8 < 4; i8++) {
            const int x8 = i8 & 1;
            const int y8 = i8 >> 1;

            if (is_b8x8 && !IS_DIRECT(sl->sub_mb_type[i8]))
                continue;
            sl->sub_mb_type[i8] = (uint16_t)sub_mb_type;

            H264_FILL_RECT(&sl->mv_cache[0][h264_scan8[i8 * 4]], 2, 2, 8, mv[0], int32_t);
            H264_FILL_RECT(&sl->mv_cache[1][h264_scan8[i8 * 4]], 2, 2, 8, mv[1], int32_t);
            H264_FILL_RECT(&sl->ref_cache[0][h264_scan8[i8 * 4]], 2, 2, 8, ref[0], int8_t);
            H264_FILL_RECT(&sl->ref_cache[1][h264_scan8[i8 * 4]], 2, 2, 8, ref[1], int8_t);

            /* col_zero_flag */
            if (!IS_INTRA(mb_type_col[0]) && !col->long_ref &&
                (l1ref0[i8] == 0 ||
                 (l1ref0[i8] < 0 && l1ref1[i8] == 0 && (unsigned)h->x264_build > 33U))) {
                const int16_t(*l1mv)[2] = l1ref0[i8] == 0 ? l1mv0 : l1mv1;
                if (IS_SUB_8X8(sub_mb_type)) {
                    const int16_t *mv_col = l1mv[x8 * 3 + y8 * 3 * b4_stride];
                    if (H264_ABS(mv_col[0]) <= 1 && H264_ABS(mv_col[1]) <= 1) {
                        if (ref[0] == 0)
                            H264_FILL_RECT(&sl->mv_cache[0][h264_scan8[i8 * 4]], 2, 2, 8, 0, int32_t);
                        if (ref[1] == 0)
                            H264_FILL_RECT(&sl->mv_cache[1][h264_scan8[i8 * 4]], 2, 2, 8, 0, int32_t);
                        n += 4;
                    }
                } else {
                    int m = 0;
                    for (i4 = 0; i4 < 4; i4++) {
                        const int16_t *mv_col = l1mv[x8 * 2 + (i4 & 1) +
                                                     (y8 * 2 + (i4 >> 1)) * b4_stride];
                        if (H264_ABS(mv_col[0]) <= 1 && H264_ABS(mv_col[1]) <= 1) {
                            if (ref[0] == 0)
                                sl->mv_cache[0][h264_scan8[i8 * 4 + i4]][0] =
                                    sl->mv_cache[0][h264_scan8[i8 * 4 + i4]][1] = 0;
                            if (ref[1] == 0)
                                sl->mv_cache[1][h264_scan8[i8 * 4 + i4]][0] =
                                    sl->mv_cache[1][h264_scan8[i8 * 4 + i4]][1] = 0;
                            m++;
                        }
                    }
                    if (!(m & 3))
                        sl->sub_mb_type[i8] += (uint16_t)(MB_TYPE_16x16 - MB_TYPE_8x8);
                    n += m;
                }
            }
        }
        if (!is_b8x8 && !(n & 15))
            *mb_type = (*mb_type & ~(MB_TYPE_8x8 | MB_TYPE_16x8 | MB_TYPE_8x16 |
                                     MB_TYPE_P1L0 | MB_TYPE_P1L1)) |
                       MB_TYPE_16x16 | MB_TYPE_DIRECT2;
    }
}

static void pred_temp_direct_motion(H264Decoder *h, H264Slice *sl,
                                    uint32_t *mb_type)
{
    H264Frame *f = h->cur_pic;
    const H264Frame *col = sl->ref_list[1][0].parent;
    await_colocated(h, sl, &sl->ref_list[1][0]);
    int b8_stride = 2;
    int b4_stride = f->b_stride;
    int mb_xy = sl->mb_xy, mb_y = sl->mb_y;
    int mb_type_col[2];
    const int16_t (*l1mv0)[2], (*l1mv1)[2];
    const int8_t *l1ref0, *l1ref1;
    const int is_b8x8 = IS_8X8(*mb_type) != 0;
    unsigned int sub_mb_type;
    int i8, i4;

    if (col->field_picture || IS_INTERLACED(col->mb_type[mb_xy])) {
        if (!IS_INTERLACED(*mb_type)) {
            mb_y = (sl->mb_y & ~1) + sl->col_parity;
            mb_xy = sl->mb_x + ((sl->mb_y & ~1) + sl->col_parity) * h->mb_stride;
            b8_stride = 0;
        } else {
            mb_y += sl->col_fieldoff;
            mb_xy += h->mb_stride * sl->col_fieldoff;
        }
        goto single_col;
    } else {
        if (IS_INTERLACED(*mb_type)) {
            mb_y = sl->mb_y & ~1;
            mb_xy = sl->mb_x + (sl->mb_y & ~1) * h->mb_stride;
            mb_type_col[0] = col->mb_type[mb_xy];
            mb_type_col[1] = col->mb_type[mb_xy + h->mb_stride];
            b8_stride = 2 + 4 * h->mb_stride;
            b4_stride *= 6;
            if (IS_INTERLACED(mb_type_col[0]) != IS_INTERLACED(mb_type_col[1])) {
                mb_type_col[0] &= ~MB_TYPE_INTERLACED;
                mb_type_col[1] &= ~MB_TYPE_INTERLACED;
            }
            sub_mb_type = MB_TYPE_16x16 | MB_TYPE_P0L0 | MB_TYPE_P0L1 |
                          MB_TYPE_DIRECT2; /* B_SUB_8x8 */
            if ((mb_type_col[0] & MB_TYPE_16x16_OR_INTRA) &&
                (mb_type_col[1] & MB_TYPE_16x16_OR_INTRA) && !is_b8x8)
                *mb_type |= MB_TYPE_16x8 | MB_TYPE_L0L1 | MB_TYPE_DIRECT2; /* B_16x8 */
            else
                *mb_type |= MB_TYPE_8x8 | MB_TYPE_L0L1;
        } else {
single_col:
            mb_type_col[0] = mb_type_col[1] = col->mb_type[mb_xy];
            sub_mb_type = MB_TYPE_16x16 | MB_TYPE_P0L0 | MB_TYPE_P0L1 |
                          MB_TYPE_DIRECT2; /* B_SUB_8x8 */
            if (!is_b8x8 && (mb_type_col[0] & MB_TYPE_16x16_OR_INTRA))
                *mb_type |= MB_TYPE_16x16 | MB_TYPE_P0L0 | MB_TYPE_P0L1 |
                            MB_TYPE_DIRECT2; /* B_16x16 */
            else if (!is_b8x8 &&
                     (mb_type_col[0] & (MB_TYPE_16x8 | MB_TYPE_8x16)))
                *mb_type |= MB_TYPE_L0L1 | MB_TYPE_DIRECT2 |
                            (mb_type_col[0] & (MB_TYPE_16x8 | MB_TYPE_8x16));
            else {
                if (!h->active_sps->direct_8x8_inference_flag)
                    sub_mb_type = MB_TYPE_8x8 | MB_TYPE_P0L0 | MB_TYPE_P0L1 |
                                  MB_TYPE_DIRECT2; /* B_SUB_4x4 */
                *mb_type |= MB_TYPE_8x8 | MB_TYPE_L0L1;
            }
        }
    }

    (void)mb_y;
    l1mv0 = col->motion_val[0] + h->mb2b_xy[mb_xy];
    l1mv1 = col->motion_val[1] + h->mb2b_xy[mb_xy];
    l1ref0 = &col->ref_index[0][4 * mb_xy];
    l1ref1 = &col->ref_index[1][4 * mb_xy];
    if (!b8_stride) {
        if (sl->mb_y & 1) {
            l1ref0 += 2;
            l1ref1 += 2;
            l1mv0 += 2 * b4_stride;
            l1mv1 += 2 * b4_stride;
        }
    }

    {
        const int *map_col_to_list0[2] = { sl->map_col_to_list0[0],
                                           sl->map_col_to_list0[1] };
        const int *dist_scale_factor = sl->dist_scale_factor;
        int ref_offset;

        if (f->mbaff && IS_INTERLACED(*mb_type)) {
            map_col_to_list0[0] = sl->map_col_to_list0_field[sl->mb_y & 1][0];
            map_col_to_list0[1] = sl->map_col_to_list0_field[sl->mb_y & 1][1];
            dist_scale_factor = sl->dist_scale_factor_field[sl->mb_y & 1];
        }
        ref_offset = (col->mbaff << 4) & (int)(mb_type_col[0] >> 3);

        if (IS_INTERLACED(*mb_type) != IS_INTERLACED(mb_type_col[0])) {
            int y_shift = 2 * !IS_INTERLACED(*mb_type);
            for (i8 = 0; i8 < 4; i8++) {
                const int x8 = i8 & 1;
                const int y8 = i8 >> 1;
                int ref0, scale;
                const int16_t(*l1mv)[2] = l1mv0;

                if (is_b8x8 && !IS_DIRECT(sl->sub_mb_type[i8]))
                    continue;
                sl->sub_mb_type[i8] = (uint16_t)sub_mb_type;

                H264_FILL_RECT(&sl->ref_cache[1][h264_scan8[i8 * 4]], 2, 2, 8, 0, int8_t);
                if (IS_INTRA(mb_type_col[y8])) {
                    H264_FILL_RECT(&sl->ref_cache[0][h264_scan8[i8 * 4]], 2, 2, 8, 0, int8_t);
                    H264_FILL_RECT(&sl->mv_cache[0][h264_scan8[i8 * 4]], 2, 2, 8, 0, int32_t);
                    H264_FILL_RECT(&sl->mv_cache[1][h264_scan8[i8 * 4]], 2, 2, 8, 0, int32_t);
                    continue;
                }

                ref0 = l1ref0[x8 + y8 * b8_stride];
                if (ref0 >= 0)
                    ref0 = map_col_to_list0[0][ref0 + ref_offset];
                else {
                    ref0 = map_col_to_list0[1][l1ref1[x8 + y8 * b8_stride] + ref_offset];
                    l1mv = l1mv1;
                }
                scale = dist_scale_factor[ref0];
                H264_FILL_RECT(&sl->ref_cache[0][h264_scan8[i8 * 4]], 2, 2, 8, ref0, int8_t);

                {
                    const int16_t *mv_col = l1mv[x8 * 3 + y8 * b4_stride];
                    int my_col = (mv_col[1] * (1 << y_shift)) / 2;
                    int mx = (scale * mv_col[0] + 128) >> 8;
                    int my = (scale * my_col + 128) >> 8;
                    H264_FILL_RECT(&sl->mv_cache[0][h264_scan8[i8 * 4]], 2, 2, 8,
                                   H264_PACK16TO32(mx, my), int32_t);
                    H264_FILL_RECT(&sl->mv_cache[1][h264_scan8[i8 * 4]], 2, 2, 8,
                                   H264_PACK16TO32(mx - mv_col[0], my - my_col), int32_t);
                }
            }
            return;
        }

        /* one-to-one mv scaling */
        if (IS_16X16(*mb_type)) {
            int ref, mv0, mv1;

            H264_FILL_RECT(&sl->ref_cache[1][h264_scan8[0]], 4, 4, 8, 0, int8_t);
            if (IS_INTRA(mb_type_col[0])) {
                ref = mv0 = mv1 = 0;
            } else {
                const int ref0 = l1ref0[0] >= 0
                    ? map_col_to_list0[0][l1ref0[0] + ref_offset]
                    : map_col_to_list0[1][l1ref1[0] + ref_offset];
                const int scale = dist_scale_factor[ref0];
                const int16_t *mv_col = l1ref0[0] >= 0 ? l1mv0[0] : l1mv1[0];
                int mv_l0[2];
                mv_l0[0] = (scale * mv_col[0] + 128) >> 8;
                mv_l0[1] = (scale * mv_col[1] + 128) >> 8;
                ref = ref0;
                mv0 = H264_PACK16TO32(mv_l0[0], mv_l0[1]);
                mv1 = H264_PACK16TO32(mv_l0[0] - mv_col[0], mv_l0[1] - mv_col[1]);
            }
            H264_FILL_RECT(&sl->ref_cache[0][h264_scan8[0]], 4, 4, 8, ref, int8_t);
            H264_FILL_RECT(&sl->mv_cache[0][h264_scan8[0]], 4, 4, 8, mv0, int32_t);
            H264_FILL_RECT(&sl->mv_cache[1][h264_scan8[0]], 4, 4, 8, mv1, int32_t);
        } else {
            for (i8 = 0; i8 < 4; i8++) {
                const int x8 = i8 & 1;
                const int y8 = i8 >> 1;
                int ref0, scale;
                const int16_t(*l1mv)[2] = l1mv0;

                if (is_b8x8 && !IS_DIRECT(sl->sub_mb_type[i8]))
                    continue;
                sl->sub_mb_type[i8] = (uint16_t)sub_mb_type;
                H264_FILL_RECT(&sl->ref_cache[1][h264_scan8[i8 * 4]], 2, 2, 8, 0, int8_t);
                if (IS_INTRA(mb_type_col[0])) {
                    H264_FILL_RECT(&sl->ref_cache[0][h264_scan8[i8 * 4]], 2, 2, 8, 0, int8_t);
                    H264_FILL_RECT(&sl->mv_cache[0][h264_scan8[i8 * 4]], 2, 2, 8, 0, int32_t);
                    H264_FILL_RECT(&sl->mv_cache[1][h264_scan8[i8 * 4]], 2, 2, 8, 0, int32_t);
                    continue;
                }

                ref0 = l1ref0[i8];
                if (ref0 >= 0)
                    ref0 = map_col_to_list0[0][ref0 + ref_offset];
                else {
                    ref0 = map_col_to_list0[1][l1ref1[i8] + ref_offset];
                    l1mv = l1mv1;
                }
                scale = dist_scale_factor[ref0];

                H264_FILL_RECT(&sl->ref_cache[0][h264_scan8[i8 * 4]], 2, 2, 8, ref0, int8_t);
                if (IS_SUB_8X8(sub_mb_type)) {
                    const int16_t *mv_col = l1mv[x8 * 3 + y8 * 3 * b4_stride];
                    int mx = (scale * mv_col[0] + 128) >> 8;
                    int my = (scale * mv_col[1] + 128) >> 8;
                    H264_FILL_RECT(&sl->mv_cache[0][h264_scan8[i8 * 4]], 2, 2, 8,
                                   H264_PACK16TO32(mx, my), int32_t);
                    H264_FILL_RECT(&sl->mv_cache[1][h264_scan8[i8 * 4]], 2, 2, 8,
                                   H264_PACK16TO32(mx - mv_col[0], my - mv_col[1]), int32_t);
                } else {
                    for (i4 = 0; i4 < 4; i4++) {
                        const int16_t *mv_col = l1mv[x8 * 2 + (i4 & 1) +
                                                     (y8 * 2 + (i4 >> 1)) * b4_stride];
                        int16_t *mv_l0 = sl->mv_cache[0][h264_scan8[i8 * 4 + i4]];
                        mv_l0[0] = (int16_t)((scale * mv_col[0] + 128) >> 8);
                        mv_l0[1] = (int16_t)((scale * mv_col[1] + 128) >> 8);
                        sl->mv_cache[1][h264_scan8[i8 * 4 + i4]][0] =
                            (int16_t)(mv_l0[0] - mv_col[0]);
                        sl->mv_cache[1][h264_scan8[i8 * 4 + i4]][1] =
                            (int16_t)(mv_l0[1] - mv_col[1]);
                    }
                }
            }
        }
    }
}

void h264_pred_direct_motion(H264Decoder *h, H264Slice *sl, uint32_t *mb_type)
{
    if (sl->direct_spatial_mv_pred)
        pred_spatial_direct_motion(h, sl, mb_type);
    else
        pred_temp_direct_motion(h, sl, mb_type);
}
