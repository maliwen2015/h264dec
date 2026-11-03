/*
 * H.264 macroblock reconstruction: motion compensation, intra prediction,
 * transform addition, MV prediction and neighbour caches.
 *
 * Ported from FFmpeg's h264_mb.c / h264_mb_template.c / h264_mvpred.h
 * (LGPL-2.1+), adapted to this decoder's structures. 8-bit 4:2:0 only.
 */
#include "h264_internal.h"

#define MB_FIELD(sl) ((sl)->mb_field_decoding_flag)
#define MB_MBAFF(sl) ((sl)->mb_mbaff)
#define FRAME_MBAFF(f) ((f)->mbaff)

#define mid_pred h264_mid_pred

static void h264_hl_decode_mb_16(H264Decoder *h, H264Slice *sl);

/* ------------------------------------------------------------------ */
/* metadata write-back                                                 */
/* ------------------------------------------------------------------ */
void h264_write_back_intra_pred_mode(H264Decoder *h, H264Slice *sl)
{
    int8_t *i4x4 = sl->intra4x4_pred_mode + h->mb2br_xy[sl->mb_xy];
    const int8_t *cache = sl->intra4x4_pred_mode_cache;

    memcpy(i4x4, cache + 4 + 8 * 4, 4);
    i4x4[4] = cache[7 + 8 * 3];
    i4x4[5] = cache[7 + 8 * 2];
    i4x4[6] = cache[7 + 8 * 1];
}

void h264_write_back_non_zero_count(H264Decoder *h, H264Slice *sl)
{
    H264Frame *f = h->cur_pic;
    uint8_t *nnz = f->nnz[sl->mb_xy];
    const uint8_t *nc = sl->non_zero_count_cache;

    memcpy(&nnz[0], &nc[4 + 8 * 1], 4);
    memcpy(&nnz[4], &nc[4 + 8 * 2], 4);
    memcpy(&nnz[8], &nc[4 + 8 * 3], 4);
    memcpy(&nnz[12], &nc[4 + 8 * 4], 4);
    memcpy(&nnz[16], &nc[4 + 8 * 6], 4);
    memcpy(&nnz[20], &nc[4 + 8 * 7], 4);
    memcpy(&nnz[32], &nc[4 + 8 * 11], 4);
    memcpy(&nnz[36], &nc[4 + 8 * 12], 4);
    if (!f->chroma_shift_y) {
        memcpy(&nnz[24], &nc[4 + 8 * 8], 4);
        memcpy(&nnz[28], &nc[4 + 8 * 9], 4);
        memcpy(&nnz[40], &nc[4 + 8 * 13], 4);
        memcpy(&nnz[44], &nc[4 + 8 * 14], 4);
    }
}

static void write_back_motion_list(H264Decoder *h, H264Slice *sl, uint32_t mb_type, int list)
{
    H264Frame *f = h->cur_pic;
    const int b_stride = f->b_stride;
    const int b_xy = 4 * sl->mb_x + 4 * sl->mb_y * b_stride;
    const int b8_xy = 4 * sl->mb_xy;
    int16_t (*mv_dst)[2] = &f->motion_val[list][b_xy];
    int16_t (*mv_src)[2] = &sl->mv_cache[list][h264_scan8[0]];
    int i;

    for (i = 0; i < 4; i++)
        memcpy(mv_dst + i * b_stride, mv_src + 8 * i, 16);
    if (f->cabac) {
        uint8_t (*mvd_dst)[2] = f->mvd[list][sl->mb_xy];
        uint8_t (*mvd_src)[2] = &sl->mvd_cache[list][h264_scan8[0]];
        if (IS_SKIP(mb_type)) {
            memset(mvd_dst, 0, 16);
        } else {
            /* entries 0..3: bottom row (blocks 12..15);
             * 4..6: right column rows 2,1,0; 7 unused */
            mvd_dst[0][0] = mvd_src[8 * 3 + 0][0]; mvd_dst[0][1] = mvd_src[8 * 3 + 0][1];
            mvd_dst[1][0] = mvd_src[8 * 3 + 1][0]; mvd_dst[1][1] = mvd_src[8 * 3 + 1][1];
            mvd_dst[2][0] = mvd_src[8 * 3 + 2][0]; mvd_dst[2][1] = mvd_src[8 * 3 + 2][1];
            mvd_dst[3][0] = mvd_src[8 * 3 + 3][0]; mvd_dst[3][1] = mvd_src[8 * 3 + 3][1];
            mvd_dst[4][0] = mvd_src[3 + 8 * 2][0]; mvd_dst[4][1] = mvd_src[3 + 8 * 2][1];
            mvd_dst[5][0] = mvd_src[3 + 8 * 1][0]; mvd_dst[5][1] = mvd_src[3 + 8 * 1][1];
            mvd_dst[6][0] = mvd_src[3 + 8 * 0][0]; mvd_dst[6][1] = mvd_src[3 + 8 * 0][1];
        }
    }
    {
        int8_t *ref_index = &f->ref_index[list][b8_xy];
        int8_t *ref_cache = sl->ref_cache[list];
        ref_index[0] = ref_cache[h264_scan8[0]];
        ref_index[1] = ref_cache[h264_scan8[4]];
        ref_index[2] = ref_cache[h264_scan8[8]];
        ref_index[3] = ref_cache[h264_scan8[12]];
    }
}

void h264_write_back_motion(H264Decoder *h, H264Slice *sl, uint32_t mb_type)
{
    H264Frame *f = h->cur_pic;
    if (USES_LIST(mb_type, 0)) {
        write_back_motion_list(h, sl, mb_type, 0);
    } else {
        memset(&f->ref_index[0][4 * sl->mb_xy], LIST_NOT_USED, 4);
    }
    if (USES_LIST(mb_type, 1))
        write_back_motion_list(h, sl, mb_type, 1);

    if (sl->slice_type_nos == H264_SLICE_B && f->cabac) {
        if (IS_8X8(mb_type)) {
            uint8_t *direct_table = &f->direct_table[4 * sl->mb_xy];
            direct_table[1] = (uint8_t)(sl->sub_mb_type[1] >> 1);
            direct_table[2] = (uint8_t)(sl->sub_mb_type[2] >> 1);
            direct_table[3] = (uint8_t)(sl->sub_mb_type[3] >> 1);
        }
    }
}

/* ------------------------------------------------------------------ */
/* MV prediction                                                       */
/* ------------------------------------------------------------------ */
/* Diagonal (top-right, falling back to top-left) neighbour of the 8x8 grid
 * slot i.  Returns its reference index and points *C at its MV. */
static int fetch_diagonal_mv(H264Decoder *h, H264Slice *sl, const int16_t **C,
                             int i, int list, int part_width)
{
    H264Frame *f = h->cur_pic;
    const int topright_ref = sl->ref_cache[list][i - 8 + part_width];

    if (FRAME_MBAFF(f)) {
        const uint32_t *mb_types = f->mb_type;
        const int16_t *mv;

        if (topright_ref == PART_NOT_AVAILABLE &&
            i >= h264_scan8[0] + 8 && (i & 7) == 4 &&
            sl->ref_cache[list][h264_scan8[0] - 1] != PART_NOT_AVAILABLE) {
            int16_t *diag = sl->mv_cache[list][h264_scan8[0] - 2];
            diag[0] = diag[1] = 0;
            *C = diag;

#define SET_DIAG_MV(MV_OP, REF_OP, XY, Y4)                                     \
            do {                                                               \
                const int xy = (XY), y4 = (Y4);                                \
                const uint32_t mt = mb_types[xy + (y4 >> 2) * f->mb_stride];   \
                if (USES_LIST(mt, list)) {                                     \
                    mv = f->motion_val[list][h->mb2b_xy[xy] + 3 +              \
                                           y4 * f->b_stride];                  \
                    diag[0] = mv[0];                                           \
                    diag[1] = (int16_t)(mv[1] MV_OP);                           \
                    *C = diag;                                                 \
                    return f->ref_index[list][4 * xy + 1 + (y4 & ~1)] REF_OP;   \
                }                                                              \
                return LIST_NOT_USED;                                          \
            } while (0)

            if (!MB_FIELD(sl) && IS_INTERLACED(sl->left_type[0]))
                SET_DIAG_MV(* 2, >> 1, sl->left_mb_xy[0] + f->mb_stride,
                            (sl->mb_y & 1) * 2 + (i >> 5));
            if (MB_FIELD(sl) && !IS_INTERLACED(sl->left_type[0]))
                SET_DIAG_MV(/ 2, * 2, sl->left_mb_xy[i >= 36], (i >> 2) & 3);
#undef SET_DIAG_MV
        }
    }

    if (topright_ref != PART_NOT_AVAILABLE) {
        *C = sl->mv_cache[list][i - 8 + part_width];
        return topright_ref;
    }
    *C = sl->mv_cache[list][i - 8 - 1];
    return sl->ref_cache[list][i - 8 - 1];
}

static void pred_motion(H264Decoder *h, H264Slice *sl, int n, int part_width,
                        int list, int ref, int *mx, int *my)
{
    const int index8 = h264_scan8[n];
    const int top_ref = sl->ref_cache[list][index8 - 8];
    const int left_ref = sl->ref_cache[list][index8 - 1];
    const int16_t *const A = sl->mv_cache[list][index8 - 1];
    const int16_t *const B = sl->mv_cache[list][index8 - 8];
    const int16_t *C;
    int diagonal_ref, match_count;

    diagonal_ref = fetch_diagonal_mv(h, sl, &C, index8, list, part_width);

    match_count = (diagonal_ref == ref) + (top_ref == ref) + (left_ref == ref);
    if (match_count > 1) {
        *mx = mid_pred(A[0], B[0], C[0]);
        *my = mid_pred(A[1], B[1], C[1]);
    } else if (match_count == 1) {
        if (left_ref == ref) { *mx = A[0]; *my = A[1]; }
        else if (top_ref == ref) { *mx = B[0]; *my = B[1]; }
        else { *mx = C[0]; *my = C[1]; }
    } else {
        if (top_ref == PART_NOT_AVAILABLE && diagonal_ref == PART_NOT_AVAILABLE &&
            left_ref != PART_NOT_AVAILABLE) {
            *mx = A[0];
            *my = A[1];
        } else {
            *mx = mid_pred(A[0], B[0], C[0]);
            *my = mid_pred(A[1], B[1], C[1]);
        }
    }
}

static void pred_16x8_motion(H264Decoder *h, H264Slice *sl, int n, int list, int ref,
                             int *mx, int *my)
{
    if (n == 0) {
        const int top_ref = sl->ref_cache[list][h264_scan8[0] - 8];
        const int16_t *const B = sl->mv_cache[list][h264_scan8[0] - 8];
        if (top_ref == ref) {
            *mx = B[0];
            *my = B[1];
            return;
        }
    } else {
        const int left_ref = sl->ref_cache[list][h264_scan8[8] - 1];
        const int16_t *const A = sl->mv_cache[list][h264_scan8[8] - 1];
        if (left_ref == ref) {
            *mx = A[0];
            *my = A[1];
            return;
        }
    }
    pred_motion(h, sl, n, 4, list, ref, mx, my);
}

static void pred_8x16_motion(H264Decoder *h, H264Slice *sl, int n, int list, int ref,
                             int *mx, int *my)
{
    if (n == 0) {
        const int left_ref = sl->ref_cache[list][h264_scan8[0] - 1];
        const int16_t *const A = sl->mv_cache[list][h264_scan8[0] - 1];
        if (left_ref == ref) {
            *mx = A[0];
            *my = A[1];
            return;
        }
    } else {
        const int16_t *C;
        const int diagonal_ref = fetch_diagonal_mv(h, sl, &C, h264_scan8[4],
                                                   list, 2);
        if (diagonal_ref == ref) {
            *mx = C[0];
            *my = C[1];
            return;
        }
    }
    pred_motion(h, sl, n, 2, list, ref, mx, my);
}

static void pred_pskip_motion(H264Decoder *h, H264Slice *sl)
{
    H264Frame *f = h->cur_pic;
    static const int16_t zeromv[2] = { 0, 0 };
    int16_t mvbuf[3][2];
    const int8_t *ref = f->ref_index[0];
    int16_t (*mv)[2] = f->motion_val[0];
    int top_ref, left_ref, diagonal_ref, match_count, mx, my;
    const int16_t *A, *B, *C;
    const int b_stride = f->b_stride;
    int i;

    for (i = 0; i < 4; i++) {
        int8_t *r = &sl->ref_cache[0][h264_scan8[0] + 8 * i];
        r[0] = r[1] = r[2] = r[3] = 0;
    }

#define FIX_MV_MBAFF(type, refn, mvn, idx)      \
    if (FRAME_MBAFF(f)) {                       \
        if (MB_FIELD(sl)) {                     \
            if (!IS_INTERLACED(type)) {         \
                refn *= 2;                     \
                mvbuf[idx][0] = (mvn)[0];       \
                mvbuf[idx][1] = (mvn)[1] / 2;   \
                mvn = mvbuf[idx];               \
            }                                   \
        } else {                                \
            if (IS_INTERLACED(type)) {          \
                refn >>= 1;                     \
                mvbuf[idx][0] = (mvn)[0];       \
                mvbuf[idx][1] = (mvn)[1] * 2;   \
                mvn = mvbuf[idx];               \
            }                                   \
        }                                       \
    }

    if (USES_LIST(sl->left_type[0], 0)) {
        left_ref = ref[4 * sl->left_mb_xy[0] + 1 + (sl->left_block[0] & ~1)];
        A = mv[h->mb2b_xy[sl->left_mb_xy[0]] + 3 + b_stride * sl->left_block[0]];
        FIX_MV_MBAFF(sl->left_type[0], left_ref, A, 0);
        if (!(left_ref | A[0] | A[1]))
            goto zeromv;
    } else if (sl->left_type[0]) {
        left_ref = LIST_NOT_USED;
        A = zeromv;
    } else {
        goto zeromv;
    }

    if (USES_LIST(sl->top_type, 0)) {
        top_ref = ref[4 * sl->top_mb_xy + 2];
        B = mv[h->mb2b_xy[sl->top_mb_xy] + 3 * b_stride];
        FIX_MV_MBAFF(sl->top_type, top_ref, B, 1);
        if (!(top_ref | B[0] | B[1]))
            goto zeromv;
    } else if (sl->top_type) {
        top_ref = LIST_NOT_USED;
        B = zeromv;
    } else {
        goto zeromv;
    }

    if (USES_LIST(sl->topright_type, 0)) {
        diagonal_ref = ref[4 * sl->topright_mb_xy + 2];
        C = mv[h->mb2b_xy[sl->topright_mb_xy] + 3 * b_stride];
        FIX_MV_MBAFF(sl->topright_type, diagonal_ref, C, 2);
    } else if (sl->topright_type) {
        diagonal_ref = LIST_NOT_USED;
        C = zeromv;
    } else {
        if (USES_LIST(sl->topleft_type, 0)) {
            diagonal_ref = ref[4 * sl->topleft_mb_xy + 1 + (sl->topleft_partition & 2)];
            C = mv[h->mb2b_xy[sl->topleft_mb_xy] + 3 + b_stride +
                   (sl->topleft_partition & 2 * b_stride)];
            FIX_MV_MBAFF(sl->topleft_type, diagonal_ref, C, 2);
        } else if (sl->topleft_type) {
            diagonal_ref = LIST_NOT_USED;
            C = zeromv;
        } else {
            diagonal_ref = PART_NOT_AVAILABLE;
            C = zeromv;
        }
    }

    match_count = !diagonal_ref + !top_ref + !left_ref;
    if (match_count > 1) {
        mx = mid_pred(A[0], B[0], C[0]);
        my = mid_pred(A[1], B[1], C[1]);
    } else if (match_count == 1) {
        if (!left_ref) { mx = A[0]; my = A[1]; }
        else if (!top_ref) { mx = B[0]; my = B[1]; }
        else { mx = C[0]; my = C[1]; }
    } else {
        mx = mid_pred(A[0], B[0], C[0]);
        my = mid_pred(A[1], B[1], C[1]);
    }
    {
        int i;
        for (i = 0; i < 4; i++)
            memset(&sl->mv_cache[0][h264_scan8[0] + 8 * i], 0, 0); /* no-op */
    }
    for (int i = 0; i < 4; i++) {
        int16_t (*dst)[2] = &sl->mv_cache[0][h264_scan8[0] + 8 * i];
        int j;
        for (j = 0; j < 4; j++) {
            dst[j][0] = (int16_t)mx;
            dst[j][1] = (int16_t)my;
        }
    }
    return;

zeromv:
    for (int i = 0; i < 4; i++) {
        int16_t (*dst)[2] = &sl->mv_cache[0][h264_scan8[0] + 8 * i];
        int j;
        for (j = 0; j < 4; j++) {
            dst[j][0] = 0;
            dst[j][1] = 0;
        }
    }
#undef FIX_MV_MBAFF
}

/* ------------------------------------------------------------------ */
/* neighbour/cache setup                                               */
/* ------------------------------------------------------------------ */
static void fill_decode_neighbors(H264Decoder *h, H264Slice *sl, uint32_t mb_type)
{
    H264Frame *f = h->cur_pic;
    const int mb_xy = sl->mb_xy;
    int topleft_xy, top_xy, topright_xy, left_xy[2];
    static const uint8_t left_block_options[4][32] = {
        { 0, 1, 2, 3, 7, 10, 8, 11, 3 + 0 * 4, 3 + 1 * 4, 3 + 2 * 4, 3 + 3 * 4, 1 + 4 * 4, 1 + 8 * 4, 1 + 5 * 4, 1 + 9 * 4 },
        { 2, 2, 3, 3, 8, 11, 8, 11, 3 + 2 * 4, 3 + 2 * 4, 3 + 3 * 4, 3 + 3 * 4, 1 + 5 * 4, 1 + 9 * 4, 1 + 5 * 4, 1 + 9 * 4 },
        { 0, 0, 1, 1, 7, 10, 7, 10, 3 + 0 * 4, 3 + 0 * 4, 3 + 1 * 4, 3 + 1 * 4, 1 + 4 * 4, 1 + 8 * 4, 1 + 4 * 4, 1 + 8 * 4 },
        { 0, 2, 0, 2, 7, 10, 7, 10, 3 + 0 * 4, 3 + 2 * 4, 3 + 0 * 4, 3 + 2 * 4, 1 + 4 * 4, 1 + 8 * 4, 1 + 4 * 4, 1 + 8 * 4 }
    };

    sl->topleft_partition = -1;
    top_xy = mb_xy - (f->mb_stride << MB_FIELD(sl));
    topleft_xy = top_xy - 1;
    topright_xy = top_xy + 1;
    left_xy[0] = left_xy[1] = mb_xy - 1;
    sl->left_block = left_block_options[0];

    if (FRAME_MBAFF(f)) {
        const int left_mb_field_flag = IS_INTERLACED(f->mb_type[mb_xy - 1]);
        const int curr_mb_field_flag = IS_INTERLACED(mb_type);
        if (sl->mb_y & 1) {
            if (left_mb_field_flag != curr_mb_field_flag) {
                left_xy[0] = left_xy[1] = mb_xy - f->mb_stride - 1;
                if (curr_mb_field_flag) {
                    left_xy[1] += f->mb_stride;
                    sl->left_block = left_block_options[3];
                } else {
                    topleft_xy += f->mb_stride;
                    sl->topleft_partition = 0;
                    sl->left_block = left_block_options[1];
                }
            }
        } else {
            if (curr_mb_field_flag) {
                topleft_xy += f->mb_stride &
                              (((f->mb_type[top_xy - 1] >> 7) & 1) - 1);
                topright_xy += f->mb_stride &
                               (((f->mb_type[top_xy + 1] >> 7) & 1) - 1);
                top_xy += f->mb_stride & (((f->mb_type[top_xy] >> 7) & 1) - 1);
            }
            if (left_mb_field_flag != curr_mb_field_flag) {
                if (curr_mb_field_flag) {
                    left_xy[1] += f->mb_stride;
                    sl->left_block = left_block_options[3];
                } else {
                    sl->left_block = left_block_options[2];
                }
            }
        }
    }

    sl->topleft_mb_xy = topleft_xy;
    sl->top_mb_xy = top_xy;
    sl->topright_mb_xy = topright_xy;
    sl->left_mb_xy[0] = left_xy[0];
    sl->left_mb_xy[1] = left_xy[1];

    sl->topleft_type = (int)f->mb_type[topleft_xy];
    sl->top_type = (int)f->mb_type[top_xy];
    sl->topright_type = (int)f->mb_type[topright_xy];
    sl->left_type[0] = (int)f->mb_type[left_xy[0]];
    sl->left_type[1] = (int)f->mb_type[left_xy[1]];

    if (f->slice_table[topleft_xy] != sl->slice_num) {
        sl->topleft_type = 0;
        if (f->slice_table[top_xy] != sl->slice_num)
            sl->top_type = 0;
        if (f->slice_table[left_xy[0]] != sl->slice_num)
            sl->left_type[0] = sl->left_type[1] = 0;
    }
    if (f->slice_table[topright_xy] != sl->slice_num)
        sl->topright_type = 0;
}

static void fill_decode_caches(H264Decoder *h, H264Slice *sl, uint32_t mb_type)
{
    H264Frame *f = h->cur_pic;
    int topleft_xy = sl->topleft_mb_xy, top_xy = sl->top_mb_xy;
    int topright_xy = sl->topright_mb_xy;
    int left_xy[2] = { sl->left_mb_xy[0], sl->left_mb_xy[1] };
    int topleft_type = sl->topleft_type, top_type = sl->top_type;
    int topright_type = sl->topright_type;
    int left_type[2] = { sl->left_type[0], sl->left_type[1] };
    const uint8_t *left_block = sl->left_block;
    int i;
    uint8_t *nnz;
    uint8_t *nnz_cache;

    if (!IS_SKIP(mb_type)) {
        if (IS_INTRA(mb_type)) {
            int type_mask = h->active_pps->constrained_intra_pred ? IS_INTRA(-1) : -1;
            sl->topleft_samples_available = sl->top_samples_available =
                sl->left_samples_available = 0xFFFF;
            sl->topright_samples_available = 0xEEEA;

            if (!(top_type & type_mask)) {
                sl->topleft_samples_available = 0xB3FF;
                sl->top_samples_available = 0x33FF;
                sl->topright_samples_available = 0x26EA;
            }
            if (IS_INTERLACED(mb_type) != IS_INTERLACED(left_type[0])) {
                if (IS_INTERLACED(mb_type)) {
                    if (!(left_type[0] & type_mask)) {
                        sl->topleft_samples_available &= 0xDFFF;
                        sl->left_samples_available &= 0x5FFF;
                    }
                    if (!(left_type[1] & type_mask)) {
                        sl->topleft_samples_available &= 0xFF5F;
                        sl->left_samples_available &= 0xFF5F;
                    }
                } else {
                    int left_typei = (int)f->mb_type[left_xy[0] + f->mb_stride];
                    if (!((left_typei & type_mask) && (left_type[0] & type_mask))) {
                        sl->topleft_samples_available &= 0xDF5F;
                        sl->left_samples_available &= 0x5F5F;
                    }
                }
            } else {
                if (!(left_type[0] & type_mask)) {
                    sl->topleft_samples_available &= 0xDF5F;
                    sl->left_samples_available &= 0x5F5F;
                }
            }
            if (!(topleft_type & type_mask))
                sl->topleft_samples_available &= 0x7FFF;
            if (!(topright_type & type_mask))
                sl->topright_samples_available &= 0xFBFF;

            if (IS_INTRA4x4(mb_type)) {
                if (IS_INTRA4x4(top_type)) {
                    memcpy(sl->intra4x4_pred_mode_cache + 4 + 8 * 0,
                           sl->intra4x4_pred_mode + h->mb2br_xy[top_xy], 4);
                } else {
                    sl->intra4x4_pred_mode_cache[4 + 8 * 0] =
                    sl->intra4x4_pred_mode_cache[5 + 8 * 0] =
                    sl->intra4x4_pred_mode_cache[6 + 8 * 0] =
                    sl->intra4x4_pred_mode_cache[7 + 8 * 0] =
                        (int8_t)(2 - 3 * !(top_type & type_mask));
                }
                for (i = 0; i < 2; i++) {
                    if (IS_INTRA4x4(left_type[i])) {
                        const int8_t *mode = sl->intra4x4_pred_mode + h->mb2br_xy[left_xy[i]];
                        sl->intra4x4_pred_mode_cache[3 + 8 * 1 + 2 * 8 * i] =
                            mode[6 - left_block[0 + 2 * i]];
                        sl->intra4x4_pred_mode_cache[3 + 8 * 2 + 2 * 8 * i] =
                            mode[6 - left_block[1 + 2 * i]];
                    } else {
                        sl->intra4x4_pred_mode_cache[3 + 8 * 1 + 2 * 8 * i] =
                        sl->intra4x4_pred_mode_cache[3 + 8 * 2 + 2 * 8 * i] =
                            (int8_t)(2 - 3 * !(left_type[i] & type_mask));
                    }
                }
            }
        }

        nnz_cache = sl->non_zero_count_cache;
        if (top_type) {
            nnz = f->nnz[top_xy];
            memcpy(&nnz_cache[4 + 8 * 0], &nnz[4 * 3], 4);
            if (!f->chroma_shift_y) {
                memcpy(&nnz_cache[4 + 8 * 5], &nnz[4 * 7], 4);
                memcpy(&nnz_cache[4 + 8 * 10], &nnz[4 * 11], 4);
            } else {
                memcpy(&nnz_cache[4 + 8 * 5], &nnz[4 * 5], 4);
                memcpy(&nnz_cache[4 + 8 * 10], &nnz[4 * 9], 4);
            }
        } else {
            uint32_t top_empty = (f->cabac && !IS_INTRA(mb_type)) ? 0 : 0x40404040u;
            memset(&nnz_cache[4 + 8 * 0], 0, 1);
            memcpy(&nnz_cache[4 + 8 * 0], &top_empty, 4);
            memcpy(&nnz_cache[4 + 8 * 5], &top_empty, 4);
            memcpy(&nnz_cache[4 + 8 * 10], &top_empty, 4);
        }

        for (i = 0; i < 2; i++) {
            if (left_type[i]) {
                nnz = f->nnz[left_xy[i]];
                nnz_cache[3 + 8 * 1 + 2 * 8 * i] = nnz[left_block[8 + 0 + 2 * i]];
                nnz_cache[3 + 8 * 2 + 2 * 8 * i] = nnz[left_block[8 + 1 + 2 * i]];
                if (f->chroma_format_idc == 3) {
                    nnz_cache[3 + 8 * 6 + 2 * 8 * i] = nnz[left_block[8 + 0 + 2 * i] + 4 * 4];
                    nnz_cache[3 + 8 * 7 + 2 * 8 * i] = nnz[left_block[8 + 1 + 2 * i] + 4 * 4];
                    nnz_cache[3 + 8 * 11 + 2 * 8 * i] = nnz[left_block[8 + 0 + 2 * i] + 8 * 4];
                    nnz_cache[3 + 8 * 12 + 2 * 8 * i] = nnz[left_block[8 + 1 + 2 * i] + 8 * 4];
                } else if (f->chroma_format_idc == 2) {
                    nnz_cache[3 + 8 * 6 + 2 * 8 * i] = nnz[left_block[8 + 0 + 2 * i] - 2 + 4 * 4];
                    nnz_cache[3 + 8 * 7 + 2 * 8 * i] = nnz[left_block[8 + 1 + 2 * i] - 2 + 4 * 4];
                    nnz_cache[3 + 8 * 11 + 2 * 8 * i] = nnz[left_block[8 + 0 + 2 * i] - 2 + 8 * 4];
                    nnz_cache[3 + 8 * 12 + 2 * 8 * i] = nnz[left_block[8 + 1 + 2 * i] - 2 + 8 * 4];
                } else {
                    nnz_cache[3 + 8 * 6 + 8 * i] = nnz[left_block[8 + 4 + 2 * i]];
                    nnz_cache[3 + 8 * 11 + 8 * i] = nnz[left_block[8 + 5 + 2 * i]];
                }
            } else {
                sl->non_zero_count_cache[3 + 8 * 1 + 2 * 8 * i] =
                sl->non_zero_count_cache[3 + 8 * 2 + 2 * 8 * i] =
                sl->non_zero_count_cache[3 + 8 * 6 + 2 * 8 * i] =
                sl->non_zero_count_cache[3 + 8 * 7 + 2 * 8 * i] =
                sl->non_zero_count_cache[3 + 8 * 11 + 2 * 8 * i] =
                sl->non_zero_count_cache[3 + 8 * 12 + 2 * 8 * i] =
                    (uint8_t)((f->cabac && !IS_INTRA(mb_type)) ? 0 : 64);
            }
        }

        if (f->cabac) {
            if (top_type)
                sl->top_cbp = f->cbp_table[top_xy];
            else
                sl->top_cbp = IS_INTRA(mb_type) ? 0x7CF : 0x00F;
            if (left_type[0]) {
                sl->left_cbp = (f->cbp_table[left_xy[0]] & 0x7F0) |
                               ((f->cbp_table[left_xy[0]] >> (left_block[0] & (~1))) & 2) |
                               (((f->cbp_table[left_xy[1]] >> (left_block[2] & (~1))) & 2) << 2);
            } else {
                sl->left_cbp = IS_INTRA(mb_type) ? 0x7CF : 0x00F;
            }
        }
    }

    if (IS_INTER(mb_type) || (IS_DIRECT(mb_type) && sl->direct_spatial_mv_pred)) {
        int list;
        const int b_stride = f->b_stride;
        for (list = 0; list < sl->list_count; list++) {
            int8_t *ref_cache = &sl->ref_cache[list][h264_scan8[0]];
            const int8_t *ref = f->ref_index[list];
            int16_t (*mv_cache)[2] = &sl->mv_cache[list][h264_scan8[0]];
            int16_t (*mv)[2] = f->motion_val[list];
            if (!USES_LIST(mb_type, list))
                continue;

            if (USES_LIST(top_type, list)) {
                const int b_xy = h->mb2b_xy[top_xy] + 3 * b_stride;
                memcpy(mv_cache - 8, mv[b_xy], 16);
                ref_cache[0 - 8] = ref_cache[1 - 8] = ref[4 * top_xy + 2];
                ref_cache[2 - 8] = ref_cache[3 - 8] = ref[4 * top_xy + 3];
            } else {
                memset(mv_cache - 8, 0, 16);
                memset(ref_cache - 8, (uint8_t)((top_type ? LIST_NOT_USED : PART_NOT_AVAILABLE) & 0xFF), 4);
            }

            if (mb_type & (MB_TYPE_16x8 | MB_TYPE_8x8)) {
                for (i = 0; i < 2; i++) {
                    int cache_idx = -1 + i * 2 * 8;
                    if (USES_LIST(left_type[i], list)) {
                        const int b_xy = h->mb2b_xy[left_xy[i]] + 3;
                        const int b8_xy = 4 * left_xy[i] + 1;
                        memcpy(mv_cache[cache_idx], mv[b_xy + b_stride * left_block[0 + i * 2]], 4);
                        memcpy(mv_cache[cache_idx + 8], mv[b_xy + b_stride * left_block[1 + i * 2]], 4);
                        ref_cache[cache_idx] = ref[b8_xy + (left_block[0 + i * 2] & ~1)];
                        ref_cache[cache_idx + 8] = ref[b8_xy + (left_block[1 + i * 2] & ~1)];
                    } else {
                        memset(mv_cache[cache_idx], 0, 4);
                        memset(mv_cache[cache_idx + 8], 0, 4);
                        ref_cache[cache_idx] = ref_cache[cache_idx + 8] =
                            (left_type[i] ? LIST_NOT_USED : PART_NOT_AVAILABLE);
                    }
                }
            } else {
                if (USES_LIST(left_type[0], list)) {
                    const int b_xy = h->mb2b_xy[left_xy[0]] + 3;
                    const int b8_xy = 4 * left_xy[0] + 1;
                    memcpy(mv_cache[-1], mv[b_xy + b_stride * left_block[0]], 4);
                    ref_cache[-1] = ref[b8_xy + (left_block[0] & ~1)];
                } else {
                    memset(mv_cache[-1], 0, 4);
                    ref_cache[-1] = left_type[0] ? LIST_NOT_USED : PART_NOT_AVAILABLE;
                }
            }

            if (USES_LIST(topright_type, list)) {
                const int b_xy = h->mb2b_xy[topright_xy] + 3 * b_stride;
                memcpy(mv_cache[4 - 8], mv[b_xy], 4);
                ref_cache[4 - 8] = ref[4 * topright_xy + 2];
            } else {
                memset(mv_cache[4 - 8], 0, 4);
                ref_cache[4 - 8] = topright_type ? LIST_NOT_USED : PART_NOT_AVAILABLE;
            }
            if (ref_cache[2 - 8] < 0 || ref_cache[4 - 8] < 0) {
                if (USES_LIST(topleft_type, list)) {
                    const int b_xy = h->mb2b_xy[topleft_xy] + 3 + b_stride +
                                     (sl->topleft_partition & 2 * b_stride);
                    const int b8_xy = 4 * topleft_xy + 1 + (sl->topleft_partition & 2);
                    memcpy(mv_cache[-1 - 8], mv[b_xy], 4);
                    ref_cache[-1 - 8] = ref[b8_xy];
                } else {
                    memset(mv_cache[-1 - 8], 0, 4);
                    ref_cache[-1 - 8] = topleft_type ? LIST_NOT_USED : PART_NOT_AVAILABLE;
                }
            }

            if ((mb_type & (MB_TYPE_SKIP | MB_TYPE_DIRECT2)) && !FRAME_MBAFF(f))
                continue;

            if (!(mb_type & (MB_TYPE_SKIP | MB_TYPE_DIRECT2))) {
                uint8_t (*mvd_cache)[2] = &sl->mvd_cache[list][h264_scan8[0]];
                ref_cache[2 + 8 * 0] = ref_cache[2 + 8 * 2] = PART_NOT_AVAILABLE;
                memset(mv_cache[2 + 8 * 0], 0, 4);
                memset(mv_cache[2 + 8 * 2], 0, 4);

                if (f->cabac) {
                    uint8_t (*mvd_top)[2] = USES_LIST(top_type, list) ? f->mvd[list][top_xy] : NULL;
                    uint8_t (*mvd_lt)[2] = USES_LIST(left_type[0], list) ? f->mvd[list][left_xy[0]] : NULL;
                    uint8_t (*mvd_lb)[2] = USES_LIST(left_type[1], list) ? f->mvd[list][left_xy[1]] : NULL;
                    if (mvd_top) {
                        memcpy(mvd_cache - 8, mvd_top, 8);
                    } else {
                        memset(mvd_cache - 8, 0, 8);
                    }
                    if (mvd_lt) {
                        mvd_cache[-1 + 0 * 8][0] = mvd_lt[6 - left_block[0]][0];
                        mvd_cache[-1 + 0 * 8][1] = mvd_lt[6 - left_block[0]][1];
                        mvd_cache[-1 + 1 * 8][0] = mvd_lt[6 - left_block[1]][0];
                        mvd_cache[-1 + 1 * 8][1] = mvd_lt[6 - left_block[1]][1];
                    } else {
                        memset(mvd_cache[-1 + 0 * 8], 0, 2);
                        memset(mvd_cache[-1 + 1 * 8], 0, 2);
                    }
                    if (mvd_lb) {
                        mvd_cache[-1 + 2 * 8][0] = mvd_lb[6 - left_block[2]][0];
                        mvd_cache[-1 + 2 * 8][1] = mvd_lb[6 - left_block[2]][1];
                        mvd_cache[-1 + 3 * 8][0] = mvd_lb[6 - left_block[3]][0];
                        mvd_cache[-1 + 3 * 8][1] = mvd_lb[6 - left_block[3]][1];
                    } else {
                        memset(mvd_cache[-1 + 2 * 8], 0, 2);
                        memset(mvd_cache[-1 + 3 * 8], 0, 2);
                    }
                    memset(mvd_cache[2 + 8 * 0], 0, 2);
                    memset(mvd_cache[2 + 8 * 2], 0, 2);

                    if (sl->slice_type_nos == H264_SLICE_B) {
                        uint8_t *direct_cache = &sl->direct_cache[h264_scan8[0]];
                        const uint8_t *direct_table = f->direct_table;
                        int k;
                        for (k = 0; k < 4; k++)
                            memset(&direct_cache[8 * k], MB_TYPE_16x16 >> 1, 4);
                        if (IS_DIRECT(top_type)) {
                            memset(&direct_cache[-8], 0x01 * (MB_TYPE_DIRECT2 >> 1), 4);
                        } else if (IS_8X8(top_type)) {
                            int b8_xy = 4 * top_xy;
                            direct_cache[0 - 8] = direct_table[b8_xy + 2];
                            direct_cache[2 - 8] = direct_table[b8_xy + 3];
                        } else {
                            memset(&direct_cache[-8], 0x01 * (MB_TYPE_16x16 >> 1), 4);
                        }
                        if (IS_DIRECT(left_type[0]))
                            direct_cache[-1 + 0 * 8] = MB_TYPE_DIRECT2 >> 1;
                        else if (IS_8X8(left_type[0]))
                            direct_cache[-1 + 0 * 8] = direct_table[4 * left_xy[0] + 1 + (left_block[0] & ~1)];
                        else
                            direct_cache[-1 + 0 * 8] = MB_TYPE_16x16 >> 1;
                        if (IS_DIRECT(left_type[1]))
                            direct_cache[-1 + 2 * 8] = MB_TYPE_DIRECT2 >> 1;
                        else if (IS_8X8(left_type[1]))
                            direct_cache[-1 + 2 * 8] = direct_table[4 * left_xy[1] + 1 + (left_block[2] & ~1)];
                        else
                            direct_cache[-1 + 2 * 8] = MB_TYPE_16x16 >> 1;
                    }
                }
            }

#define MAP_F2F(idx, mbt)                                                \
    if (MB_FIELD(sl)) {                                                  \
        if (!IS_INTERLACED(mbt) && sl->ref_cache[list][idx] >= 0) {      \
            sl->ref_cache[list][idx] *= 2;                               \
            sl->mv_cache[list][idx][1] /= 2;                             \
            sl->mvd_cache[list][idx][1] >>= 1;                           \
        }                                                                \
    } else {                                                             \
        if (IS_INTERLACED(mbt) && sl->ref_cache[list][idx] >= 0) {       \
            sl->ref_cache[list][idx] >>= 1;                              \
            sl->mv_cache[list][idx][1] *= 2;                             \
            sl->mvd_cache[list][idx][1] *= 2;                           \
        }                                                                \
    }

            if (FRAME_MBAFF(f)) {
                MAP_F2F(h264_scan8[0] - 1 - 8, topleft_type)
                MAP_F2F(h264_scan8[0] + 0 - 8, top_type)
                MAP_F2F(h264_scan8[0] + 1 - 8, top_type)
                MAP_F2F(h264_scan8[0] + 2 - 8, top_type)
                MAP_F2F(h264_scan8[0] + 3 - 8, top_type)
                MAP_F2F(h264_scan8[0] + 4 - 8, topright_type)
                MAP_F2F(h264_scan8[0] - 1 + 0 * 8, left_type[0])
                MAP_F2F(h264_scan8[0] - 1 + 1 * 8, left_type[0])
                MAP_F2F(h264_scan8[0] - 1 + 2 * 8, left_type[1])
                MAP_F2F(h264_scan8[0] - 1 + 3 * 8, left_type[1])
            }
#undef MAP_F2F
        }
    }

    sl->neighbor_transform_size = !!IS_8x8DCT(top_type) + !!IS_8x8DCT(left_type[0]);
}

void h264_pred_motion(H264Decoder *h, H264Slice *sl, int n, int part_width,
                      int list, int ref, int *mx, int *my)
{
    pred_motion(h, sl, n, part_width, list, ref, mx, my);
}

void h264_pred_16x8_motion(H264Decoder *h, H264Slice *sl, int n, int list, int ref,
                           int *mx, int *my)
{
    pred_16x8_motion(h, sl, n, list, ref, mx, my);
}

void h264_pred_8x16_motion(H264Decoder *h, H264Slice *sl, int n, int list, int ref,
                           int *mx, int *my)
{
    pred_8x16_motion(h, sl, n, list, ref, mx, my);
}

static void decode_mb_skip(H264Decoder *h, H264Slice *sl)
{
    H264Frame *f = h->cur_pic;
    const int mb_xy = sl->mb_xy;
    uint32_t mb_type = 0;

    memset(f->nnz[mb_xy], 0, 48);
    if (MB_FIELD(sl))
        mb_type |= MB_TYPE_INTERLACED;

    if (sl->slice_type_nos == H264_SLICE_B) {
        mb_type |= MB_TYPE_L0L1 | MB_TYPE_DIRECT2 | MB_TYPE_SKIP;
        if (sl->direct_spatial_mv_pred) {
            fill_decode_neighbors(h, sl, mb_type);
            fill_decode_caches(h, sl, mb_type);
        }
        h264_pred_direct_motion(h, sl, &mb_type);
        mb_type |= MB_TYPE_SKIP;
    } else {
        mb_type |= MB_TYPE_16x16 | MB_TYPE_P0L0 | MB_TYPE_P1L0 | MB_TYPE_SKIP;
        fill_decode_neighbors(h, sl, mb_type);
        pred_pskip_motion(h, sl);
    }

    h264_write_back_motion(h, sl, mb_type);
    f->mb_type[mb_xy] = mb_type;
    f->qscale_table[mb_xy] = (int8_t)sl->qscale;
    f->slice_table[mb_xy] = (uint16_t)sl->slice_num;
    sl->prev_mb_skipped = 1;
}

void h264_decode_mb_skip(H264Decoder *h, H264Slice *sl)
{
    decode_mb_skip(h, sl);
}

/* ------------------------------------------------------------------ */
/* motion compensation                                                 */
/* ------------------------------------------------------------------ */
/* With frame threading a reference picture may still be under construction.
 * A partition reaches down to its integer row plus the three extra rows the
 * vertical half-pel filter reads, so the producer's published loop-filter row
 * has to be at least that far before the pixels are touched. */
static void note_part_y(H264Decoder *h, H264Slice *sl, int16_t refs[2][48],
                        int nrefs[2], int list, int n, int height, int y_off,
                        int dir)
{
    const H264Ref *ref;
    int ref_n, line;

    if (!dir)
        return;
    y_off += 16 * (sl->mb_y >> MB_FIELD(sl));
    ref_n = sl->ref_cache[list][h264_scan8[n]];
    ref = &sl->ref_list[list][ref_n];
    /* error resilience can put this very picture in the list; waiting on it
     * would deadlock */
    if (!ref->parent ||
        (ref->parent == h->cur_pic && (ref->reference & 3) == h->picture_structure))
        return;
    {
        const int raw_my = sl->mv_cache[list][h264_scan8[n]][1];
        const int filter_down = (raw_my & 3) ? 3 : 0;
        line = H264_MAX(0, (raw_my >> 2) + y_off + filter_down + height);
    }
    if (refs[list][ref_n] < 0)
        nrefs[list]++;
    if (line > refs[list][ref_n])
        refs[list][ref_n] = (int16_t)line;
}

static void await_references(H264Decoder *h, H264Slice *sl, uint32_t mb_type)
{
    int16_t refs[2][48];
    int nrefs[2] = { 0, 0 };
    int list, ref;

    if (!h->threaded)
        return;
    memset(refs, -1, sizeof(refs));

    if (IS_16X16(mb_type)) {
        note_part_y(h, sl, refs, nrefs, 0, 0, 16, 0, IS_DIR(mb_type, 0, 0));
        note_part_y(h, sl, refs, nrefs, 1, 0, 16, 0, IS_DIR(mb_type, 0, 1));
    } else if (IS_16X8(mb_type)) {
        note_part_y(h, sl, refs, nrefs, 0, 0, 8, 0, IS_DIR(mb_type, 0, 0));
        note_part_y(h, sl, refs, nrefs, 1, 0, 8, 0, IS_DIR(mb_type, 0, 1));
        note_part_y(h, sl, refs, nrefs, 0, 8, 8, 8, IS_DIR(mb_type, 1, 0));
        note_part_y(h, sl, refs, nrefs, 1, 8, 8, 8, IS_DIR(mb_type, 1, 1));
    } else if (IS_8X16(mb_type)) {
        note_part_y(h, sl, refs, nrefs, 0, 0, 16, 0, IS_DIR(mb_type, 0, 0));
        note_part_y(h, sl, refs, nrefs, 1, 0, 16, 0, IS_DIR(mb_type, 0, 1));
        note_part_y(h, sl, refs, nrefs, 0, 4, 16, 0, IS_DIR(mb_type, 1, 0));
        note_part_y(h, sl, refs, nrefs, 1, 4, 16, 0, IS_DIR(mb_type, 1, 1));
    } else {
        int i;
        for (i = 0; i < 4; i++) {
            const int sub_mb_type = sl->sub_mb_type[i];
            const int n = 4 * i;
            const int y_off = (i & 2) << 2;
            if (IS_SUB_8X8(sub_mb_type)) {
                note_part_y(h, sl, refs, nrefs, 0, n, 8, y_off,
                            IS_DIR(sub_mb_type, 0, 0));
                note_part_y(h, sl, refs, nrefs, 1, n, 8, y_off,
                            IS_DIR(sub_mb_type, 0, 1));
            } else if (IS_SUB_8X4(sub_mb_type)) {
                note_part_y(h, sl, refs, nrefs, 0, n, 4, y_off,
                            IS_DIR(sub_mb_type, 0, 0));
                note_part_y(h, sl, refs, nrefs, 1, n, 4, y_off,
                            IS_DIR(sub_mb_type, 0, 1));
                note_part_y(h, sl, refs, nrefs, 0, n + 2, 4, y_off + 4,
                            IS_DIR(sub_mb_type, 0, 0));
                note_part_y(h, sl, refs, nrefs, 1, n + 2, 4, y_off + 4,
                            IS_DIR(sub_mb_type, 0, 1));
            } else if (IS_SUB_4X8(sub_mb_type)) {
                note_part_y(h, sl, refs, nrefs, 0, n, 8, y_off,
                            IS_DIR(sub_mb_type, 0, 0));
                note_part_y(h, sl, refs, nrefs, 1, n, 8, y_off,
                            IS_DIR(sub_mb_type, 0, 1));
                note_part_y(h, sl, refs, nrefs, 0, n + 1, 8, y_off,
                            IS_DIR(sub_mb_type, 0, 0));
                note_part_y(h, sl, refs, nrefs, 1, n + 1, 8, y_off,
                            IS_DIR(sub_mb_type, 0, 1));
            } else {
                int j;
                for (j = 0; j < 4; j++) {
                    const int sub_y_off = y_off + 2 * (j & 2);
                    note_part_y(h, sl, refs, nrefs, 0, n + j, 4, sub_y_off,
                                IS_DIR(sub_mb_type, 0, 0));
                    note_part_y(h, sl, refs, nrefs, 1, n + j, 4, sub_y_off,
                                IS_DIR(sub_mb_type, 0, 1));
                }
            }
        }
    }

    for (list = sl->list_count - 1; list >= 0; list--)
        for (ref = 0; ref < 48 && nrefs[list]; ref++) {
            const H264Ref *rp = &sl->ref_list[list][ref];
            const H264Frame *pic;
            int row = refs[list][ref];
            int ref_field, field_picture, pic_height;

            if (row < 0)
                continue;
            nrefs[list]--;
            if (!rp->parent)
                continue;
            pic = rp->parent;
            ref_field = rp->reference - 1;
            field_picture = pic->field_picture;
            pic_height = 16 * h->mb_height >> field_picture;
            row <<= MB_MBAFF(sl);

            if (h->picture_structure == PICT_FRAME && field_picture) {
                /* a frame uses both fields, which are decoded separately */
                h264_progress_await(h, pic,
                                    H264_MIN((row >> 1) - !(row & 1),
                                             pic_height - 1), 1);
                h264_progress_await(h, pic,
                                    H264_MIN(row >> 1, pic_height - 1), 0);
            } else if (h->picture_structure != PICT_FRAME && !field_picture) {
                h264_progress_await(h, pic,
                                    H264_MIN(row * 2 + ref_field,
                                             pic_height - 1), 0);
            } else if (h->picture_structure != PICT_FRAME) {
                h264_progress_await(h, pic, H264_MIN(row, pic_height - 1),
                                    ref_field);
            } else {
                h264_progress_await(h, pic, H264_MIN(row, pic_height - 1), 0);
            }
        }
}

static void mc_dir_part(H264Decoder *h, H264Slice *sl, H264Ref *pic, int n,
                        int w, int hgt, int x_off, int y_off,
                        int cw, int ch, int list,
                        uint8_t *dest_y, uint8_t *dest_cb, uint8_t *dest_cr,
                        int avg)
{
    H264Frame *f = h->cur_pic;
    /* mx/my are quarter-luma-pel units that include the MB and sub-block
     * position, so source addresses and edge emulation tests are absolute. */
    const int mx = sl->mv_cache[list][h264_scan8[n]][0] +
                   4 * (x_off + 16 * sl->mb_x);
    int my = sl->mv_cache[list][h264_scan8[n]][1] +
             4 * (y_off + 16 * (sl->mb_y >> MB_FIELD(sl)));
    const int full_mx = mx >> 2;
    const int full_my = my >> 2;
    const int pic_width = 16 * f->mb_width;
    const int pic_height = 16 * f->mb_height >> MB_FIELD(sl);
    const int extra_width = (mx & 7) ? -3 : 0;
    const int extra_height = (my & 7) ? -3 : 0;
    int emu = 0;
    uint8_t *src_y, *src_cb, *src_cr;

    src_y = pic->data[0] + full_mx + full_my * sl->mb_linesize;

    if (full_mx < 0 - extra_width || full_my < 0 - extra_height ||
        full_mx + 16 > pic_width + extra_width ||
        full_my + 16 > pic_height + extra_height) {
        h264_emulated_edge_mc(sl->edge_emu_buffer,
                              src_y - 2 - 2 * sl->mb_linesize,
                              sl->mb_linesize, 16 + 5, 16 + 5,
                              full_mx - 2, full_my - 2, pic_width, pic_height);
        src_y = sl->edge_emu_buffer + 2 + 2 * sl->mb_linesize;
        emu = 1;
    }

    h264_qpel_mc(dest_y, src_y, sl->mb_linesize, w, hgt, mx & 3, my & 3, avg);

    if (MB_FIELD(sl)) {
        /* chroma of a field macroblock samples the same-parity chroma rows of
         * the reference, which may be the opposite field of a frame */
        my += 2 * ((sl->mb_y & 1) - (pic->reference - 1));
        if ((mx >> 3) < 0 || (my >> 3) < 0 || (my >> 3) + ch >= (pic_height >> 1))
            emu = 1;
    }

    {
        const int chroma_x = mx >> 3;
        const int chroma_y = my >> 3;
        src_cb = pic->data[1] + chroma_x + chroma_y * sl->mb_uvlinesize;
        src_cr = pic->data[2] + chroma_x + chroma_y * sl->mb_uvlinesize;

        if (emu) {
            h264_emulated_edge_mc(sl->edge_emu_buffer, src_cb, sl->mb_uvlinesize,
                                  9, 9, chroma_x, chroma_y,
                                  pic_width >> 1, pic_height >> 1);
            src_cb = sl->edge_emu_buffer;
        }
        h264_chroma_mc(dest_cb, src_cb, sl->mb_uvlinesize, cw, ch, mx & 7, my & 7, avg);

        if (emu) {
            h264_emulated_edge_mc(sl->edge_emu_buffer, src_cr, sl->mb_uvlinesize,
                                  9, 9, chroma_x, chroma_y,
                                  pic_width >> 1, pic_height >> 1);
            src_cr = sl->edge_emu_buffer;
        }
        h264_chroma_mc(dest_cr, src_cr, sl->mb_uvlinesize, cw, ch, mx & 7, my & 7, avg);
    }
}

static void mc_part_std(H264Decoder *h, H264Slice *sl, int n, int lw, int lh,
                        int cw, int ch, int x_off, int y_off,
                        uint8_t *dest_y, uint8_t *dest_cb, uint8_t *dest_cr,
                        int list0, int list1)
{
    dest_y += x_off + y_off * sl->mb_linesize;
    dest_cb += (x_off >> 1) + (y_off >> 1) * sl->mb_uvlinesize;
    dest_cr += (x_off >> 1) + (y_off >> 1) * sl->mb_uvlinesize;

    if (list0) {
        H264Ref *ref = &sl->ref_list[0][sl->ref_cache[0][h264_scan8[n]]];
        mc_dir_part(h, sl, ref, n, lw, lh, x_off, y_off, cw, ch, 0,
                    dest_y, dest_cb, dest_cr, 0);
    }
    if (list1) {
        H264Ref *ref = &sl->ref_list[1][sl->ref_cache[1][h264_scan8[n]]];
        mc_dir_part(h, sl, ref, n, lw, lh, x_off, y_off, cw, ch, 1,
                    dest_y, dest_cb, dest_cr, list0 ? 1 : 0);
    }
}

static void mc_part_weighted(H264Decoder *h, H264Slice *sl, int n, int lw, int lh,
                             int cw, int ch, int x_off, int y_off,
                             uint8_t *dest_y, uint8_t *dest_cb, uint8_t *dest_cr,
                             int list0, int list1)
{
    H264Frame *f = h->cur_pic;

    dest_y += x_off + y_off * sl->mb_linesize;
    dest_cb += (x_off >> 1) + (y_off >> 1) * sl->mb_uvlinesize;
    dest_cr += (x_off >> 1) + (y_off >> 1) * sl->mb_uvlinesize;

    if (list0 && list1) {
        uint8_t *tmp_cb = sl->bipred_scratchpad;
        uint8_t *tmp_cr = sl->bipred_scratchpad + 8;
        uint8_t *tmp_y = sl->bipred_scratchpad + 16 * sl->mb_uvlinesize;
        int refn0 = sl->ref_cache[0][h264_scan8[n]];
        int refn1 = sl->ref_cache[1][h264_scan8[n]];

        mc_dir_part(h, sl, &sl->ref_list[0][refn0], n, lw, lh, x_off, y_off, cw, ch, 0,
                    dest_y, dest_cb, dest_cr, 0);
        mc_dir_part(h, sl, &sl->ref_list[1][refn1], n, lw, lh, x_off, y_off, cw, ch, 1,
                    tmp_y, tmp_cb, tmp_cr, 0);

        if (sl->pwt.use_weight == 2) {
            int weight0 = sl->pwt.implicit_weight[refn0][refn1][sl->mb_y & 1];
            int weight1 = 64 - weight0;
            h264_biweight_pixels(dest_y, tmp_y, sl->mb_linesize, lw, lh,
                                 5, weight0, weight1, 0);
            h264_biweight_pixels(dest_cb, tmp_cb, sl->mb_uvlinesize, cw, ch,
                                 5, weight0, weight1, 0);
            h264_biweight_pixels(dest_cr, tmp_cr, sl->mb_uvlinesize, cw, ch,
                                 5, weight0, weight1, 0);
        } else {
            h264_biweight_pixels(dest_y, tmp_y, sl->mb_linesize, lw, lh,
                                 sl->pwt.luma_log2_weight_denom,
                                 sl->pwt.luma_weight[refn0][0][0],
                                 sl->pwt.luma_weight[refn1][1][0],
                                 sl->pwt.luma_weight[refn0][0][1] +
                                 sl->pwt.luma_weight[refn1][1][1]);
            h264_biweight_pixels(dest_cb, tmp_cb, sl->mb_uvlinesize, cw, ch,
                                 sl->pwt.chroma_log2_weight_denom,
                                 sl->pwt.chroma_weight[refn0][0][0][0],
                                 sl->pwt.chroma_weight[refn1][1][0][0],
                                 sl->pwt.chroma_weight[refn0][0][0][1] +
                                 sl->pwt.chroma_weight[refn1][1][0][1]);
            h264_biweight_pixels(dest_cr, tmp_cr, sl->mb_uvlinesize, cw, ch,
                                 sl->pwt.chroma_log2_weight_denom,
                                 sl->pwt.chroma_weight[refn0][0][1][0],
                                 sl->pwt.chroma_weight[refn1][1][1][0],
                                 sl->pwt.chroma_weight[refn0][0][1][1] +
                                 sl->pwt.chroma_weight[refn1][1][1][1]);
        }
    } else {
        int list = list1 ? 1 : 0;
        int refn = sl->ref_cache[list][h264_scan8[n]];
        H264Ref *ref = &sl->ref_list[list][refn];
        mc_dir_part(h, sl, ref, n, lw, lh, x_off, y_off, cw, ch, list,
                    dest_y, dest_cb, dest_cr, 0);

        h264_weight_pixels(dest_y, sl->mb_linesize, lw, lh,
                           sl->pwt.luma_log2_weight_denom,
                           sl->pwt.luma_weight[refn][list][0],
                           sl->pwt.luma_weight[refn][list][1]);
        if (sl->pwt.use_weight_chroma) {
            h264_weight_pixels(dest_cb, sl->mb_uvlinesize, cw, ch,
                               sl->pwt.chroma_log2_weight_denom,
                               sl->pwt.chroma_weight[refn][list][0][0],
                               sl->pwt.chroma_weight[refn][list][0][1]);
            h264_weight_pixels(dest_cr, sl->mb_uvlinesize, cw, ch,
                               sl->pwt.chroma_log2_weight_denom,
                               sl->pwt.chroma_weight[refn][list][1][0],
                               sl->pwt.chroma_weight[refn][list][1][1]);
        }
    }
    (void)f;
}

static void mc_part(H264Decoder *h, H264Slice *sl, int n, int lw, int lh,
                    int cw, int ch, int x_off, int y_off,
                    uint8_t *dest_y, uint8_t *dest_cb, uint8_t *dest_cr,
                    int list0, int list1)
{
    if ((sl->pwt.use_weight == 2 && list0 && list1 &&
         sl->pwt.implicit_weight[sl->ref_cache[0][h264_scan8[n]]]
                                 [sl->ref_cache[1][h264_scan8[n]]][sl->mb_y & 1] != 32) ||
        sl->pwt.use_weight == 1) {
        mc_part_weighted(h, sl, n, lw, lh, cw, ch, x_off, y_off,
                         dest_y, dest_cb, dest_cr, list0, list1);
    } else {
        mc_part_std(h, sl, n, lw, lh, cw, ch, x_off, y_off,
                    dest_y, dest_cb, dest_cr, list0, list1);
    }
}

static void hl_motion(H264Decoder *h, H264Slice *sl, uint8_t *dest_y,
                      uint8_t *dest_cb, uint8_t *dest_cr)
{
    H264Frame *f = h->cur_pic;
    const int mb_xy = sl->mb_xy;
    const uint32_t mb_type = f->mb_type[mb_xy];

    await_references(h, sl, mb_type);

    if (IS_16X16(mb_type)) {
        mc_part(h, sl, 0, 16, 16, 8, 8, 0, 0, dest_y, dest_cb, dest_cr,
                IS_DIR(mb_type, 0, 0), IS_DIR(mb_type, 0, 1));
    } else if (IS_16X8(mb_type)) {
        mc_part(h, sl, 0, 16, 8, 8, 4, 0, 0, dest_y, dest_cb, dest_cr,
                IS_DIR(mb_type, 0, 0), IS_DIR(mb_type, 0, 1));
        mc_part(h, sl, 8, 16, 8, 8, 4, 0, 8, dest_y, dest_cb, dest_cr,
                IS_DIR(mb_type, 1, 0), IS_DIR(mb_type, 1, 1));
    } else if (IS_8X16(mb_type)) {
        mc_part(h, sl, 0, 8, 16, 4, 8, 0, 0, dest_y, dest_cb, dest_cr,
                IS_DIR(mb_type, 0, 0), IS_DIR(mb_type, 0, 1));
        mc_part(h, sl, 4, 8, 16, 4, 8, 8, 0, dest_y, dest_cb, dest_cr,
                IS_DIR(mb_type, 1, 0), IS_DIR(mb_type, 1, 1));
    } else {
        int i;
        for (i = 0; i < 4; i++) {
            const int sub_mb_type = sl->sub_mb_type[i];
            const int n = 4 * i;
            int x8 = (i & 1) << 3;
            int y8 = (i & 2) << 2;
            if (IS_SUB_8X8(sub_mb_type)) {
                mc_part(h, sl, n, 8, 8, 4, 4, x8, y8, dest_y, dest_cb, dest_cr,
                        IS_DIR(sub_mb_type, 0, 0), IS_DIR(sub_mb_type, 0, 1));
            } else if (IS_SUB_8X4(sub_mb_type)) {
                mc_part(h, sl, n, 8, 4, 4, 2, x8, y8, dest_y, dest_cb, dest_cr,
                        IS_DIR(sub_mb_type, 0, 0), IS_DIR(sub_mb_type, 0, 1));
                mc_part(h, sl, n + 2, 8, 4, 4, 2, x8, y8 + 4, dest_y, dest_cb, dest_cr,
                        IS_DIR(sub_mb_type, 0, 0), IS_DIR(sub_mb_type, 0, 1));
            } else if (IS_SUB_4X8(sub_mb_type)) {
                mc_part(h, sl, n, 4, 8, 2, 4, x8, y8, dest_y, dest_cb, dest_cr,
                        IS_DIR(sub_mb_type, 0, 0), IS_DIR(sub_mb_type, 0, 1));
                mc_part(h, sl, n + 1, 4, 8, 2, 4, x8 + 4, y8, dest_y, dest_cb, dest_cr,
                        IS_DIR(sub_mb_type, 0, 0), IS_DIR(sub_mb_type, 0, 1));
            } else {
                int j;
                for (j = 0; j < 4; j++) {
                    int sx = x8 + 4 * (j & 1);
                    int sy = y8 + 4 * (j >> 1);
                    mc_part(h, sl, n + j, 4, 4, 2, 2, sx, sy, dest_y, dest_cb, dest_cr,
                            IS_DIR(sub_mb_type, 0, 0), IS_DIR(sub_mb_type, 0, 1));
                }
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/* 10-bit motion compensation                                          */
/* ------------------------------------------------------------------ */
static void mc_dir_part_16(H264Decoder *h, H264Slice *sl, H264Ref *pic, int n,
                           int w, int hgt, int x_off, int y_off,
                           int cw, int ch, int list,
                           uint8_t *dest_y, uint8_t *dest_cb, uint8_t *dest_cr,
                           int avg)
{
    H264Frame *f = h->cur_pic;
    const int cidc = f->chroma_format_idc;
    const int mx = sl->mv_cache[list][h264_scan8[n]][0] +
                   4 * (x_off + 16 * sl->mb_x);
    int my = sl->mv_cache[list][h264_scan8[n]][1] +
             4 * (y_off + 16 * (sl->mb_y >> MB_FIELD(sl)));
    const int full_mx = mx >> 2;
    const int full_my = my >> 2;
    const int pic_width = 16 * f->mb_width;
    const int pic_height = 16 * f->mb_height >> MB_FIELD(sl);
    const int extra_width = (mx & 7) ? -3 : 0;
    const int extra_height = (my & 7) ? -3 : 0;
    int emu = 0;
    uint8_t *src_y, *src_cb, *src_cr;

    src_y = pic->data[0] + full_mx * 2 + full_my * sl->mb_linesize;

    if (full_mx < 0 - extra_width || full_my < 0 - extra_height ||
        full_mx + 16 > pic_width + extra_width ||
        full_my + 16 > pic_height + extra_height) {
        h264_emulated_edge_mc_16(sl->edge_emu_buffer,
                                 src_y - 4 - 2 * sl->mb_linesize,
                                 sl->mb_linesize, 16 + 5, 16 + 5,
                                 full_mx - 2, full_my - 2, pic_width, pic_height);
        src_y = sl->edge_emu_buffer + 4 + 2 * sl->mb_linesize;
        emu = 1;
    }

    h264_qpel_mc_16(dest_y, src_y, sl->mb_linesize, w, hgt, mx & 3, my & 3, avg);

    if (cidc == 0)
        return;

    if (cidc == 3) {
        const int offset = full_mx * 2 + full_my * sl->mb_uvlinesize;
        src_cb = pic->data[1] + offset;
        src_cr = pic->data[2] + offset;
        if (emu) {
            h264_emulated_edge_mc_16(sl->edge_emu_buffer,
                                     src_cb - 4 - 2 * sl->mb_uvlinesize,
                                     sl->mb_uvlinesize, 16 + 5, 16 + 5,
                                     full_mx - 2, full_my - 2, pic_width, pic_height);
            src_cb = sl->edge_emu_buffer + 4 + 2 * sl->mb_uvlinesize;
        }
        h264_qpel_mc_16(dest_cb, src_cb, sl->mb_uvlinesize, w, hgt,
                        mx & 3, my & 3, avg);
        if (emu) {
            h264_emulated_edge_mc_16(sl->edge_emu_buffer,
                                     src_cr - 4 - 2 * sl->mb_uvlinesize,
                                     sl->mb_uvlinesize, 16 + 5, 16 + 5,
                                     full_mx - 2, full_my - 2, pic_width, pic_height);
            src_cr = sl->edge_emu_buffer + 4 + 2 * sl->mb_uvlinesize;
        }
        h264_qpel_mc_16(dest_cr, src_cr, sl->mb_uvlinesize, w, hgt,
                        mx & 3, my & 3, avg);
        return;
    }

    if (MB_FIELD(sl) && cidc == 1) {
        my += 2 * ((sl->mb_y & 1) - (pic->reference - 1));
        if ((mx >> 3) < 0 || (my >> 3) < 0 || (my >> 3) + ch >= (pic_height >> 1))
            emu = 1;
    }

    {
        const int ysh = 3 - (cidc == 2);
        const int chroma_x = mx >> 3;
        const int chroma_y = my >> ysh;
        const int chroma_w = pic_width >> 1;
        const int chroma_h = pic_height >> (cidc == 1);
        src_cb = pic->data[1] + chroma_x * 2 + chroma_y * sl->mb_uvlinesize;
        src_cr = pic->data[2] + chroma_x * 2 + chroma_y * sl->mb_uvlinesize;

        if (emu) {
            h264_emulated_edge_mc_16(sl->edge_emu_buffer, src_cb,
                                     sl->mb_uvlinesize, 9, 8 * cidc + 1,
                                     chroma_x, chroma_y, chroma_w, chroma_h);
            src_cb = sl->edge_emu_buffer;
        }
        h264_chroma_mc_16(dest_cb, src_cb, sl->mb_uvlinesize, cw, ch,
                          mx & 7, ((unsigned)my << (cidc == 2)) & 7, avg);

        if (emu) {
            h264_emulated_edge_mc_16(sl->edge_emu_buffer, src_cr,
                                     sl->mb_uvlinesize, 9, 8 * cidc + 1,
                                     chroma_x, chroma_y, chroma_w, chroma_h);
            src_cr = sl->edge_emu_buffer;
        }
        h264_chroma_mc_16(dest_cr, src_cr, sl->mb_uvlinesize, cw, ch,
                          mx & 7, ((unsigned)my << (cidc == 2)) & 7, avg);
    }
}

static void mc_part_std_16(H264Decoder *h, H264Slice *sl, int n, int lw, int lh,
                           int cw, int ch, int x_off, int y_off,
                           uint8_t *dest_y, uint8_t *dest_cb, uint8_t *dest_cr,
                           int list0, int list1)
{
    H264Frame *f = h->cur_pic;
    const int cidc = f->chroma_format_idc;
    cw = lw >> f->chroma_shift_x;
    ch = lh >> f->chroma_shift_y;
    dest_y += (x_off << 1) + y_off * sl->mb_linesize;
    if (cidc == 3) {
        dest_cb += (x_off << 1) + y_off * sl->mb_uvlinesize;
        dest_cr += (x_off << 1) + y_off * sl->mb_uvlinesize;
    } else if (cidc == 2) {
        dest_cb += x_off + y_off * sl->mb_uvlinesize;
        dest_cr += x_off + y_off * sl->mb_uvlinesize;
    } else if (cidc == 1) {
        dest_cb += x_off + (y_off >> 1) * sl->mb_uvlinesize;
        dest_cr += x_off + (y_off >> 1) * sl->mb_uvlinesize;
    }

    if (list0) {
        H264Ref *ref = &sl->ref_list[0][sl->ref_cache[0][h264_scan8[n]]];
        mc_dir_part_16(h, sl, ref, n, lw, lh, x_off, y_off, cw, ch, 0,
                       dest_y, dest_cb, dest_cr, 0);
    }
    if (list1) {
        H264Ref *ref = &sl->ref_list[1][sl->ref_cache[1][h264_scan8[n]]];
        mc_dir_part_16(h, sl, ref, n, lw, lh, x_off, y_off, cw, ch, 1,
                       dest_y, dest_cb, dest_cr, list0 ? 1 : 0);
    }
}

static void mc_part_weighted_16(H264Decoder *h, H264Slice *sl, int n, int lw, int lh,
                                int cw, int ch, int x_off, int y_off,
                                uint8_t *dest_y, uint8_t *dest_cb, uint8_t *dest_cr,
                                int list0, int list1)
{
    H264Frame *f = h->cur_pic;
    const int cidc = f->chroma_format_idc;
    cw = lw >> f->chroma_shift_x;
    ch = lh >> f->chroma_shift_y;
    dest_y += (x_off << 1) + y_off * sl->mb_linesize;
    if (cidc == 3) {
        dest_cb += (x_off << 1) + y_off * sl->mb_uvlinesize;
        dest_cr += (x_off << 1) + y_off * sl->mb_uvlinesize;
    } else if (cidc == 2) {
        dest_cb += x_off + y_off * sl->mb_uvlinesize;
        dest_cr += x_off + y_off * sl->mb_uvlinesize;
    } else if (cidc == 1) {
        dest_cb += x_off + (y_off >> 1) * sl->mb_uvlinesize;
        dest_cr += x_off + (y_off >> 1) * sl->mb_uvlinesize;
    }

    if (list0 && list1) {
        uint8_t *tmp_cb = sl->bipred_scratchpad;
        uint8_t *tmp_cr = sl->bipred_scratchpad + 16 * sl->mb_uvlinesize;
        uint8_t *tmp_y = sl->bipred_scratchpad + 32 * sl->mb_uvlinesize;
        int refn0 = sl->ref_cache[0][h264_scan8[n]];
        int refn1 = sl->ref_cache[1][h264_scan8[n]];

        mc_dir_part_16(h, sl, &sl->ref_list[0][refn0], n, lw, lh, x_off, y_off,
                       cw, ch, 0, dest_y, dest_cb, dest_cr, 0);
        mc_dir_part_16(h, sl, &sl->ref_list[1][refn1], n, lw, lh, x_off, y_off,
                       cw, ch, 1, tmp_y, tmp_cb, tmp_cr, 0);

        if (sl->pwt.use_weight == 2) {
            int weight0 = sl->pwt.implicit_weight[refn0][refn1][sl->mb_y & 1];
            int weight1 = 64 - weight0;
            h264_biweight_pixels_16(dest_y, tmp_y, sl->mb_linesize, lw, lh,
                                    5, weight0, weight1, 0);
            if (cidc) {
                h264_biweight_pixels_16(dest_cb, tmp_cb, sl->mb_uvlinesize, cw, ch,
                                        5, weight0, weight1, 0);
                h264_biweight_pixels_16(dest_cr, tmp_cr, sl->mb_uvlinesize, cw, ch,
                                        5, weight0, weight1, 0);
            }
        } else {
            h264_biweight_pixels_16(dest_y, tmp_y, sl->mb_linesize, lw, lh,
                                    sl->pwt.luma_log2_weight_denom,
                                    sl->pwt.luma_weight[refn0][0][0],
                                    sl->pwt.luma_weight[refn1][1][0],
                                    sl->pwt.luma_weight[refn0][0][1] +
                                    sl->pwt.luma_weight[refn1][1][1]);
            if (cidc) {
                h264_biweight_pixels_16(dest_cb, tmp_cb, sl->mb_uvlinesize, cw, ch,
                                        sl->pwt.chroma_log2_weight_denom,
                                        sl->pwt.chroma_weight[refn0][0][0][0],
                                        sl->pwt.chroma_weight[refn1][1][0][0],
                                        sl->pwt.chroma_weight[refn0][0][0][1] +
                                        sl->pwt.chroma_weight[refn1][1][0][1]);
                h264_biweight_pixels_16(dest_cr, tmp_cr, sl->mb_uvlinesize, cw, ch,
                                        sl->pwt.chroma_log2_weight_denom,
                                        sl->pwt.chroma_weight[refn0][0][1][0],
                                        sl->pwt.chroma_weight[refn1][1][1][0],
                                        sl->pwt.chroma_weight[refn0][0][1][1] +
                                        sl->pwt.chroma_weight[refn1][1][1][1]);
            }
        }
    } else {
        int list = list1 ? 1 : 0;
        int refn = sl->ref_cache[list][h264_scan8[n]];
        H264Ref *ref = &sl->ref_list[list][refn];
        mc_dir_part_16(h, sl, ref, n, lw, lh, x_off, y_off, cw, ch, list,
                       dest_y, dest_cb, dest_cr, 0);

        h264_weight_pixels_16(dest_y, sl->mb_linesize, lw, lh,
                              sl->pwt.luma_log2_weight_denom,
                              sl->pwt.luma_weight[refn][list][0],
                              sl->pwt.luma_weight[refn][list][1]);
        if (cidc && sl->pwt.use_weight_chroma) {
            h264_weight_pixels_16(dest_cb, sl->mb_uvlinesize, cw, ch,
                                  sl->pwt.chroma_log2_weight_denom,
                                  sl->pwt.chroma_weight[refn][list][0][0],
                                  sl->pwt.chroma_weight[refn][list][0][1]);
            h264_weight_pixels_16(dest_cr, sl->mb_uvlinesize, cw, ch,
                                  sl->pwt.chroma_log2_weight_denom,
                                  sl->pwt.chroma_weight[refn][list][1][0],
                                  sl->pwt.chroma_weight[refn][list][1][1]);
        }
    }
}

static void mc_part_16(H264Decoder *h, H264Slice *sl, int n, int lw, int lh,
                       int cw, int ch, int x_off, int y_off,
                       uint8_t *dest_y, uint8_t *dest_cb, uint8_t *dest_cr,
                       int list0, int list1)
{
    if ((sl->pwt.use_weight == 2 && list0 && list1 &&
         sl->pwt.implicit_weight[sl->ref_cache[0][h264_scan8[n]]]
                                 [sl->ref_cache[1][h264_scan8[n]]][sl->mb_y & 1] != 32) ||
        sl->pwt.use_weight == 1) {
        mc_part_weighted_16(h, sl, n, lw, lh, cw, ch, x_off, y_off,
                            dest_y, dest_cb, dest_cr, list0, list1);
    } else {
        mc_part_std_16(h, sl, n, lw, lh, cw, ch, x_off, y_off,
                       dest_y, dest_cb, dest_cr, list0, list1);
    }
}

static void hl_motion_16(H264Decoder *h, H264Slice *sl, uint8_t *dest_y,
                         uint8_t *dest_cb, uint8_t *dest_cr)
{
    H264Frame *f = h->cur_pic;
    const int mb_xy = sl->mb_xy;
    const uint32_t mb_type = f->mb_type[mb_xy];

    await_references(h, sl, mb_type);

    if (IS_16X16(mb_type)) {
        mc_part_16(h, sl, 0, 16, 16, 8, 8, 0, 0, dest_y, dest_cb, dest_cr,
                   IS_DIR(mb_type, 0, 0), IS_DIR(mb_type, 0, 1));
    } else if (IS_16X8(mb_type)) {
        mc_part_16(h, sl, 0, 16, 8, 8, 4, 0, 0, dest_y, dest_cb, dest_cr,
                   IS_DIR(mb_type, 0, 0), IS_DIR(mb_type, 0, 1));
        mc_part_16(h, sl, 8, 16, 8, 8, 4, 0, 8, dest_y, dest_cb, dest_cr,
                   IS_DIR(mb_type, 1, 0), IS_DIR(mb_type, 1, 1));
    } else if (IS_8X16(mb_type)) {
        mc_part_16(h, sl, 0, 8, 16, 4, 8, 0, 0, dest_y, dest_cb, dest_cr,
                   IS_DIR(mb_type, 0, 0), IS_DIR(mb_type, 0, 1));
        mc_part_16(h, sl, 4, 8, 16, 4, 8, 8, 0, dest_y, dest_cb, dest_cr,
                   IS_DIR(mb_type, 1, 0), IS_DIR(mb_type, 1, 1));
    } else {
        int i;
        for (i = 0; i < 4; i++) {
            const int sub_mb_type = sl->sub_mb_type[i];
            const int n = 4 * i;
            int x8 = (i & 1) << 3;
            int y8 = (i & 2) << 2;
            if (IS_SUB_8X8(sub_mb_type)) {
                mc_part_16(h, sl, n, 8, 8, 4, 4, x8, y8, dest_y, dest_cb, dest_cr,
                           IS_DIR(sub_mb_type, 0, 0), IS_DIR(sub_mb_type, 0, 1));
            } else if (IS_SUB_8X4(sub_mb_type)) {
                mc_part_16(h, sl, n, 8, 4, 4, 2, x8, y8, dest_y, dest_cb, dest_cr,
                           IS_DIR(sub_mb_type, 0, 0), IS_DIR(sub_mb_type, 0, 1));
                mc_part_16(h, sl, n + 2, 8, 4, 4, 2, x8, y8 + 4, dest_y, dest_cb, dest_cr,
                           IS_DIR(sub_mb_type, 0, 0), IS_DIR(sub_mb_type, 0, 1));
            } else if (IS_SUB_4X8(sub_mb_type)) {
                mc_part_16(h, sl, n, 4, 8, 2, 4, x8, y8, dest_y, dest_cb, dest_cr,
                           IS_DIR(sub_mb_type, 0, 0), IS_DIR(sub_mb_type, 0, 1));
                mc_part_16(h, sl, n + 1, 4, 8, 2, 4, x8 + 4, y8, dest_y, dest_cb, dest_cr,
                           IS_DIR(sub_mb_type, 0, 0), IS_DIR(sub_mb_type, 0, 1));
            } else {
                int j;
                for (j = 0; j < 4; j++) {
                    int sx = x8 + 4 * (j & 1);
                    int sy = y8 + 4 * (j >> 1);
                    mc_part_16(h, sl, n + j, 4, 4, 2, 2, sx, sy, dest_y, dest_cb, dest_cr,
                               IS_DIR(sub_mb_type, 0, 0), IS_DIR(sub_mb_type, 0, 1));
                }
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/* intra / idct                                                        */
/* ------------------------------------------------------------------ */
static void hl_decode_mb_predict_luma(H264Decoder *h, H264Slice *sl, uint32_t mb_type,
                                      const int *block_offset, int linesize,
                                      uint8_t *dest_y, int transform_bypass)
{
    void (*idct_dc_add)(uint8_t *dst, int16_t *block, int stride);
    void (*idct_add)(uint8_t *dst, int16_t *block, int stride);
    int i;
    int qscale = sl->qscale;

    if (IS_INTRA4x4(mb_type)) {
        if (IS_8x8DCT(mb_type)) {
            if (transform_bypass) {
                idct_dc_add = idct_add = h264_add_pixels8;
            } else {
                idct_dc_add = h264_idct8_dc_add;
                idct_add    = h264_idct8_add;
            }
            for (i = 0; i < 16; i += 4) {
                uint8_t *const ptr = dest_y + block_offset[i];
                const int dir = sl->intra4x4_pred_mode_cache[h264_scan8[i]];
                const int tl =
                    (int)((sl->topleft_samples_available << i) & 0x8000) != 0;
                const int tr =
                    (int)((sl->topright_samples_available << i) & 0x4000) != 0;
                const int nnz = sl->non_zero_count_cache[h264_scan8[i]];
                if (transform_bypass && h->active_sps->profile_idc == 244 &&
                    dir <= 1) {
                    if ((unsigned)h->x264_build < 151U)
                        h264_pred8x8l_add(dir, ptr, sl->mb + i * 16, linesize);
                    else
                        h264_pred8x8l_filter_add(dir, ptr, sl->mb + i * 16,
                                                 tl, tr, linesize);
                    continue;
                }
                h264_pred8x8l_dispatch(dir, ptr, tl, tr, linesize);
                if (nnz) {
                    if (nnz == 1 && sl->mb[i * 16])
                        idct_dc_add(ptr, sl->mb + i * 16, linesize);
                    else
                        idct_add(ptr, sl->mb + i * 16, linesize);
                }
            }
        } else {
            if (transform_bypass) {
                idct_dc_add = idct_add = h264_add_pixels4;
            } else {
                idct_dc_add = h264_idct_dc_add;
                idct_add    = h264_idct_add;
            }
            for (i = 0; i < 16; i++) {
                uint8_t *const ptr = dest_y + block_offset[i];
                const int dir = sl->intra4x4_pred_mode_cache[h264_scan8[i]];
                uint8_t *topright = NULL;
                if (transform_bypass && h->active_sps->profile_idc == 244 &&
                    dir <= 1) {
                    h264_pred4x4_add(dir, ptr, sl->mb + i * 16, linesize);
                    continue;
                }
                if (dir == 3 /* DIAG_DOWN_LEFT */ || dir == 7 /* VERT_LEFT */) {
                    const int topright_avail = (sl->topright_samples_available << i) & 0x8000;
                    if (!topright_avail) {
                        int tr = ptr[3 - linesize] * 0x01010101u;
                        uint8_t trbuf[4];
                        memcpy(trbuf, &tr, 4);
                        topright = trbuf;
                        h264_pred4x4_dispatch(dir, ptr, topright, linesize);
                    } else {
                        topright = ptr + 4 - linesize;
                        h264_pred4x4_dispatch(dir, ptr, topright, linesize);
                    }
                } else {
                    h264_pred4x4_dispatch(dir, ptr, topright, linesize);
                }
                if (sl->non_zero_count_cache[h264_scan8[i]]) {
                    const int nnz = sl->non_zero_count_cache[h264_scan8[i]];
                    if (nnz == 1 && sl->mb[i * 16])
                        idct_dc_add(ptr, sl->mb + i * 16, linesize);
                    else
                        idct_add(ptr, sl->mb + i * 16, linesize);
                }
            }
        }
    } else {
        h264_pred16x16_dispatch(sl->intra16x16_pred_mode, dest_y, linesize);
        if (sl->non_zero_count_cache[h264_scan8[48]]) {
            if (!transform_bypass) {
                h264_luma_dc_dequant_idct(sl->mb, sl->mb_luma_dc[0],
                                          h->active_pps->dequant4_coeff[0][qscale][0]);
            } else {
                /* the DC block of an I16x16 macroblock keeps its coefficients in
                 * mb_luma_dc, in raster order, and has to be spread over the
                 * sixteen 4x4 blocks that make up the macroblock */
                static const uint8_t dc_mapping[16] = {
                    0 * 16,  1 * 16,  4 * 16,  5 * 16,
                    2 * 16,  3 * 16,  6 * 16,  7 * 16,
                    8 * 16,  9 * 16, 12 * 16, 13 * 16,
                    10 * 16, 11 * 16, 14 * 16, 15 * 16
                };
                for (i = 0; i < 16; i++)
                    sl->mb[dc_mapping[i]] = sl->mb_luma_dc[0][i];
            }
        }
    }
}

static void hl_decode_mb_idct_luma(H264Decoder *h, H264Slice *sl, uint32_t mb_type,
                                   const int *block_offset, int linesize,
                                   uint8_t *dest_y, int transform_bypass)
{
    int i;

    if (!IS_INTRA4x4(mb_type)) {
        if (IS_INTRA16x16(mb_type)) {
            if (transform_bypass) {
                if (h->active_sps->profile_idc == 244 &&
                    (sl->intra16x16_pred_mode == 1 ||
                     sl->intra16x16_pred_mode == 2)) {
                    h264_pred16x16_add(sl->intra16x16_pred_mode, dest_y,
                                       block_offset, sl->mb, linesize);
                } else {
                    for (i = 0; i < 16; i++)
                        if (sl->non_zero_count_cache[h264_scan8[i]] || sl->mb[i * 16])
                            h264_add_pixels4(dest_y + block_offset[i],
                                             sl->mb + i * 16, linesize);
                }
            } else {
                h264_idct_add16intra(dest_y, block_offset, sl->mb, linesize,
                                     sl->non_zero_count_cache);
            }
        } else if (sl->cbp & 15) {
            if (transform_bypass) {
                const int di = IS_8x8DCT(mb_type) ? 4 : 1;
                void (*add_pixels)(uint8_t *, int16_t *, int) =
                    IS_8x8DCT(mb_type) ? h264_add_pixels8 : h264_add_pixels4;
                for (i = 0; i < 16; i += di)
                    if (sl->non_zero_count_cache[h264_scan8[i]])
                        add_pixels(dest_y + block_offset[i],
                                   sl->mb + i * 16, linesize);
            } else if (IS_8x8DCT(mb_type)) {
                h264_idct8_add4(dest_y, block_offset, sl->mb, linesize,
                                sl->non_zero_count_cache);
            } else {
                h264_idct_add16(dest_y, block_offset, sl->mb, linesize,
                                sl->non_zero_count_cache);
            }
        }
    }
}

static void hl_decode_mb_predict_luma_16(H264Decoder *h, H264Slice *sl,
                                         uint32_t mb_type,
                                         const int *block_offset, int stride,
                                         uint8_t *dest_y, int transform_bypass,
                                         int p)
{
    uint16_t *d = (uint16_t *)dest_y;
    int32_t *mb = (int32_t *)sl->mb + p * 256;
    void (*idct_dc_add)(uint16_t *, int32_t *, int);
    void (*idct_add)(uint16_t *, int32_t *, int);
    int i;
    int qscale = p == 0 ? sl->qscale : sl->chroma_qp[p - 1];
    const int nnz_off = 16 * p;

    if (IS_INTRA4x4(mb_type)) {
        if (IS_8x8DCT(mb_type)) {
            if (transform_bypass) {
                idct_dc_add = idct_add = h264_add_pixels8_16;
            } else {
                idct_dc_add = h264_idct8_dc_add_16;
                idct_add    = h264_idct8_add_16;
            }
            for (i = 0; i < 16; i += 4) {
                uint16_t *const ptr = d + block_offset[i];
                const int dir = sl->intra4x4_pred_mode_cache[h264_scan8[i]];
                const int tl =
                    (int)((sl->topleft_samples_available << i) & 0x8000) != 0;
                const int tr =
                    (int)((sl->topright_samples_available << i) & 0x4000) != 0;
                const int nnz = sl->non_zero_count_cache[h264_scan8[i + nnz_off]];
                if (transform_bypass && h->active_sps->profile_idc == 244 &&
                    dir <= 1) {
                    if ((unsigned)h->x264_build < 151U)
                        h264_pred8x8l_add_16(dir, ptr, mb + i * 16, stride);
                    else
                        h264_pred8x8l_filter_add_16(dir, ptr, mb + i * 16,
                                                    tl, tr, stride);
                    continue;
                }
                h264_pred8x8l_dispatch_16(dir, ptr, tl, tr, stride);
                if (nnz) {
                    if (nnz == 1 && mb[i * 16])
                        idct_dc_add(ptr, mb + i * 16, stride);
                    else
                        idct_add(ptr, mb + i * 16, stride);
                }
            }
        } else {
            if (transform_bypass) {
                idct_dc_add = idct_add = h264_add_pixels4_16;
            } else {
                idct_dc_add = h264_idct_dc_add_16;
                idct_add    = h264_idct_add_16;
            }
            for (i = 0; i < 16; i++) {
                uint16_t *const ptr = d + block_offset[i];
                const int dir = sl->intra4x4_pred_mode_cache[h264_scan8[i]];
                uint16_t trbuf[4];
                uint16_t *topright = NULL;
                if (transform_bypass && h->active_sps->profile_idc == 244 &&
                    dir <= 1) {
                    h264_pred4x4_add_16(dir, ptr, mb + i * 16, stride);
                    continue;
                }
                if (dir == 3 || dir == 7) {
                    const int topright_avail =
                        (sl->topright_samples_available << i) & 0x8000;
                    if (!topright_avail) {
                        const uint16_t v = ptr[3 - stride];
                        trbuf[0] = trbuf[1] = trbuf[2] = trbuf[3] = v;
                        topright = trbuf;
                        h264_pred4x4_dispatch_16(dir, ptr, topright, stride);
                    } else {
                        topright = ptr + 4 - stride;
                        h264_pred4x4_dispatch_16(dir, ptr, topright, stride);
                    }
                } else {
                    h264_pred4x4_dispatch_16(dir, ptr, topright, stride);
                }
                if (sl->non_zero_count_cache[h264_scan8[i + nnz_off]]) {
                    const int nnz = sl->non_zero_count_cache[h264_scan8[i + nnz_off]];
                    if (nnz == 1 && mb[i * 16])
                        idct_dc_add(ptr, mb + i * 16, stride);
                    else
                        idct_add(ptr, mb + i * 16, stride);
                }
            }
        }
    } else {
        h264_pred16x16_dispatch_16(sl->intra16x16_pred_mode, d, stride);
        if (sl->non_zero_count_cache[h264_scan8[48 + p]]) {
            if (!transform_bypass) {
                h264_luma_dc_dequant_idct_16(mb, (int32_t *)sl->mb_luma_dc[p],
                                             h->active_pps->dequant4_coeff[p][qscale][0]);
            } else {
                static const uint8_t dc_mapping[16] = {
                    0 * 16,  1 * 16,  4 * 16,  5 * 16,
                    2 * 16,  3 * 16,  6 * 16,  7 * 16,
                    8 * 16,  9 * 16, 12 * 16, 13 * 16,
                    10 * 16, 11 * 16, 14 * 16, 15 * 16
                };
                const int32_t *dc = (const int32_t *)sl->mb_luma_dc[p];
                for (i = 0; i < 16; i++)
                    mb[dc_mapping[i]] = dc[i];
            }
        }
    }
}

static void hl_decode_mb_idct_luma_16(H264Decoder *h, H264Slice *sl,
                                      uint32_t mb_type,
                                      const int *block_offset, int stride,
                                      uint8_t *dest_y, int transform_bypass,
                                      int p)
{
    uint16_t *d = (uint16_t *)dest_y;
    int32_t *mb = (int32_t *)sl->mb + p * 256;
    const uint8_t *nnzc = sl->non_zero_count_cache + p * 5 * 8;
    const int nnz_off = 16 * p;
    int i;

    if (!IS_INTRA4x4(mb_type)) {
        if (IS_INTRA16x16(mb_type)) {
            if (transform_bypass) {
                if (h->active_sps->profile_idc == 244 &&
                    (sl->intra16x16_pred_mode == 1 ||
                     sl->intra16x16_pred_mode == 2)) {
                    h264_pred16x16_add_16(sl->intra16x16_pred_mode, d,
                                          block_offset, mb, stride);
                } else {
                    for (i = 0; i < 16; i++)
                        if (sl->non_zero_count_cache[h264_scan8[i + nnz_off]] ||
                            mb[i * 16])
                            h264_add_pixels4_16(d + block_offset[i],
                                                mb + i * 16, stride);
                }
            } else {
                h264_idct_add16intra_16(d, block_offset, mb, stride, nnzc);
            }
        } else if (sl->cbp & 15) {
            if (transform_bypass) {
                const int di = IS_8x8DCT(mb_type) ? 4 : 1;
                void (*add_pixels)(uint16_t *, int32_t *, int) =
                    IS_8x8DCT(mb_type) ? h264_add_pixels8_16 : h264_add_pixels4_16;
                for (i = 0; i < 16; i += di)
                    if (sl->non_zero_count_cache[h264_scan8[i + nnz_off]])
                        add_pixels(d + block_offset[i], mb + i * 16, stride);
            } else if (IS_8x8DCT(mb_type)) {
                h264_idct8_add4_16(d, block_offset, mb, stride, nnzc);
            } else {
                h264_idct_add16_16(d, block_offset, mb, stride, nnzc);
            }
        }
    }
}


static void decode_mb_pcm_16(H264Decoder *h, H264Slice *sl, uint8_t *dest_y,
                             uint8_t *dest_cb, uint8_t *dest_cr,
                             int linesize, int uvlinesize)
{
    H264Frame *f = h->cur_pic;
    const int depth = h->active_sps->bit_depth_luma;
    const int cidc = f->chroma_format_idc;
    const int cw = 16 >> f->chroma_shift_x;
    const int chh = 16 >> f->chroma_shift_y;
    const int mb_samples = cidc == 3 ? 768 : cidc == 2 ? 512 : cidc ? 384 : 256;
    H264BitReader br;
    int i, j;

    h264_br_init(&br, sl->intra_pcm_ptr, mb_samples * depth);
    for (i = 0; i < 16; i++) {
        uint16_t *row = (uint16_t *)(dest_y + i * linesize);
        for (j = 0; j < 16; j++)
            row[j] = (uint16_t)h264_br_get_bits(&br, depth);
    }
    if (cidc) {
        for (i = 0; i < chh; i++) {
            uint16_t *row = (uint16_t *)(dest_cb + i * uvlinesize);
            for (j = 0; j < cw; j++)
                row[j] = (uint16_t)h264_br_get_bits(&br, depth);
        }
        for (i = 0; i < chh; i++) {
            uint16_t *row = (uint16_t *)(dest_cr + i * uvlinesize);
            for (j = 0; j < cw; j++)
                row[j] = (uint16_t)h264_br_get_bits(&br, depth);
        }
    }
}

static void decode_mb_pcm(uint8_t *dest_y, uint8_t *dest_cb, uint8_t *dest_cr,
                          int linesize, int uvlinesize, const uint8_t *pcm)
{
    int i;

    for (i = 0; i < 16; i++)
        memcpy(dest_y + i * linesize, pcm + i * 16, 16);
    for (i = 0; i < 8; i++)
        memcpy(dest_cb + i * uvlinesize, pcm + 256 + i * 8, 8);
    for (i = 0; i < 8; i++)
        memcpy(dest_cr + i * uvlinesize, pcm + 256 + 64 + i * 8, 8);
}

void h264_hl_decode_mb(H264Decoder *h, H264Slice *sl)
{
    if (h->cur_pic->bit_depth > 8) {
        h264_hl_decode_mb_16(h, sl);
        return;
    }
    H264Frame *f = h->cur_pic;
    const int mb_x = sl->mb_x;
    const int mb_y = sl->mb_y;
    const int mb_xy = sl->mb_xy;
    const uint32_t mb_type = f->mb_type[mb_xy];
    const int transform_bypass = sl->qscale == 0 && h->active_sps->transform_bypass;
    uint8_t *dest_y, *dest_cb, *dest_cr;
    int linesize, uvlinesize;
    const int *block_offset = f->block_offset[MB_FIELD(sl) ? 1 : 0];
    int i, j;

    f->list_counts[mb_xy] = (uint8_t)sl->list_count;
    /* the deblocking pass runs after the whole picture, so the slice state
     * that each MB was decoded with has to be snapshotted here. */
    f->mb_deblock_idc[mb_xy] = (int8_t)sl->deblocking_filter;
    f->mb_alpha[mb_xy] = (int8_t)sl->slice_alpha_c0_offset;
    f->mb_beta[mb_xy] = (int8_t)sl->slice_beta_offset;

    if (MB_FIELD(sl)) {
        linesize = sl->mb_linesize = f->linesize[0] * 2;
        uvlinesize = sl->mb_uvlinesize = f->linesize[1] * 2;
    } else {
        linesize = sl->mb_linesize = f->linesize[0];
        uvlinesize = sl->mb_uvlinesize = f->linesize[1];
    }

    dest_y = f->data[0] + mb_x * 16 + mb_y * f->linesize[0] * 16;
    dest_cb = f->data[1] + mb_x * 8 + mb_y * f->linesize[1] * 8;
    dest_cr = f->data[2] + mb_x * 8 + mb_y * f->linesize[2] * 8;
    if (MB_FIELD(sl) && (mb_y & 1)) {
        dest_y -= f->linesize[0] * 15;
        dest_cb -= f->linesize[1] * 7;
        dest_cr -= f->linesize[2] * 7;
    }

    /* only field macroblocks address the parity-doubled reference entries */
    if (MB_FIELD(sl) && FRAME_MBAFF(f)) {
        int list;
        for (list = 0; list < sl->list_count; list++) {
            if (!USES_LIST(mb_type, list))
                continue;
            if (IS_16X16(mb_type)) {
                int8_t *ref = &sl->ref_cache[list][h264_scan8[0]];
                int v = (16 + *ref) ^ (mb_y & 1);
                for (i = 0; i < 4; i++)
                    memset(&sl->ref_cache[list][h264_scan8[0] + 8 * i], v, 4);
            } else {
                for (i = 0; i < 16; i += 4) {
                    int ref = sl->ref_cache[list][h264_scan8[i]];
                    int v;
                    if (ref < 0)
                        continue;
                    v = (16 + ref) ^ (mb_y & 1);
                    memset(&sl->ref_cache[list][h264_scan8[i]], v, 2);
                    memset(&sl->ref_cache[list][h264_scan8[i] + 8], v, 2);
                }
            }
        }
    }

    if (IS_INTRA_PCM(mb_type)) {
        decode_mb_pcm(dest_y, dest_cb, dest_cr, linesize, uvlinesize, sl->intra_pcm_ptr);
        return;
    }

    if (IS_INTRA(mb_type)) {
        h264_pred8x8_dispatch(sl->chroma_pred_mode, dest_cb, uvlinesize);
        h264_pred8x8_dispatch(sl->chroma_pred_mode, dest_cr, uvlinesize);
        hl_decode_mb_predict_luma(h, sl, mb_type, block_offset, linesize, dest_y,
                                  transform_bypass);
    } else {
        hl_motion(h, sl, dest_y, dest_cb, dest_cr);
    }

    hl_decode_mb_idct_luma(h, sl, mb_type, block_offset, linesize, dest_y,
                           transform_bypass);

    if (sl->cbp & 0x30) {
        uint8_t *dest[2] = { dest_cb, dest_cr };
        int qp[2];
        qp[0] = sl->chroma_qp[0];
        qp[1] = sl->chroma_qp[1];
        if (transform_bypass) {
            if (IS_INTRA(mb_type) && h->active_sps->profile_idc == 244 &&
                (sl->chroma_pred_mode == 1 || sl->chroma_pred_mode == 2)) {
                h264_pred8x8_add(sl->chroma_pred_mode, dest_cb, block_offset + 16,
                                 sl->mb + 16 * 16, uvlinesize);
                h264_pred8x8_add(sl->chroma_pred_mode, dest_cr, block_offset + 32,
                                 sl->mb + 16 * 16 * 2, uvlinesize);
            } else {
                for (j = 1; j < 3; j++)
                    for (i = j * 16; i < j * 16 + 4; i++)
                        if (sl->non_zero_count_cache[h264_scan8[i]] || sl->mb[i * 16])
                            h264_add_pixels4(dest[j - 1] + block_offset[i],
                                             sl->mb + i * 16, uvlinesize);
            }
        } else {
            if (sl->non_zero_count_cache[h264_scan8[49]])
                h264_chroma_dc_dequant_idct(sl->mb + 16 * 16,
                                            h->active_pps->dequant4_coeff[IS_INTRA(mb_type) ? 1 : 4][qp[0]][0]);
            if (sl->non_zero_count_cache[h264_scan8[50]])
                h264_chroma_dc_dequant_idct(sl->mb + 16 * 16 * 2,
                                            h->active_pps->dequant4_coeff[IS_INTRA(mb_type) ? 2 : 5][qp[1]][0]);
            h264_idct_add8(dest, block_offset, sl->mb, uvlinesize, sl->non_zero_count_cache);
        }
    }
}

static void h264_hl_decode_mb_16(H264Decoder *h, H264Slice *sl)
{
    H264Frame *f = h->cur_pic;
    const int mb_x = sl->mb_x;
    const int mb_y = sl->mb_y;
    const int mb_xy = sl->mb_xy;
    const uint32_t mb_type = f->mb_type[mb_xy];
    const int transform_bypass = sl->qscale == 0 && h->active_sps->transform_bypass;
    const int cidc = f->chroma_format_idc;
    const int c444 = cidc == 3;
    uint8_t *dest_y, *dest_cb, *dest_cr;
    int linesize, uvlinesize, stride, uvstride;
    int block_offset[48];
    int i, j;

    f->list_counts[mb_xy] = (uint8_t)sl->list_count;
    f->mb_deblock_idc[mb_xy] = (int8_t)sl->deblocking_filter;
    f->mb_alpha[mb_xy] = (int8_t)sl->slice_alpha_c0_offset;
    f->mb_beta[mb_xy] = (int8_t)sl->slice_beta_offset;

    if (MB_FIELD(sl)) {
        linesize = sl->mb_linesize = f->linesize[0] * 2;
        uvlinesize = sl->mb_uvlinesize = f->linesize[1] * 2;
    } else {
        linesize = sl->mb_linesize = f->linesize[0];
        uvlinesize = sl->mb_uvlinesize = f->linesize[1];
    }
    stride = linesize >> 1;
    uvstride = uvlinesize >> 1;

    dest_y = f->data[0] + (mb_x * 16 << 1) + mb_y * f->linesize[0] * 16;
    if (c444) {
        dest_cb = f->data[1] + (mb_x * 16 << 1) + mb_y * f->linesize[1] * 16;
        dest_cr = f->data[2] + (mb_x * 16 << 1) + mb_y * f->linesize[2] * 16;
    } else if (cidc == 2) {
        dest_cb = f->data[1] + (mb_x * 8 << 1) + mb_y * f->linesize[1] * 16;
        dest_cr = f->data[2] + (mb_x * 8 << 1) + mb_y * f->linesize[2] * 16;
    } else if (cidc == 1) {
        dest_cb = f->data[1] + (mb_x * 8 << 1) + mb_y * f->linesize[1] * 8;
        dest_cr = f->data[2] + (mb_x * 8 << 1) + mb_y * f->linesize[2] * 8;
    } else {
        dest_cb = dest_cr = NULL;
    }
    if (MB_FIELD(sl) && (mb_y & 1)) {
        const int ch = cidc == 1 ? 7 : 15;
        dest_y -= f->linesize[0] * 15;
        if (cidc) {
            dest_cb -= f->linesize[1] * ch;
            dest_cr -= f->linesize[2] * ch;
        }
    }

    if (MB_FIELD(sl) && FRAME_MBAFF(f)) {
        int list;
        for (list = 0; list < sl->list_count; list++) {
            if (!USES_LIST(mb_type, list))
                continue;
            if (IS_16X16(mb_type)) {
                int8_t *ref = &sl->ref_cache[list][h264_scan8[0]];
                int v = (16 + *ref) ^ (mb_y & 1);
                for (i = 0; i < 4; i++)
                    memset(&sl->ref_cache[list][h264_scan8[0] + 8 * i], v, 4);
            } else {
                for (i = 0; i < 16; i += 4) {
                    int ref = sl->ref_cache[list][h264_scan8[i]];
                    int v;
                    if (ref < 0)
                        continue;
                    v = (16 + ref) ^ (mb_y & 1);
                    memset(&sl->ref_cache[list][h264_scan8[i]], v, 2);
                    memset(&sl->ref_cache[list][h264_scan8[i] + 8], v, 2);
                }
            }
        }
    }

    for (i = 0; i < 16; i++) {
        const int o = (4 * ((h264_scan8[i] - h264_scan8[0]) & 7)) +
                      4 * stride * ((h264_scan8[i] - h264_scan8[0]) >> 3);
        block_offset[i] = o;
        if (c444) {
            block_offset[16 + i] = block_offset[32 + i] = o;
        } else {
            block_offset[16 + i] = block_offset[32 + i] =
                (4 * ((h264_scan8[i] - h264_scan8[0]) & 7)) +
                4 * uvstride * ((h264_scan8[i] - h264_scan8[0]) >> 3);
        }
    }

    if (IS_INTRA_PCM(mb_type)) {
        decode_mb_pcm_16(h, sl, dest_y, dest_cb, dest_cr, linesize, uvlinesize);
        return;
    }

    if (IS_INTRA(mb_type)) {
        if (c444) {
            hl_decode_mb_predict_luma_16(h, sl, mb_type, block_offset, uvstride,
                                         dest_cb, transform_bypass, 1);
            hl_decode_mb_predict_luma_16(h, sl, mb_type, block_offset, uvstride,
                                         dest_cr, transform_bypass, 2);
            hl_decode_mb_predict_luma_16(h, sl, mb_type, block_offset, stride,
                                         dest_y, transform_bypass, 0);
        } else {
            if (cidc == 2) {
                h264_pred8x16_dispatch_16(sl->chroma_pred_mode, (uint16_t *)dest_cb, uvstride);
                h264_pred8x16_dispatch_16(sl->chroma_pred_mode, (uint16_t *)dest_cr, uvstride);
            } else if (cidc == 1) {
                h264_pred8x8_dispatch_16(sl->chroma_pred_mode, (uint16_t *)dest_cb, uvstride);
                h264_pred8x8_dispatch_16(sl->chroma_pred_mode, (uint16_t *)dest_cr, uvstride);
            }
            hl_decode_mb_predict_luma_16(h, sl, mb_type, block_offset, stride, dest_y,
                                         transform_bypass, 0);
        }
    } else {
        hl_motion_16(h, sl, dest_y, dest_cb, dest_cr);
    }

    if (c444) {
        hl_decode_mb_idct_luma_16(h, sl, mb_type, block_offset, uvstride, dest_cb,
                                  transform_bypass, 1);
        hl_decode_mb_idct_luma_16(h, sl, mb_type, block_offset, uvstride, dest_cr,
                                  transform_bypass, 2);
        hl_decode_mb_idct_luma_16(h, sl, mb_type, block_offset, stride, dest_y,
                                  transform_bypass, 0);
    } else {
        hl_decode_mb_idct_luma_16(h, sl, mb_type, block_offset, stride, dest_y,
                                  transform_bypass, 0);
    }

    if (cidc && !c444 && (sl->cbp & 0x30)) {
        uint16_t *dest[2] = { (uint16_t *)dest_cb, (uint16_t *)dest_cr };
        int qp[2];
        qp[0] = sl->chroma_qp[0];
        qp[1] = sl->chroma_qp[1];
        if (cidc == 2) {
            qp[0] += 3;
            qp[1] += 3;
        }
        if (transform_bypass) {
            if (IS_INTRA(mb_type) && h->active_sps->profile_idc == 244 &&
                (sl->chroma_pred_mode == 1 || sl->chroma_pred_mode == 2)) {
                h264_pred8x8_add_16(sl->chroma_pred_mode, (uint16_t *)dest_cb,
                                    block_offset + 16,
                                    ((int32_t *)sl->mb) + 16 * 16, uvstride);
                h264_pred8x8_add_16(sl->chroma_pred_mode, (uint16_t *)dest_cr,
                                    block_offset + 32,
                                    ((int32_t *)sl->mb) + 16 * 16 * 2, uvstride);
            } else {
                for (j = 1; j < 3; j++) {
                    for (i = j * 16; i < j * 16 + 4; i++)
                        if (sl->non_zero_count_cache[h264_scan8[i]] ||
                            ((int32_t *)sl->mb)[i * 16])
                            h264_add_pixels4_16(dest[j - 1] + block_offset[i],
                                                ((int32_t *)sl->mb) + i * 16,
                                                uvstride);
                    if (cidc == 2) {
                        for (i = j * 16 + 4; i < j * 16 + 8; i++)
                            if (sl->non_zero_count_cache[h264_scan8[i + 4]] ||
                                ((int32_t *)sl->mb)[i * 16])
                                h264_add_pixels4_16(dest[j - 1] + block_offset[i + 4],
                                                    ((int32_t *)sl->mb) + i * 16,
                                                    uvstride);
                    }
                }
            }
        } else {
            if (sl->non_zero_count_cache[h264_scan8[49]]) {
                if (cidc == 2)
                    h264_chroma422_dc_dequant_idct_16(
                        (int32_t *)(sl->mb + (256 << 1)),
                        h->active_pps->dequant4_coeff[IS_INTRA(mb_type) ? 1 : 4][qp[0]][0]);
                else
                    h264_chroma_dc_dequant_idct_16(
                        (int32_t *)(sl->mb + (256 << 1)),
                        h->active_pps->dequant4_coeff[IS_INTRA(mb_type) ? 1 : 4][qp[0]][0]);
            }
            if (sl->non_zero_count_cache[h264_scan8[50]]) {
                if (cidc == 2)
                    h264_chroma422_dc_dequant_idct_16(
                        (int32_t *)(sl->mb + ((256 + 256) << 1)),
                        h->active_pps->dequant4_coeff[IS_INTRA(mb_type) ? 2 : 5][qp[1]][0]);
                else
                    h264_chroma_dc_dequant_idct_16(
                        (int32_t *)(sl->mb + ((256 + 256) << 1)),
                        h->active_pps->dequant4_coeff[IS_INTRA(mb_type) ? 2 : 5][qp[1]][0]);
            }
            if (cidc == 2)
                h264_idct_add8_422_16(dest, block_offset, (int32_t *)sl->mb,
                                      uvstride, sl->non_zero_count_cache);
            else
                h264_idct_add8_16(dest, block_offset, (int32_t *)sl->mb, uvstride,
                                  sl->non_zero_count_cache);
        }
    }
}

/* Filter-cache entry points used by the deblocker. */
void h264_fill_decode_neighbors(H264Decoder *h, H264Slice *sl, uint32_t mb_type)
{
    fill_decode_neighbors(h, sl, mb_type);
}

void h264_fill_decode_caches(H264Decoder *h, H264Slice *sl, uint32_t mb_type)
{
    fill_decode_caches(h, sl, mb_type);
}
