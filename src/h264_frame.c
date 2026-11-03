/*
 * Picture buffers, POC, reference lists/marking and output reordering.
 */
#include "h264_internal.h"

#define EDGE_PAD 32

static void reset_meta(H264Frame *f)
{
    const int off = 2 * f->mb_stride + 1;
    const int n = f->meta_count;
    int i;

    memset(f->mb_type - off, 0, (size_t)n * 4);
    memset(f->qscale_table - off, 0, (size_t)n);
    memset(f->slice_table - off, 0xFF, (size_t)n * 2);
    memset(f->nnz - off, 0, (size_t)n * 48);
    memset(f->cbp_table - off, 0, (size_t)n * 2);
    memset(f->chroma_pred_mode - off, 0, (size_t)n);
    memset(f->list_counts - off, 0, (size_t)n);
    memset(f->direct_table - off * 4, 0, (size_t)n * 4);
    memset(f->mb_deblock_idc - off, 0, (size_t)n);
    memset(f->mb_alpha - off, 0, (size_t)n);
    memset(f->mb_beta - off, 0, (size_t)n);
    memset(f->mb_skip - off, 0, (size_t)n);
    for (i = 0; i < 2; i++) {
        memset(f->mvd[i] - off, 0, (size_t)n * 16);
        memset(f->ref_index[i] - off * 4, LIST_NOT_USED, (size_t)n * 4);
        memset(f->motion_val[i] - f->b_stride - 4, 0,
               (size_t)f->b_stride * (f->mb_height * 4 + 2) * sizeof(int16_t) * 2);
    }
    memset(f->ref2frm, 0, H264_MAX_SLICES * sizeof(*f->ref2frm));
}

static int alloc_frame_buffers(H264Decoder *h, H264Frame *f, const H264SPS *sps)
{
    const int w = 16 * sps->mb_width;
    const int hh = 16 * sps->mb_height;
    const int cidc = sps->chroma_format_idc;
    const int ch = cidc == 3 ? w : w / 2;
    const int cv = cidc == 1 ? hh / 2 : hh;
    const int bytes = sps->bit_depth_luma > 8 ? 2 : 1;
    const int luma_stride = H264_ALIGN((w + 2 * EDGE_PAD) * bytes, 64);
    const int chroma_stride = H264_ALIGN((ch + 2 * EDGE_PAD) * bytes, 64);
    const int mb_stride = sps->mb_width + 1;
    const int big_mb_num = mb_stride * (sps->mb_height + 1);
    const int base_count = big_mb_num + 3 * mb_stride + 1;
    const int off = 2 * mb_stride + 1;
    const int b_stride = sps->mb_width * 4;
    int i;
    size_t need = 0;
    uint8_t *p;

    h264_free_frame_buffers(f);

    f->width = w;
    f->height = hh;
    f->bit_depth = sps->bit_depth_luma;
    f->chroma_format_idc = cidc;
    f->chroma_shift_x = cidc == 3 ? 0 : 1;
    f->chroma_shift_y = cidc == 1 ? 1 : 0;
    f->chroma_width = ch;
    f->chroma_height = cv;
    f->mb_width = sps->mb_width;
    f->mb_height = sps->mb_height;
    f->mb_stride = mb_stride;
    f->b_stride = b_stride;
    f->meta_count = base_count;
    f->crop_left = sps->crop_left;
    f->crop_top = sps->crop_top;
    f->crop_right = sps->crop_right;
    f->crop_bottom = sps->crop_bottom;
    f->disp_width = w - sps->crop_left - sps->crop_right;
    f->disp_height = hh - sps->crop_top - sps->crop_bottom;
    f->linesize[0] = luma_stride;
    f->linesize[1] = f->linesize[2] = chroma_stride;

    /* every macroblock needs the same 48 block offsets, so they are built once
     * per frame instead of in h264_hl_decode_mb. h264_scan8's low three bits
     * carry the block's x position in units of four samples and its next bits
     * the y position; a field-coded macroblock doubles both strides. */
    for (i = 0; i < 16; i++) {
        const int x = 4 * ((h264_scan8[i] - h264_scan8[0]) & 7);
        const int y = (h264_scan8[i] - h264_scan8[0]) >> 3;
        f->block_offset[0][i] = x + 4 * luma_stride * y;
        f->block_offset[0][16 + i] = f->block_offset[0][32 + i] =
            x + 4 * chroma_stride * y;
        f->block_offset[1][i] = x + 8 * luma_stride * y;
        f->block_offset[1][16 + i] = f->block_offset[1][32 + i] =
            x + 8 * chroma_stride * y;
    }

    f->buf_base[0] = (uint8_t *)calloc(1, (size_t)luma_stride * (hh + 2 * EDGE_PAD));
    if (cidc) {
        f->buf_base[1] = (uint8_t *)calloc(1, (size_t)chroma_stride * (cv + 2 * EDGE_PAD));
        f->buf_base[2] = (uint8_t *)calloc(1, (size_t)chroma_stride * (cv + 2 * EDGE_PAD));
    } else {
        f->buf_base[1] = f->buf_base[2] = NULL;
    }
    if (!f->buf_base[0] || (cidc && (!f->buf_base[1] || !f->buf_base[2])))
        goto fail;
    if (cidc && bytes == 1) {
        memset(f->buf_base[1], 0x80, (size_t)chroma_stride * (cv + 2 * EDGE_PAD));
        memset(f->buf_base[2], 0x80, (size_t)chroma_stride * (cv + 2 * EDGE_PAD));
    } else if (cidc) {
        const size_t n = (size_t)chroma_stride * (cv + 2 * EDGE_PAD) / 2;
        uint16_t *c1 = (uint16_t *)f->buf_base[1];
        uint16_t *c2 = (uint16_t *)f->buf_base[2];
        for (i = 0; i < (int)n; i++) {
            c1[i] = 512;
            c2[i] = 512;
        }
    }
    f->data[0] = f->buf_base[0] + EDGE_PAD * luma_stride + EDGE_PAD * bytes;
    if (cidc) {
        f->data[1] = f->buf_base[1] + EDGE_PAD * chroma_stride + EDGE_PAD * bytes;
        f->data[2] = f->buf_base[2] + EDGE_PAD * chroma_stride + EDGE_PAD * bytes;
    } else {
        f->data[1] = f->data[2] = NULL;
    }

    need += H264_ALIGN((size_t)base_count * 4, 16);
    need += H264_ALIGN((size_t)base_count, 16) * 2;         /* qscale, chroma_pred */
    need += H264_ALIGN((size_t)base_count * 2, 16);
    need += H264_ALIGN((size_t)base_count * 48, 16);
    need += H264_ALIGN((size_t)base_count * 2, 16);         /* cbp */
    need += H264_ALIGN((size_t)base_count, 16) * 5;         /* list_counts, idc, alpha, beta, skip */
    need += H264_ALIGN((size_t)base_count * 4, 16);
    need += H264_ALIGN((size_t)base_count * 16, 16) * 2;
    need += H264_ALIGN((size_t)base_count * 4, 16) * 2;
    f->meta_base = (uint8_t *)calloc(1, need);
    f->ref2frm = (int (*)[2][64])calloc(H264_MAX_SLICES, sizeof(*f->ref2frm));
    if (!f->meta_base || !f->ref2frm)
        goto fail;

    p = f->meta_base;
    f->mb_type = (uint32_t *)(void *)p; p += H264_ALIGN((size_t)base_count * 4, 16);
    f->qscale_table = (int8_t *)p;      p += H264_ALIGN((size_t)base_count, 16);
    f->slice_table = (uint16_t *)(void *)p; p += H264_ALIGN((size_t)base_count * 2, 16);
    f->nnz = (uint8_t (*)[48])(void *)p; p += H264_ALIGN((size_t)base_count * 48, 16);
    f->cbp_table = (uint16_t *)(void *)p; p += H264_ALIGN((size_t)base_count * 2, 16);
    f->chroma_pred_mode = (uint8_t *)p; p += H264_ALIGN((size_t)base_count, 16);
    f->list_counts = (uint8_t *)p;      p += H264_ALIGN((size_t)base_count, 16);
    f->direct_table = (uint8_t *)p;     p += H264_ALIGN((size_t)base_count * 4, 16);
    f->mb_deblock_idc = (int8_t *)p;    p += H264_ALIGN((size_t)base_count, 16);
    f->mb_alpha = (int8_t *)p;          p += H264_ALIGN((size_t)base_count, 16);
    f->mb_beta = (int8_t *)p;           p += H264_ALIGN((size_t)base_count, 16);
    f->mb_skip = (uint8_t *)p;          p += H264_ALIGN((size_t)base_count, 16);
    for (i = 0; i < 2; i++) {
        f->mvd[i] = (uint8_t (*)[8][2])(void *)p;
        p += H264_ALIGN((size_t)base_count * 16, 16);
    }
    for (i = 0; i < 2; i++) {
        f->ref_index[i] = (int8_t *)p;
        p += H264_ALIGN((size_t)base_count * 4, 16);
    }

    f->mb_type += off;
    f->qscale_table += off;
    f->slice_table += off;
    f->nnz += off;
    f->cbp_table += off;
    f->chroma_pred_mode += off;
    f->list_counts += off;
    f->direct_table += off * 4;
    f->mb_deblock_idc += off;
    f->mb_alpha += off;
    f->mb_beta += off;
    f->mb_skip += off;
    for (i = 0; i < 2; i++) {
        f->mvd[i] += off;
        f->ref_index[i] += off * 4;
    }

    for (i = 0; i < 2; i++) {
        size_t mv_count = (size_t)b_stride * (sps->mb_height * 4 + 2) + 8;
        f->motion_val_base[i] = (int16_t (*)[2])calloc(mv_count, sizeof(int16_t) * 2);
        if (!f->motion_val_base[i])
            goto fail;
        f->motion_val[i] = f->motion_val_base[i] + b_stride + 4;
    }

    reset_meta(f);
    return 0;

fail:
    h264_free_frame_buffers(f);
    return -1;
}

void h264_free_frame_buffers(H264Frame *f)
{
    int i;
    for (i = 0; i < 3; i++) {
        free(f->buf_base[i]);
        f->buf_base[i] = NULL;
        f->data[i] = NULL;
    }
    free(f->meta_base);
    f->meta_base = NULL;
    f->mb_type = NULL;
    f->qscale_table = NULL;
    f->slice_table = NULL;
    f->nnz = NULL;
    f->cbp_table = NULL;
    f->chroma_pred_mode = NULL;
    f->list_counts = NULL;
    f->direct_table = NULL;
    f->mb_deblock_idc = f->mb_alpha = f->mb_beta = NULL;
    f->mb_skip = NULL;
    f->mvd[0] = f->mvd[1] = NULL;
    f->ref_index[0] = f->ref_index[1] = NULL;
    for (i = 0; i < 2; i++) {
        free(f->motion_val_base[i]);
        f->motion_val_base[i] = NULL;
        f->motion_val[i] = NULL;
    }
    free(f->ref2frm);
    f->ref2frm = NULL;
    f->width = f->height = 0;
    f->mb_width = f->mb_height = 0;
}

/* Per-picture state that must not live on the thread: the lock and cond that
 * row progress is waited on, and the loop filter's macroblock caches. The filter
 * caches carry the previous macroblock from one row to the next, so a filter pass
 * that moves to another thread has to find them with the picture. */
static int frame_priv_create(H264Frame *f)
{
    f->sync = (H264FrameSync *)calloc(1, sizeof(*f->sync));
    if (!f->sync)
        return -1;
    h264_mutex_init(&f->sync->m);
    h264_cond_init(&f->sync->c);
    f->filter_sl = (struct H264Slice *)calloc(1, sizeof(*f->filter_sl));
    if (!f->filter_sl) {
        h264_cond_destroy(&f->sync->c);
        h264_mutex_destroy(&f->sync->m);
        free(f->sync);
        f->sync = NULL;
        return -1;
    }
    return 0;
}

static void frame_priv_destroy(H264Frame *f)
{
    if (f->sync) {
        h264_cond_destroy(&f->sync->c);
        h264_mutex_destroy(&f->sync->m);
        free(f->sync);
        f->sync = NULL;
    }
    free(f->filter_sl);
    f->filter_sl = NULL;
}

H264Frame *h264_alloc_frame(H264Decoder *h)
{
    int i;
    for (i = 0; i < H264_MAX_PICTURE_COUNT; i++) {
        if (!h->frame_pool[i]) {
            h->frame_pool[i] = (H264Frame *)calloc(1, sizeof(H264Frame));
            if (h->frame_pool[i] && frame_priv_create(h->frame_pool[i]) < 0) {
                free(h->frame_pool[i]);
                h->frame_pool[i] = NULL;
            }
        }
        if (h->frame_pool[i] && h->frame_pool[i]->ref_count == 0) {
            H264Frame *f = h->frame_pool[i];
            f->ref_count = 1;
            f->reference = 0;
            f->long_ref = 0;
            f->in_short_ref = f->in_long_ref = 0;
            f->field_picture = 0;
            f->mbaff = 0;
            f->picture_structure = PICT_FRAME;
            f->have_top_field = f->have_bottom_field = 0;
            f->field_poc[0] = f->field_poc[1] = 0;
            f->poc = 0;
            f->frame_num = 0;
            f->pic_id = 0;
            f->slice_count = 0;
            f->decode_done = 0;
            f->deblock_done = 0;
            f->deblock_row[0] = 0;
            f->deblock_row[1] = 1;
            f->progress[0] = -1;
            f->progress[1] = -1;
            f->recon[0] = -1;
            f->recon[1] = -1;
            f->filter_split = 0;
            /* The filter flags belong to the queue lock, not to whoever is
             * recycling the picture: a helper writes `claimed` under that lock, so
             * reading or resetting them outside it is a data race even when the
             * ordering makes it impossible in time. The guard sits in the same
             * section, so the state it reports is the state it resets. */
            h264_mutex_lock(&h->filter_q.m);
            if (f->filter_queued || f->filter_claimed)
                fprintf(stderr, "[h264dec] frame recycle assert: pool %d at zero "
                        "references still queued=%d claimed=%d done=%d\n",
                        i, f->filter_queued, f->filter_claimed, f->filter_done);
            __atomic_store_n(&f->filter_done, 0, __ATOMIC_RELEASE);
            f->filter_queued = 0;
            f->filter_claimed = 0;
            h264_mutex_unlock(&h->filter_q.m);
            f->b_decode_error = 0;
            f->output_pending = 0;
            f->output_sent = 0;
            f->pts = f->dts = 0;
            f->qp = 0;
            f->type = 0;
            return f;
        }
    }
    return NULL;
}

H264Frame *h264_frame_ref(H264Frame *f)
{
    if (f)
        f->ref_count++;
    return f;
}

static void frame_unref(H264Decoder *h, H264Frame *f)
{
    (void)h;
    if (!f)
        return;
    f->ref_count--;
    if (f->ref_count < 0)
        fprintf(stderr, "[h264dec] frame refcount underflow\n");
}

int h264_frame_start(H264Decoder *h, const H264Slice *sl)
{
    H264Frame *f;
    const H264SPS *sps = sl->sps;
    int t;

    f = h264_alloc_frame(h);
    if (!f) {
        fprintf(stderr, "[h264dec] no frame buffer available, DPB full\n");
        return -1;
    }

    if (f->mb_width != sps->mb_width || f->mb_height != sps->mb_height ||
        f->bit_depth != sps->bit_depth_luma ||
        f->chroma_format_idc != sps->chroma_format_idc || !f->data[0]) {
        if (alloc_frame_buffers(h, f, sps) < 0) {
            f->ref_count = 0;
            return -1;
        }
    } else {
        reset_meta(f);
    }

    f->field_picture = sl->picture_structure != PICT_FRAME;
    f->mbaff = sps->mb_aff && sl->picture_structure == PICT_FRAME;
    f->picture_structure = sl->picture_structure;
    f->field_poc[0] = f->field_poc[1] = INT32_MAX;
    /* picture metadata, until the SEI of this access unit says better */
    f->sei_applied = 0;
    f->pt_present = 0;
    memset(&f->pt, 0, sizeof(f->pt));
    f->pt.cpb_removal_delay = -1;
    f->pt.pic_struct = -1;
    f->recovery_frame_cnt = -1;
    f->exact_match = 0;
    f->broken_recovery = 0;
    f->interlaced = 0;
    f->repeat_pict = 0;
    f->top_field_first = 0;
    f->reference = h->droppable ? 0 : sl->picture_structure;
    f->frame_num = h->poc.frame_num;
    f->cabac = sl->pps->cabac;
    for (t = 0; t < 2; t++)
        memcpy(f->chroma_qp_table[t], sl->pps->chroma_qp_table[t],
               sizeof(f->chroma_qp_table[t]));
    f->chroma_qp_diff = sl->pps->chroma_qp_diff;
    f->pts = h->packet_pts;
    f->dts = h->packet_dts;
    f->type = sl->slice_type_nos;
    f->qp = sl->qscale;
    f->intra_only = 1;

    h->cur_pic = f;
    h->cur_first_mb = 0;
    return 0;
}

/* ------------------------------------------------------------------ */
/* POC                                                                 */
/* ------------------------------------------------------------------ */
int h264_init_poc(H264Decoder *h, int field_poc[2], int *poc,
                  int picture_structure, int nal_ref_idc)
{
    const H264SPS *sps = h->active_sps;
    H264POCContext *pc = &h->poc;
    const int max_frame_num = 1 << sps->log2_max_frame_num;
    int64_t fpoc[2];

    pc->frame_num_offset = pc->prev_frame_num_offset;
    if (pc->frame_num < pc->prev_frame_num)
        pc->frame_num_offset += max_frame_num;

    if (sps->poc_type == 0) {
        const int max_poc_lsb = 1 << sps->log2_max_poc_lsb;
        if (pc->prev_poc_lsb < 0)
            pc->prev_poc_lsb = pc->poc_lsb;
        if (pc->poc_lsb < pc->prev_poc_lsb &&
            pc->prev_poc_lsb - pc->poc_lsb >= max_poc_lsb / 2)
            pc->poc_msb = pc->prev_poc_msb + max_poc_lsb;
        else if (pc->poc_lsb > pc->prev_poc_lsb &&
                 pc->prev_poc_lsb - pc->poc_lsb < -max_poc_lsb / 2)
            pc->poc_msb = pc->prev_poc_msb - max_poc_lsb;
        else
            pc->poc_msb = pc->prev_poc_msb;
        fpoc[0] = fpoc[1] = pc->poc_msb + pc->poc_lsb;
        if (picture_structure == PICT_FRAME)
            fpoc[1] += pc->delta_poc_bottom;
    } else if (sps->poc_type == 1) {
        int abs_frame_num;
        int64_t expected_delta_per_poc_cycle = 0, expectedpoc;
        int i;

        if (sps->poc_cycle_length != 0)
            abs_frame_num = pc->frame_num_offset + pc->frame_num;
        else
            abs_frame_num = 0;
        if (nal_ref_idc == 0 && abs_frame_num > 0)
            abs_frame_num--;

        for (i = 0; i < sps->poc_cycle_length; i++)
            expected_delta_per_poc_cycle += sps->offset_for_ref_frame[i];

        if (abs_frame_num > 0) {
            int poc_cycle_cnt = (abs_frame_num - 1) / sps->poc_cycle_length;
            int frame_num_in_poc_cycle = (abs_frame_num - 1) % sps->poc_cycle_length;
            expectedpoc = poc_cycle_cnt * expected_delta_per_poc_cycle;
            for (i = 0; i <= frame_num_in_poc_cycle; i++)
                expectedpoc += sps->offset_for_ref_frame[i];
        } else {
            expectedpoc = 0;
        }
        if (nal_ref_idc == 0)
            expectedpoc += sps->offset_for_non_ref_pic;
        fpoc[0] = expectedpoc + pc->delta_poc[0];
        fpoc[1] = fpoc[0] + sps->offset_for_top_to_bottom_field;
        if (picture_structure == PICT_FRAME)
            fpoc[1] += pc->delta_poc[1];
    } else {
        int p = 2 * (pc->frame_num_offset + pc->frame_num);
        if (!nal_ref_idc)
            p--;
        fpoc[0] = fpoc[1] = p;
    }

    if (fpoc[0] != (int)fpoc[0] || fpoc[1] != (int)fpoc[1])
        return -1;

    if (picture_structure != PICT_BOTTOM_FIELD)
        field_poc[0] = (int)fpoc[0];
    if (picture_structure != PICT_TOP_FIELD)
        field_poc[1] = (int)fpoc[1];
    *poc = H264_MIN(field_poc[0], field_poc[1]);
    if (*poc == INT32_MAX)
        *poc = (int)fpoc[0];
    return 0;
}

/* ------------------------------------------------------------------ */
/* reference lists                                                     */
/* ------------------------------------------------------------------ */
static void ref_from_h264pic(H264Ref *dst, const H264Frame *src)
{
    memcpy(dst->data, src->data, sizeof(dst->data));
    memcpy(dst->linesize, src->linesize, sizeof(dst->linesize));
    dst->reference = src->reference;
    dst->poc = src->poc;
    dst->pic_id = src->pic_id;
    dst->parent = src;
}

static void pic_as_field(H264Ref *pic, int parity)
{
    int i;
    for (i = 0; i < 3; i++) {
        if (parity == PICT_BOTTOM_FIELD)
            pic->data[i] += pic->linesize[i];
        pic->linesize[i] *= 2;
    }
    pic->reference = parity;
    pic->poc = pic->parent->field_poc[parity == PICT_BOTTOM_FIELD];
}

static int split_field_copy(H264Ref *dest, const H264Frame *src, int parity, int id_add)
{
    int match = !!(src->reference & parity);
    if (match) {
        ref_from_h264pic(dest, src);
        if (parity != PICT_FRAME) {
            pic_as_field(dest, parity);
            dest->pic_id *= 2;
            dest->pic_id += id_add;
        }
    }
    return match;
}

static int build_def_list(H264Ref *def, int def_len, H264Frame *const *in,
                          int len, int is_long, int sel)
{
    int i[2] = { 0 }, index = 0;
    while (i[0] < len || i[1] < len) {
        while (i[0] < len && !(in[i[0]] && (in[i[0]]->reference & sel)))
            i[0]++;
        while (i[1] < len && !(in[i[1]] && (in[i[1]]->reference & (sel ^ 3))))
            i[1]++;
        if (i[0] < len) {
            if (index >= def_len) break;
            in[i[0]]->pic_id = is_long ? i[0] : in[i[0]]->frame_num;
            split_field_copy(&def[index++], in[i[0]++], sel, 1);
        }
        if (i[1] < len) {
            if (index >= def_len) break;
            in[i[1]]->pic_id = is_long ? i[1] : in[i[1]]->frame_num;
            split_field_copy(&def[index++], in[i[1]++], sel ^ 3, 0);
        }
    }
    return index;
}

static int add_sorted(H264Frame **sorted, H264Frame *const *src, int len,
                      int limit, int dir)
{
    int out_i = 0;
    for (;;) {
        int best_poc = dir ? INT32_MIN : INT32_MAX;
        H264Frame *best = NULL;
        int i;
        for (i = 0; i < len; i++) {
            const int poc = src[i]->poc;
            if (((poc > limit) ^ dir) && ((poc < best_poc) ^ dir)) {
                best_poc = poc;
                best = src[i];
            }
        }
        if (!best)
            break;
        sorted[out_i++] = best;
        limit = best_poc - dir;
    }
    return out_i;
}

static void fill_mbaff_ref_list(H264Slice *sl)
{
    int list, i, j;
    for (list = 0; list < sl->list_count; list++) {
        for (i = 0; i < sl->ref_count[list]; i++) {
            const H264Ref *frame = &sl->ref_list[list][i];
            H264Ref *field = &sl->ref_list[list][16 + 2 * i];
            field[0] = *frame;
            for (j = 0; j < 3; j++)
                field[0].linesize[j] <<= 1;
            field[0].reference = PICT_TOP_FIELD;
            field[0].poc = field[0].parent->field_poc[0];
            field[1] = field[0];
            for (j = 0; j < 3; j++)
                field[1].data[j] += frame->parent->linesize[j];
            field[1].reference = PICT_BOTTOM_FIELD;
            field[1].poc = field[1].parent->field_poc[1];
        }
    }
}

static void initialise_ref_list(H264Decoder *h, H264Slice *sl)
{
    int len;

    if (sl->slice_type_nos == H264_SLICE_B) {
        H264Frame *sorted[32];
        int cur_poc, lens[2], list;
        if (h->picture_structure != PICT_FRAME)
            cur_poc = h->cur_pic->field_poc[h->picture_structure == PICT_BOTTOM_FIELD];
        else
            cur_poc = h->cur_pic->poc;

        for (list = 0; list < 2; list++) {
            len  = add_sorted(sorted, h->short_ref, h->short_ref_count, cur_poc, 1 ^ list);
            len += add_sorted(sorted + len, h->short_ref, h->short_ref_count, cur_poc, 0 ^ list);
            len  = build_def_list(sl->ref_list[list], H264_MAX_LIST_SIZE, sorted, len, 0,
                                  h->picture_structure);
            len += build_def_list(sl->ref_list[list] + len, H264_MAX_LIST_SIZE - len,
                                  h->long_ref, 16, 1, h->picture_structure);
            memset(&sl->ref_list[list][len], 0, sizeof(H264Ref) * (H264_MAX_LIST_SIZE - len));
            lens[list] = len;
        }
        if (lens[0] == lens[1] && lens[1] > 1) {
            int i;
            for (i = 0; i < lens[0] &&
                        sl->ref_list[0][i].parent == sl->ref_list[1][i].parent; i++);
            if (i == lens[0]) {
                H264Ref tmp = sl->ref_list[1][0];
                sl->ref_list[1][0] = sl->ref_list[1][1];
                sl->ref_list[1][1] = tmp;
            }
        }
    } else {
        len  = build_def_list(sl->ref_list[0], H264_MAX_LIST_SIZE, h->short_ref,
                              h->short_ref_count, 0, h->picture_structure);
        len += build_def_list(sl->ref_list[0] + len, H264_MAX_LIST_SIZE - len,
                              h->long_ref, 16, 1, h->picture_structure);
        memset(&sl->ref_list[0][len], 0, sizeof(H264Ref) * (H264_MAX_LIST_SIZE - len));
    }

    for (len = 0; len < (int)(1 + (sl->slice_type_nos == H264_SLICE_B)); len++) {
        int i;
        for (i = 0; i < sl->ref_count[len]; i++) {
            if (sl->ref_list[len][i].parent &&
                (sl->ref_list[len][i].parent->width != h->cur_pic->width ||
                 sl->ref_list[len][i].parent->height != h->cur_pic->height))
                memset(&sl->ref_list[len][i], 0, sizeof(H264Ref));
        }
        h->default_ref[len] = sl->ref_list[len][0];
    }
}

static int pic_num_extract(H264Decoder *h, int pic_num, int *structure)
{
    *structure = h->picture_structure;
    if (h->picture_structure != PICT_FRAME) {
        if (!(pic_num & 1))
            *structure ^= PICT_FRAME;
        pic_num >>= 1;
    }
    return pic_num;
}

int h264_build_ref_list(H264Decoder *h, H264Slice *sl)
{
    int list, index;

    initialise_ref_list(h, sl);

    for (list = 0; list < sl->list_count; list++) {
        int pred = sl->curr_pic_num;
        for (index = 0; index < sl->nb_ref_modifications[list]; index++) {
            const unsigned op = sl->ref_modifications[list][index].op;
            const unsigned val = sl->ref_modifications[list][index].val;
            unsigned pic_id = 0;
            int i = -1, pic_structure = 0;
            H264Frame *ref = NULL;

            switch (op) {
            case 0:
            case 1: {
                const unsigned abs_diff_pic_num = val + 1;
                int frame_num;
                if (abs_diff_pic_num > (unsigned)sl->max_pic_num)
                    return -1;
                if (op == 0) pred -= abs_diff_pic_num;
                else         pred += abs_diff_pic_num;
                pred &= sl->max_pic_num - 1;
                frame_num = pic_num_extract(h, pred, &pic_structure);
                for (i = h->short_ref_count - 1; i >= 0; i--) {
                    ref = h->short_ref[i];
                    if (ref->frame_num == frame_num && (ref->reference & pic_structure))
                        break;
                }
                if (i >= 0)
                    pic_id = pred;
                break;
            }
            case 2: {
                int long_idx;
                pic_id = val;
                long_idx = pic_num_extract(h, pic_id, &pic_structure);
                if (long_idx > 31)
                    return -1;
                ref = h->long_ref[long_idx];
                i = (ref && (ref->reference & pic_structure)) ? 0 : -1;
                break;
            }
            default:
                return -1;
            }

            if (i < 0 || !ref) {
                fprintf(stderr, "[h264dec] missing reference during reorder\n");
                memset(&sl->ref_list[list][index], 0, sizeof(H264Ref));
            } else {
                for (i = index; i + 1 < sl->ref_count[list]; i++) {
                    if (sl->ref_list[list][i].parent &&
                        ref->long_ref == sl->ref_list[list][i].parent->long_ref &&
                        pic_id == (unsigned)sl->ref_list[list][i].pic_id)
                        break;
                }
                for (; i > index; i--)
                    sl->ref_list[list][i] = sl->ref_list[list][i - 1];
                ref_from_h264pic(&sl->ref_list[list][index], ref);
                if (h->picture_structure != PICT_FRAME)
                    pic_as_field(&sl->ref_list[list][index], pic_structure);
            }
        }
    }

    for (list = 0; list < sl->list_count; list++) {
        for (index = 0; index < sl->ref_count[list]; index++) {
            if (!sl->ref_list[list][index].parent ||
                (h->picture_structure == PICT_FRAME &&
                 (sl->ref_list[list][index].reference & 3) != 3)) {
                if (h->default_ref[list].parent &&
                    !(h->picture_structure == PICT_FRAME &&
                      (h->default_ref[list].reference & 3) != 3)) {
                    sl->ref_list[list][index] = h->default_ref[list];
                } else {
                    return -1;
                }
            }
        }
    }

    if (h->cur_pic->mbaff)
        fill_mbaff_ref_list(sl);

    return 0;
}

void h264_build_ref2frm(H264Decoder *h, H264Slice *sl)
{
    H264Frame *f = h->cur_pic;
    int list, i, k;

    for (list = 0; list < 2; list++) {
        int id_list[16];
        int *ref2frm = f->ref2frm[sl->slice_num & (H264_MAX_SLICES - 1)][list];

        for (i = 0; i < 16; i++) {
            id_list[i] = 60;
            if (list < sl->list_count && i < sl->ref_count[list] &&
                sl->ref_list[list][i].parent) {
                const H264Frame *parent = sl->ref_list[list][i].parent;
                for (k = 0; k < h->short_ref_count; k++)
                    if (h->short_ref[k] == parent) {
                        id_list[i] = k;
                        break;
                    }
                for (k = 0; k < 32; k++)
                    if (h->long_ref[k] == parent) {
                        id_list[i] = h->short_ref_count + k;
                        break;
                    }
            }
        }
        ref2frm[0] = ref2frm[1] = -1;
        for (i = 0; i < 16; i++)
            ref2frm[i + 2] = 4 * id_list[i] + (sl->ref_list[list][i].reference & 3);
        ref2frm[18] = ref2frm[19] = -1;
        for (i = 16; i < 48; i++)
            ref2frm[i + 4] = 4 * id_list[(i - 16) >> 1] +
                             (sl->ref_list[list][i].reference & 3);
    }
}

/* ------------------------------------------------------------------ */
/* reference marking                                                   */
/* ------------------------------------------------------------------ */
static int find_short(H264Decoder *h, int frame_num, int *idx)
{
    int i;
    for (i = 0; i < h->short_ref_count; i++) {
        if (h->short_ref[i]->frame_num == frame_num) {
            *idx = i;
            return 1;
        }
    }
    return 0;
}

static void drop_short_ref(H264Decoder *h, int i)
{
    H264Frame *f = h->short_ref[i];
    if (f && f->in_short_ref) {
        f->in_short_ref = 0;
        frame_unref(h, f);
    }
    h->short_ref[i] = NULL;
    if (--h->short_ref_count)
        memmove(&h->short_ref[i], &h->short_ref[i + 1],
                (h->short_ref_count - i) * sizeof(H264Frame *));
}

static void unreference_pic(H264Decoder *h, H264Frame *f, int refmask)
{
    f->reference &= refmask;
    if (!f->reference) {
        if (f->in_short_ref) {
            f->in_short_ref = 0;
            frame_unref(h, f);
        }
        if (f->in_long_ref) {
            f->in_long_ref = 0;
            f->long_ref = 0;
            frame_unref(h, f);
            h->long_ref_count--;
        }
    }
}

static H264Frame *remove_short(H264Decoder *h, int frame_num, int ref_mask)
{
    H264Frame *pic;
    int i;
    if (!find_short(h, frame_num, &i))
        return NULL;
    pic = h->short_ref[i];
    unreference_pic(h, pic, ref_mask);
    if (!pic->reference && pic->in_short_ref) {
        /* unreference_pic already released the list ref */
        h->short_ref[i] = NULL;
        if (--h->short_ref_count)
            memmove(&h->short_ref[i], &h->short_ref[i + 1],
                    (h->short_ref_count - i) * sizeof(H264Frame *));
    } else if (!pic->reference) {
        drop_short_ref(h, i);
    }
    return pic;
}

static H264Frame *remove_long(H264Decoder *h, int i, int ref_mask)
{
    H264Frame *pic = h->long_ref[i];
    if (pic) {
        unreference_pic(h, pic, ref_mask);
        if (!pic->reference && !pic->in_long_ref) {
            h->long_ref[i] = NULL;
        }
    }
    return pic;
}

void h264_clear_refs(H264Decoder *h)
{
    int i;
    for (i = 0; i < 32; i++)
        remove_long(h, i, 0);
    {
        int n = h->short_ref_count;
        for (i = 0; i < n; i++)
            unreference_pic(h, h->short_ref[i], 0);
    }
    h->short_ref_count = 0;
    memset(h->short_ref, 0, sizeof(h->short_ref));
    memset(h->long_ref, 0, sizeof(h->long_ref));
    h->long_ref_count = 0;
    memset(h->default_ref, 0, sizeof(h->default_ref));
}

static void generate_sliding_window_mmcos(H264Decoder *h)
{
    H264MMCO *mmco = h->mmco;
    int nb = 0;

    if (h->short_ref_count &&
        h->long_ref_count + h->short_ref_count >= h->active_sps->ref_frame_count &&
        !(h->picture_structure != PICT_FRAME && !h->first_field &&
          h->cur_pic->reference)) {
        mmco[0].opcode = 1;
        mmco[0].short_pic_num = h->short_ref[h->short_ref_count - 1]->frame_num;
        nb = 1;
        if (h->picture_structure != PICT_FRAME) {
            mmco[0].short_pic_num *= 2;
            mmco[1].opcode = 1;
            mmco[1].short_pic_num = mmco[0].short_pic_num + 1;
            nb = 2;
        }
    }
    h->nb_mmco = nb;
}

static void add_short_ref(H264Decoder *h, H264Frame *f)
{
    if (!f->in_short_ref && !f->in_long_ref) {
        h264_frame_ref(f);
        f->in_short_ref = 1;
    }
    if (h->short_ref_count)
        memmove(&h->short_ref[1], &h->short_ref[0],
                h->short_ref_count * sizeof(H264Frame *));
    h->short_ref[0] = f;
    h->short_ref_count++;
    f->reference |= h->picture_structure;
}

static void add_long_ref(H264Decoder *h, H264Frame *f, int arg)
{
    if (!f->in_long_ref) {
        h264_frame_ref(f);
        f->in_long_ref = 1;
        f->long_ref = 1;
    }
    h->long_ref[arg] = f;
    h->long_ref_count++;
    f->reference |= h->picture_structure;
}

int h264_execute_ref_pic_marking(H264Decoder *h)
{
    H264MMCO *mmco = h->mmco;
    int nb, i;
    int current_ref_assigned = 0;

    if (h->droppable)
        return 0;

    if (!h->explicit_ref_marking)
        generate_sliding_window_mmcos(h);
    nb = h->nb_mmco;

    for (i = 0; i < nb; i++) {
        switch (mmco[i].opcode) {
        case 1:
        case 3: {
            int structure, j;
            int frame_num = pic_num_extract(h, mmco[i].short_pic_num, &structure);
            if (!find_short(h, frame_num, &j)) {
                fprintf(stderr, "[h264dec] mmco: unref short failure\n");
                continue;
            }
            if (mmco[i].opcode == 1) {
                remove_short(h, frame_num, structure ^ PICT_FRAME);
            } else {
                H264Frame *pic = h->short_ref[j];
                if (h->long_ref[mmco[i].long_arg] != pic)
                    remove_long(h, mmco[i].long_arg, 0);
                /* move ownership from short list to long list */
                drop_short_ref(h, j);
                if (!pic->in_long_ref) {
                    h264_frame_ref(pic);
                    pic->in_long_ref = 1;
                    pic->long_ref = 1;
                    pic->in_short_ref = 0;
                }
                h->long_ref[mmco[i].long_arg] = pic;
                h->long_ref_count++;
            }
            break;
        }
        case 2: {
            int structure, j = pic_num_extract(h, mmco[i].long_arg, &structure);
            if (h->long_ref[j])
                remove_long(h, j, structure ^ PICT_FRAME);
            break;
        }
        case 5:
            if (h->short_ref_count && h->short_ref[0] == h->cur_pic)
                drop_short_ref(h, 0);
            if (h->cur_pic->long_ref) {
                int j;
                for (j = 0; j < 32; j++) {
                    if (h->long_ref[j] == h->cur_pic) {
                        if (j != mmco[i].long_arg)
                            fprintf(stderr, "[h264dec] mmco: double long ref\n");
                        remove_long(h, j, 0);
                    }
                }
            }
            if (h->long_ref[mmco[i].long_arg] != h->cur_pic) {
                remove_long(h, mmco[i].long_arg, 0);
                add_long_ref(h, h->cur_pic, mmco[i].long_arg);
            }
            h->cur_pic->reference |= h->picture_structure;
            current_ref_assigned = 1;
            break;
        case 4:
            if (mmco[i].long_arg <= 32) {
                int j;
                for (j = mmco[i].long_arg; j < 32; j++)
                    remove_long(h, j, 0);
            }
            break;
        case 6:
            while (h->short_ref_count)
                remove_short(h, h->short_ref[0]->frame_num, 0);
            {
                int j;
                for (j = 0; j < 32; j++)
                    remove_long(h, j, 0);
            }
            h->poc.frame_num = h->cur_pic->frame_num = 0;
            h->mmco_reset = 1;
            break;
        default:
            break;
        }
    }

    if (!current_ref_assigned) {
        if (h->short_ref_count && h->short_ref[0] == h->cur_pic) {
            h->cur_pic->reference |= h->picture_structure;
        } else if (h->cur_pic->long_ref) {
            fprintf(stderr, "[h264dec] illegal long-term second field\n");
        } else {
            H264Frame *pic = remove_short(h, h->cur_pic->frame_num, 0);
            if (pic)
                fprintf(stderr, "[h264dec] illegal short-term state\n");
            add_short_ref(h, h->cur_pic);
        }
    }

    if (h->long_ref_count + h->short_ref_count > H264_MAX(h->active_sps->ref_frame_count, 1)) {
        fprintf(stderr, "[h264dec] too many reference frames, discarding one\n");
        if (h->long_ref_count && !h->short_ref_count) {
            int j;
            for (j = 0; j < 32; j++)
                if (h->long_ref[j])
                    break;
            if (j < 32)
                remove_long(h, j, 0);
        } else if (h->short_ref_count) {
            H264Frame *pic = h->short_ref[h->short_ref_count - 1];
            remove_short(h, pic->frame_num, 0);
        }
    }

    return 0;
}

/* ------------------------------------------------------------------ */
/* output                                                              */
/* ------------------------------------------------------------------ */
static void push_ready(H264Decoder *h, H264Frame *f)
{
    if (h->ready_count < H264_MAX_PICTURE_COUNT)
        h->ready[h->ready_count++] = f;
    else
        frame_unref(h, f);
}

static void pop_output(H264Decoder *h)
{
    int i, best = -1;
    for (i = 0; i < h->delayed_count; i++) {
        if (best < 0 || h->delayed[i]->poc < h->delayed[best]->poc)
            best = i;
    }
    if (best < 0)
        return;
    {
        H264Frame *f = h->delayed[best];
        memmove(&h->delayed[best], &h->delayed[best + 1],
                (h->delayed_count - best - 1) * sizeof(H264Frame *));
        h->delayed_count--;
        f->output_pending = 0;
        f->output_sent = 1;
        h->next_outputed_poc = f->poc;
        push_ready(h, f);
    }
}

void h264_flush_frames(H264Decoder *h)
{
    while (h->delayed_count)
        pop_output(h);
}

void h264_queue_output(H264Decoder *h, H264Frame *f)
{
    if (h->param.skip_non_ref && !f->reference && !f->long_ref)
        return;
    if (h->param.output_order == 1) {
        f->output_sent = 1;
        h264_frame_ref(f);
        push_ready(h, f);
        return;
    }
    f->output_pending = 1;
    h264_frame_ref(f);
    if (h->delayed_count < H264_MAX_PICTURE_COUNT)
        h->delayed[h->delayed_count++] = f;
    else
        frame_unref(h, f);
    while (h->delayed_count > h->max_reorder)
        pop_output(h);
}

void h264_frame_unref(H264Decoder *h, H264Frame *f)
{
    frame_unref(h, f);
}

void h264_pool_free(H264Decoder *h)
{
    int i;
    for (i = 0; i < H264_MAX_PICTURE_COUNT; i++) {
        if (h->frame_pool[i]) {
            h264_free_frame_buffers(h->frame_pool[i]);
            frame_priv_destroy(h->frame_pool[i]);
            free(h->frame_pool[i]);
            h->frame_pool[i] = NULL;
        }
    }
    h->frame_pool_count = 0;
}
