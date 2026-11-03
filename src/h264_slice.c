/*
 * Slice header parsing, picture setup and the macroblock decode loop.
 */
#include "h264_internal.h"

#define FIELD_OR_MBAFF_PICTURE(h) (h->cur_pic && (h->cur_pic->mbaff || h->cur_pic->field_picture))
#define MB_FIELD(sl) ((sl)->mb_field_decoding_flag)

static int64_t gcd64(int64_t a, int64_t b)
{
    while (b) {
        int64_t t = a % b;
        a = b;
        b = t;
    }
    return a;
}

static void fill_stream_info(H264Decoder *h, const H264SPS *sps)
{
    h264_stream_info *info = &h->info;
    info->profile = sps->profile_idc;
    info->level = sps->level_idc;
    info->coded_width = 16 * sps->mb_width;
    info->coded_height = 16 * sps->mb_height;
    info->width = info->coded_width - sps->crop_left - sps->crop_right;
    info->height = info->coded_height - sps->crop_top - sps->crop_bottom;
    info->chroma_format = sps->chroma_format_idc;
    info->bit_depth = sps->bit_depth_luma;
    info->progressive_seq = sps->frame_mbs_only_flag;
    info->sar_width = sps->sar_num;
    info->sar_height = sps->sar_den;
    info->max_num_reorder_frames = sps->num_reorder_frames;
    info->max_dec_frame_buffering = sps->max_dec_frame_buffering;
    if (sps->timing_info_present_flag && sps->num_units_in_tick && sps->time_scale) {
        int64_t num = sps->time_scale;
        int64_t den = 2 * (int64_t)sps->num_units_in_tick;
        int64_t g = gcd64(num, den);
        if (g > 0) {
            num /= g;
            den /= g;
        }
        /* a damaged VUI can name values that do not fit the int fields, and a
         * zero denominator would make the caller's timestamp arithmetic a
         * division by zero */
        if (num > 0 && den > 0 && num <= 0x7fffffff && den <= 0x7fffffff) {
            info->frame_rate_num = (int)num;
            info->frame_rate_den = (int)den;
        } else {
            info->frame_rate_num = 0;
            info->frame_rate_den = 1;
        }
    } else {
        info->frame_rate_num = 0;
        info->frame_rate_den = 1;
    }
}

void h264_slice_bind_scratch(H264Decoder *h, H264Slice *sl)
{
    /* the rolling two-row intra mode buffer belongs to the decoder copy that
     * runs the pixel work, which may not be the one that initialised the
     * slice */
    sl->intra4x4_pred_mode = h->intra4x4_pred_mode + I4_MODE_PAD;
}

static int alloc_mb_tables(H264Decoder *h)
{
    const int big_mb_num = h->mb_stride * (h->mb_height + 1) + h->mb_stride;
    const int row_mb_num = 2 * h->mb_stride;
    int x, y;

    free(h->mb2b_xy);
    free(h->mb2br_xy);
    free(h->intra4x4_pred_mode);
    h->mb2b_xy = (uint32_t *)calloc(big_mb_num, sizeof(uint32_t));
    h->mb2br_xy = (uint32_t *)calloc(big_mb_num, sizeof(uint32_t));
    /* The MBAFF mode[] reads can index below slot 0, so keep a guard area. */
    h->intra4x4_pred_mode = (int8_t *)calloc(1, (size_t)row_mb_num * 8 + 2 * I4_MODE_PAD);
    if (!h->mb2b_xy || !h->mb2br_xy || !h->intra4x4_pred_mode)
        return -1;
    for (y = 0; y < h->mb_height; y++)
        for (x = 0; x < h->mb_width; x++) {
            const int mb_xy = x + y * h->mb_stride;
            h->mb2b_xy[mb_xy] = 4 * x + 4 * y * h->b_stride;
            h->mb2br_xy[mb_xy] = 8 * (mb_xy % (2 * h->mb_stride));
        }
    return 0;
}

int h264_activate_ps(H264Decoder *h, H264Slice *sl)
{
    H264SPS *sps = sl->sps;
    H264PPS *pps = sl->pps;
    int dims_changed = !h->mb2b_xy ||
        h->mb_width != sps->mb_width || h->mb_height != sps->mb_height ||
        (h->active_sps &&
         (h->active_sps->bit_depth_luma != sps->bit_depth_luma ||
          h->active_sps->chroma_format_idc != sps->chroma_format_idc));

    if (h->active_sps && h->active_sps != sps && dims_changed) {
        h264_job_wait_all(h);
        h264_clear_refs(h);
        h264_flush_frames(h);
        h->next_outputed_poc = 0;
    }
    /* The active pair must outlive the parameter set table: a repeated SPS/PPS
     * at an IDR replaces the slot while a worker may still be decoding against
     * the old objects, and a borrowed pointer dangles the moment the table
     * drops its own reference. FFmpeg holds AVBufferRefs here for the same
     * reason. */
    if (h->active_sps != sps) {
        h264_sps_unref(&h->active_sps, "active swap");
        h->active_sps = h264_sps_ref(sps, "active");
    }
    if (h->active_pps != pps) {
        h264_pps_unref(&h->active_pps, "active swap");
        h->active_pps = h264_pps_ref(pps, "active");
    }
    h->mb_width = sps->mb_width;
    h->mb_height = sps->mb_height;
    h->mb_stride = sps->mb_width + 1;
    h->b_stride = sps->mb_width * 4;
    h->width = 16 * h->mb_width;
    h->height = 16 * h->mb_height;
    h->chroma_y_shift = 1;
    h->max_reorder = H264_CLIP(sps->num_reorder_frames, 0, 16);

    if (dims_changed) {
        /* the MB geometry tables are shared with the workers */
        h264_job_wait_all(h);
        if (alloc_mb_tables(h) < 0)
            return -1;
    }
    fill_stream_info(h, sps);
    h264_init_scan_tables(h);
    h->got_header = 1;
    return 0;
}

/* ------------------------------------------------------------------ */
/* slice header                                                        */
/* ------------------------------------------------------------------ */
static int parse_ref_count(H264BitReader *br, const H264PPS *pps, int slice_type_nos,
                           int picture_structure, int ref_count[2], int *list_count)
{
    int override;
    unsigned max, v;

    ref_count[0] = pps->ref_count[0];
    ref_count[1] = pps->ref_count[1];
    if (slice_type_nos == H264_SLICE_I) {
        *list_count = 0;
        ref_count[0] = ref_count[1] = 0;
        return 0;
    }

    override = (int)h264_br_get_bits1(br);
    if (override) {
        /* bounded before the +1 so the value cannot wrap when it becomes int */
        max = picture_structure == PICT_FRAME ? 15u : 31u;
        v = h264_br_get_ue(br);
        if (v >= max)
            return -1;
        ref_count[0] = (int)v + 1;
        if (slice_type_nos == H264_SLICE_B) {
            v = h264_br_get_ue(br);
            if (v >= max)
                return -1;
            ref_count[1] = (int)v + 1;
        } else {
            ref_count[1] = 1;
        }
    }
    *list_count = slice_type_nos == H264_SLICE_B ? 2 : 1;
    if (ref_count[0] < 1 || ref_count[1] < 1)
        return -1;
    /* the defaults in the PPS are not tied to a picture structure, so they
     * need the same limit the override gets: 16 frames, 32 fields */
    if (picture_structure == PICT_FRAME &&
        (ref_count[0] > 16 || ref_count[1] > 16))
        return -1;
    return 0;
}

static int parse_ref_reordering(H264Slice *sl, H264BitReader *br)
{
    int list;
    sl->nb_ref_modifications[0] = sl->nb_ref_modifications[1] = 0;
    for (list = 0; list < sl->list_count; list++) {
        int index;
        if (!h264_br_get_bits1(br))
            continue;
        for (index = 0; ; index++) {
            unsigned op = h264_br_get_ue(br);
            if (op == 3)
                break;
            if (index >= sl->ref_count[list] || op > 2)
                return -1;
            sl->ref_modifications[list][index].op = (uint8_t)op;
            sl->ref_modifications[list][index].val = h264_br_get_ue(br);
            sl->nb_ref_modifications[list]++;
        }
    }
    return 0;
}

static int parse_pred_weight(H264Slice *sl, H264BitReader *br, const H264SPS *sps)
{
    int list, i, j;
    int luma_def, chroma_def;
    H264PredWeightTable *pwt = &sl->pwt;

    memset(pwt, 0, sizeof(*pwt));
    pwt->luma_log2_weight_denom = (int)h264_br_get_ue(br);
    if (pwt->luma_log2_weight_denom > 7)
        return -1;
    luma_def = 1 << pwt->luma_log2_weight_denom;
    if (sps->chroma_format_idc) {
        pwt->chroma_log2_weight_denom = (int)h264_br_get_ue(br);
        if (pwt->chroma_log2_weight_denom > 7)
            return -1;
        chroma_def = 1 << pwt->chroma_log2_weight_denom;
    } else {
        chroma_def = 0;
    }

    for (list = 0; list < 2; list++) {
        pwt->luma_weight_flag[list] = 0;
        pwt->chroma_weight_flag[list] = 0;
        for (i = 0; i < sl->ref_count[list]; i++) {
            int luma_flag = (int)h264_br_get_bits1(br);
            if (luma_flag) {
                pwt->luma_weight[i][list][0] = h264_br_get_se(br);
                pwt->luma_weight[i][list][1] = h264_br_get_se(br);
                if ((int8_t)pwt->luma_weight[i][list][0] != pwt->luma_weight[i][list][0] ||
                    (int8_t)pwt->luma_weight[i][list][1] != pwt->luma_weight[i][list][1])
                    return -1;
                if (pwt->luma_weight[i][list][0] != luma_def ||
                    pwt->luma_weight[i][list][1] != 0) {
                    pwt->use_weight = 1;
                    pwt->luma_weight_flag[list] = 1;
                }
            } else {
                pwt->luma_weight[i][list][0] = luma_def;
                pwt->luma_weight[i][list][1] = 0;
            }
            if (sps->chroma_format_idc) {
                int chroma_flag = (int)h264_br_get_bits1(br);
                if (chroma_flag) {
                    for (j = 0; j < 2; j++) {
                        pwt->chroma_weight[i][list][j][0] = h264_br_get_se(br);
                        pwt->chroma_weight[i][list][j][1] = h264_br_get_se(br);
                        if ((int8_t)pwt->chroma_weight[i][list][j][0] != pwt->chroma_weight[i][list][j][0] ||
                            (int8_t)pwt->chroma_weight[i][list][j][1] != pwt->chroma_weight[i][list][j][1])
                            return -1;
                        if (pwt->chroma_weight[i][list][j][0] != chroma_def ||
                            pwt->chroma_weight[i][list][j][1] != 0) {
                            pwt->use_weight_chroma = 1;
                            pwt->chroma_weight_flag[list] = 1;
                        }
                    }
                } else {
                    for (j = 0; j < 2; j++) {
                        pwt->chroma_weight[i][list][j][0] = chroma_def;
                        pwt->chroma_weight[i][list][j][1] = 0;
                    }
                }
            }
            if (sl->picture_structure == PICT_FRAME) {
                pwt->luma_weight[16 + 2 * i][list][0] = pwt->luma_weight[16 + 2 * i + 1][list][0] = pwt->luma_weight[i][list][0];
                pwt->luma_weight[16 + 2 * i][list][1] = pwt->luma_weight[16 + 2 * i + 1][list][1] = pwt->luma_weight[i][list][1];
                if (sps->chroma_format_idc) {
                    for (j = 0; j < 2; j++) {
                        pwt->chroma_weight[16 + 2 * i][list][j][0] = pwt->chroma_weight[16 + 2 * i + 1][list][j][0] = pwt->chroma_weight[i][list][j][0];
                        pwt->chroma_weight[16 + 2 * i][list][j][1] = pwt->chroma_weight[16 + 2 * i + 1][list][j][1] = pwt->chroma_weight[i][list][j][1];
                    }
                }
            }
        }
        if (sl->slice_type_nos != H264_SLICE_B)
            break;
    }
    pwt->use_weight = pwt->use_weight || pwt->use_weight_chroma;
    return br->error ? -1 : 0;
}

static int parse_ref_marking(H264Slice *sl, H264BitReader *br, int nal_type)
{
    int nb = 0;
    H264MMCO *mmco = sl->mmco;

    if (nal_type == 5) {
        h264_br_get_bits1(br); /* no_output_of_prior_pics_flag */
        if (h264_br_get_bits1(br)) {
            mmco[0].opcode = 5; /* MMCO_LONG */
            mmco[0].long_arg = 0;
            nb = 1;
        }
        sl->explicit_ref_marking = 1;
    } else {
        sl->explicit_ref_marking = (int)h264_br_get_bits1(br);
        if (sl->explicit_ref_marking) {
            int i;
            for (i = 0; i < H264_MAX_MMCO_COUNT; i++) {
                unsigned opcode = h264_br_get_ue(br);
                if (opcode > 6)
                    return -1;
                mmco[i].opcode = (uint8_t)opcode;
                if (opcode == 1 || opcode == 3) {
                    mmco[i].short_pic_num = (int)((sl->curr_pic_num -
                        (int)h264_br_get_ue(br) - 1) & (sl->max_pic_num - 1));
                }
                if (opcode == 3 || opcode == 2 || opcode == 5 || opcode == 4) {
                    unsigned arg = h264_br_get_ue(br);
                    if (arg > 32)
                        return -1;
                    mmco[i].long_arg = (int)arg;
                }
                if (opcode == 0) {
                    nb = i;
                    break;
                }
                nb = i + 1;
            }
        }
    }
    sl->nb_mmco = nb;
    return br->error ? -1 : 0;
}

int h264_decode_slice_header(H264Decoder *h, H264Slice *sl, const H264NAL *nal,
                             int nal_ref_idc, int nal_type)
{
    H264BitReader *br = &sl->gb;
    unsigned raw_slice_type;
    unsigned pps_id;
    H264PPS *pps;
    H264SPS *sps;
    int field_pic_flag = 0, bottom_field_flag = 0;
    int picture_structure;
    int i;

    (void)h;

    sl->first_mb_addr = h264_br_get_ue(br);
    raw_slice_type = h264_br_get_ue(br);
    if (raw_slice_type > 9) {
        fprintf(stderr, "[hdr] slice_type %u too large (nal %d, first_mb %u)\n",
                raw_slice_type, nal_type, sl->first_mb_addr);
        return -1;
    }
    if (raw_slice_type > 4) {
        raw_slice_type -= 5;
        sl->slice_type_fixed = 1;
    } else {
        sl->slice_type_fixed = 0;
    }
    switch (raw_slice_type) {
    case 0: sl->slice_type = H264_SLICE_P; break;
    case 1: sl->slice_type = H264_SLICE_B; break;
    case 2: sl->slice_type = H264_SLICE_I; break;
    case 3: sl->slice_type = H264_SLICE_SP; break;
    case 4: sl->slice_type = H264_SLICE_SI; break;
    default:
        fprintf(stderr, "[h264dec] invalid slice_type %u\n", raw_slice_type);
        return -1;
    }
    sl->slice_type_nos = sl->slice_type == H264_SLICE_SI ? H264_SLICE_I :
                         sl->slice_type == H264_SLICE_SP ? H264_SLICE_P :
                         sl->slice_type;
    if (nal_type == 5 && sl->slice_type_nos != H264_SLICE_I) {
        fprintf(stderr, "[hdr] non-intra slice in IDR\n");
        return -1;
    }

    pps_id = h264_br_get_ue(br);
    if (pps_id >= H264_MAX_PPS_COUNT || !h->pps_list[pps_id]) {
        fprintf(stderr, "[hdr] bad pps_id %u (slice_type %d)\n", pps_id, sl->slice_type);
        return -1;
    }
    pps = h->pps_list[pps_id];
    sps = pps->sps;
    sl->pps_id = pps_id;
    sl->pps = h264_pps_ref(pps, "slice");
    sl->sps = h264_sps_ref(sps, "slice");

    sl->frame_num = (int)h264_br_get_bits(br, sps->log2_max_frame_num);
    sl->mb_mbaff = 0;

    if (sps->frame_mbs_only_flag) {
        picture_structure = PICT_FRAME;
    } else {
        field_pic_flag = (int)h264_br_get_bits1(br);
        if (field_pic_flag) {
            bottom_field_flag = (int)h264_br_get_bits1(br);
            picture_structure = PICT_TOP_FIELD + bottom_field_flag;
        } else {
            picture_structure = PICT_FRAME;
        }
    }
    sl->picture_structure = picture_structure;
    sl->mb_field_decoding_flag = picture_structure != PICT_FRAME;

    if (picture_structure == PICT_FRAME) {
        sl->curr_pic_num = sl->frame_num;
        sl->max_pic_num = 1 << sps->log2_max_frame_num;
    } else {
        sl->curr_pic_num = 2 * sl->frame_num + 1;
        sl->max_pic_num = 1 << (sps->log2_max_frame_num + 1);
    }

    if (nal_type == 5)
        sl->idr_pic_id = (int)h264_br_get_ue(br);

    sl->poc_lsb = 0;
    sl->delta_poc_bottom = 0;
    if (sps->poc_type == 0) {
        sl->poc_lsb = (int)h264_br_get_bits(br, sps->log2_max_poc_lsb);
        if (pps->pic_order_present == 1 && picture_structure == PICT_FRAME)
            sl->delta_poc_bottom = h264_br_get_se(br);
    }
    sl->delta_poc[0] = sl->delta_poc[1] = 0;
    if (sps->poc_type == 1 && !sps->delta_pic_order_always_zero_flag) {
        sl->delta_poc[0] = h264_br_get_se(br);
        if (pps->pic_order_present == 1 && picture_structure == PICT_FRAME)
            sl->delta_poc[1] = h264_br_get_se(br);
    }

    sl->redundant_pic_count = 0;
    if (pps->redundant_pic_cnt_present)
        sl->redundant_pic_count = (int)h264_br_get_ue(br);

    if (sl->slice_type_nos == H264_SLICE_B)
        sl->direct_spatial_mv_pred = (int)h264_br_get_bits1(br);

    if (parse_ref_count(br, pps, sl->slice_type_nos, picture_structure,
                        sl->ref_count, &sl->list_count) < 0) {
        fprintf(stderr, "[hdr] ref_count fail type=%d\n", sl->slice_type_nos);
        return -1;
    }

    if (h264_dbg()->bits)
        fprintf(stderr, "[hdr] stage ref_count pos=%d type=%d list=%d rc0=%d rc1=%d\n",
                br->pos, sl->slice_type_nos, sl->list_count, sl->ref_count[0], sl->ref_count[1]);
    if (sl->slice_type_nos != H264_SLICE_I) {
        if (parse_ref_reordering(sl, br) < 0) {
            fprintf(stderr, "[hdr] ref_reorder fail type=%d\n", sl->slice_type_nos);
            return -1;
        }
    }

    memset(&sl->pwt, 0, sizeof(sl->pwt));
    if ((pps->weighted_pred && sl->slice_type_nos == H264_SLICE_P) ||
        (pps->weighted_bipred_idc == 1 && sl->slice_type_nos == H264_SLICE_B)) {
        if (parse_pred_weight(sl, br, sps) < 0) {
            fprintf(stderr, "[hdr] pred_weight fail type=%d\n", sl->slice_type_nos);
            return -1;
        }
    }

    if (h264_dbg()->bits)
        fprintf(stderr, "[hdr] stage reorder pos=%d\n", br->pos);
    sl->explicit_ref_marking = 0;
    sl->nb_mmco = 0;
    if (nal_ref_idc) {
        if (parse_ref_marking(sl, br, nal_type) < 0) {
            fprintf(stderr, "[hdr] ref_marking fail type=%d\n", sl->slice_type_nos);
            return -1;
        }
    }

    if (sl->slice_type_nos != H264_SLICE_I && pps->cabac) {
        sl->cabac_init_idc = (int)h264_br_get_ue(br);
        if (sl->cabac_init_idc > 2)
            return -1;
    }

    if (h264_dbg()->bits)
        fprintf(stderr, "[hdr] stage marking pos=%d nb_mmco=%d\n", br->pos, sl->nb_mmco);
    sl->last_qscale_diff = 0;
    {
        int dqp;
        if (h264_dbg()->bits)
            fprintf(stderr, "[hdr] preqp pos=%d init=%d\n", br->pos, pps->init_qp);
        dqp = h264_br_get_se(br);
        int qp = pps->init_qp + dqp;
        const int max_qp = 51 + 6 * (sl->sps->bit_depth_luma - 8);
        if (h264_dbg()->bits)
            fprintf(stderr, "[hdr] postqp pos=%d dqp=%d qp=%d\n", br->pos, dqp, qp);
        if (qp < 0 || qp > max_qp) {
            fprintf(stderr, "[hdr] qp out of range %d (init_qp %d, type %d)\n",
                    qp, pps->init_qp, sl->slice_type_nos);
            return -1;
        }
        sl->qscale = qp;
    }
    sl->chroma_qp[0] = h264_get_chroma_qp(pps, 0, sl->qscale);
    sl->chroma_qp[1] = h264_get_chroma_qp(pps, 1, sl->qscale);
    if (sl->slice_type == H264_SLICE_SP)
        h264_br_get_bits1(br);      /* sp_for_switch_flag */
    if (sl->slice_type == H264_SLICE_SP || sl->slice_type == H264_SLICE_SI)
        h264_br_get_se(br);         /* slice_qs_delta */

    sl->deblocking_filter = 1;
    sl->slice_alpha_c0_offset = 0;
    sl->slice_beta_offset = 0;
    if (pps->deblocking_filter_parameters_present) {
        unsigned idc = h264_br_get_ue(br);
        if (idc > 2) {
            fprintf(stderr, "[hdr] deblock idc %u type=%d pos=%d\n", idc, sl->slice_type_nos, br->pos);
            return -1;
        }
        sl->deblocking_filter = (int)idc;
        if (idc < 2)
            sl->deblocking_filter ^= 1; /* 0->1 enabled, 1->0 disabled */
        if (h264_dbg()->bits)
            fprintf(stderr, "[hdr] deblock idc=%u pos=%d\n", idc, br->pos);
        if (sl->deblocking_filter) {
            int a = h264_br_get_se(br);
            int b = h264_br_get_se(br);
            if (h264_dbg()->bits)
                fprintf(stderr, "[hdr] deblock a=%d b=%d pos=%d\n", a, b, br->pos);
            if (a < -6 || a > 6 || b < -6 || b > 6) {
                fprintf(stderr, "[hdr] deblock offsets out of range\n");
                return -1;
            }
            sl->slice_alpha_c0_offset = a * 2;
            sl->slice_beta_offset = b * 2;
        }
    }

    sl->nal_ref_idc = nal_ref_idc;
    sl->nal_type = nal_type;

    if (h264_dbg()->bits)
        fprintf(stderr, "HDR fn %d nr %d st %d type %d bit %d qp %d deb %d a%d b%d mmco %d ps %d\n",
                sl->frame_num, nal_ref_idc, sl->slice_type, nal_type,
                br->pos, sl->qscale, sl->deblocking_filter,
                sl->slice_alpha_c0_offset, sl->slice_beta_offset, sl->nb_mmco,
                sl->picture_structure);

    if (br->error) {
        fprintf(stderr, "[hdr] bitreader error pos=%d size=%d type=%d\n",
                br->pos, br->size_bits, sl->slice_type_nos);
        return -1;
    }
    (void)i;
    return 0;
}

/* ------------------------------------------------------------------ */
/* picture setup                                                       */
/* ------------------------------------------------------------------ */
static void reset_poc_state(H264Decoder *h)
{
    h->poc.prev_poc_msb = 1 << 16;
    h->poc.prev_poc_lsb = -1;
    h->poc.prev_frame_num_offset = 0;
    h->poc.prev_frame_num = 0;
    h->poc.frame_num = 0;
}

int h264_field_start(H264Decoder *h, H264Slice *sl)
{
    H264Frame *f;
    int second_field = 0;
    int structure = sl->picture_structure;

    /* An unmatched first field is output when superseded by another picture */
    if (h->cur_pic && h->first_field) {
        H264Frame *prev = h->cur_pic;
        if (structure != PICT_FRAME && prev->field_picture &&
            structure != h->picture_structure &&
            sl->frame_num == h->poc.frame_num &&
            (h->droppable == (sl->nal_ref_idc == 0)) &&
            /* the second field has to land in a frame allocated for the same
             * geometry; a damaged header can name a different SPS between the
             * two fields, and pairing them would write outside the first
             * field's buffers */
            prev->mb_width == sl->sps->mb_width &&
            prev->mb_height == sl->sps->mb_height &&
            prev->bit_depth == sl->sps->bit_depth_luma &&
            prev->chroma_format_idc == sl->sps->chroma_format_idc) {
            second_field = 1;
        } else {
            h264_job_wait_all(h);
            h264_queue_output(h, prev);
            h264_frame_unref(h, prev);
            h->cur_pic = NULL;
            h->first_field = 0;
        }
    }

    h->picture_structure = structure;
    h->droppable = sl->nal_ref_idc == 0;
    h->picture_idr = sl->nal_type == 5;

    if (h->picture_idr && !second_field) {
        /* the pictures before the recovery point still have to reach the
         * output queue in decode order */
        h264_job_wait_all(h);
        h264_clear_refs(h);
        h264_flush_frames(h);
        reset_poc_state(h);
        h->seq_epoch++;
        h->saw_idr = 1;
    }

    h->poc.frame_num = sl->frame_num;
    h->poc.poc_lsb = sl->poc_lsb;
    h->poc.delta_poc_bottom = sl->delta_poc_bottom;
    h->poc.delta_poc[0] = sl->delta_poc[0];
    h->poc.delta_poc[1] = sl->delta_poc[1];

    if (second_field) {
        f = h->cur_pic;
        f->picture_structure = structure;
        h->first_field = 0;
        if (structure == PICT_TOP_FIELD)
            f->have_top_field = 1;
        else
            f->have_bottom_field = 1;
    } else {
        if (h264_frame_start(h, sl) < 0)
            return -1;
        f = h->cur_pic;
        h->first_field = structure != PICT_FRAME;
        if (structure == PICT_TOP_FIELD)
            f->have_top_field = 1;
        else if (structure == PICT_BOTTOM_FIELD)
            f->have_bottom_field = 1;
    }

    f->frame_num = sl->frame_num;
    f->mbaff = sl->sps->mb_aff && structure == PICT_FRAME;
    f->cabac = sl->pps->cabac;

    if (h264_init_poc(h, f->field_poc, &f->poc, structure, sl->nal_ref_idc) < 0)
        return -1;
    f->poc = H264_MIN(f->field_poc[0], f->field_poc[1]);
    if (f->poc == INT32_MAX)
        f->poc = f->field_poc[0] == INT32_MAX ? 0 : f->field_poc[0];

    memcpy(h->mmco, sl->mmco, sizeof(h->mmco));
    h->nb_mmco = sl->nb_mmco;
    h->explicit_ref_marking = sl->explicit_ref_marking;
    h->current_slice = 0;
    h->mmco_reset = 0;
    return 0;
}

int h264_slice_init(H264Decoder *h, H264Slice *sl)
{
    H264Frame *f = h->cur_pic;
    const H264SPS *sps = sl->sps;
    int i;

    if (sl->first_mb_addr >= (unsigned)(h->mb_width * h->mb_height))
        return -1;
    /* every slice of an access unit belongs to the same picture structure;
     * a damaged header can mix a bottom-field slice into an MBAFF frame,
     * where the field row offset then walks the pair past the bottom row */
    if (sl->picture_structure != f->picture_structure)
        return -1;

    sl->pixel_shift = f->bit_depth > 8;
    sl->mb_x = sl->first_mb_addr % h->mb_width;
    sl->mb_y = (sl->first_mb_addr / h->mb_width) <<
               (f->mbaff || f->field_picture ? 1 : 0);
    if (sl->picture_structure == PICT_BOTTOM_FIELD)
        sl->mb_y++;
    if (sl->mb_y >= h->mb_height)
        return -1;
    sl->resync_mb_x = sl->mb_x;
    sl->resync_mb_y = sl->mb_y;
    sl->mb_xy = sl->mb_x + sl->mb_y * h->mb_stride;
    h264_slice_bind_scratch(h, sl);

    /* wrapped top-right slots of the 8x8 MV grid: never filled per MB, so
     * they must read as not available (as in the reference) */
    sl->ref_cache[0][h264_scan8[5] + 1] =
    sl->ref_cache[0][h264_scan8[7] + 1] =
    sl->ref_cache[0][h264_scan8[13] + 1] =
    sl->ref_cache[1][h264_scan8[5] + 1] =
    sl->ref_cache[1][h264_scan8[7] + 1] =
    sl->ref_cache[1][h264_scan8[13] + 1] = PART_NOT_AVAILABLE;

    if (sl->slice_type_nos != H264_SLICE_I) {
        if (h264_build_ref_list(h, sl) < 0)
            return -1;
    }

    if (sl->pps->weighted_bipred_idc == 2 && sl->slice_type_nos == H264_SLICE_B) {
        h264_implicit_weight_table(h, sl, -1);
        if (f->mbaff) {
            h264_implicit_weight_table(h, sl, 0);
            h264_implicit_weight_table(h, sl, 1);
        }
    }
    if (sl->slice_type_nos == H264_SLICE_B && !sl->direct_spatial_mv_pred)
        h264_direct_dist_scale_factor(h, sl);
    h264_direct_ref_list_init(h, sl);

    if (h->param.disable_lf)
        sl->deblocking_filter = 0;

    sl->slice_num = ++h->current_slice;
    f->slice_count = h->current_slice;

    h264_build_ref2frm(h, sl);

    sl->mb_skip_run = -1;
    sl->prev_mb_skipped = 0;
    sl->next_mb_skipped = 0;

    /* allocate slice scratch buffers */
    if (!sl->edge_emu_buffer) {
        int linelen = H264_ALIGN(f->linesize[0] * 2 + 64, 64);
        sl->edge_emu_alloc = linelen * 21 + 64;
        sl->edge_emu_buffer = (uint8_t *)malloc(sl->edge_emu_alloc);
        sl->bipred_alloc = linelen * 16 * 6;
        sl->bipred_scratchpad = (uint8_t *)malloc(sl->bipred_alloc);
        if (!sl->edge_emu_buffer || !sl->bipred_scratchpad)
            return -1;
    }

    (void)i;
    (void)sps;
    return 0;
}

/* ------------------------------------------------------------------ */
/* slice decoding                                                      */
/* ------------------------------------------------------------------ */

/* An MBAFF macroblock pair's field flag carries over from the pair above
 * when the row ends without the decoder reaching its read point, so it is
 * predicted from the already decoded neighbours (as in the reference). */
static void predict_field_decoding_flag(H264Decoder *h, H264Slice *sl)
{
    const int mb_xy = sl->mb_x + sl->mb_y * h->mb_stride;
    H264Frame *f = h->cur_pic;
    uint32_t mb_type;

    if (f->slice_table[mb_xy - 1] == sl->slice_num)
        mb_type = f->mb_type[mb_xy - 1];
    else if (f->slice_table[mb_xy - h->mb_stride] == sl->slice_num)
        mb_type = f->mb_type[mb_xy - h->mb_stride];
    else
        mb_type = 0;
    sl->mb_mbaff = sl->mb_field_decoding_flag = IS_INTERLACED(mb_type) ? 1 : 0;
}

int h264_decode_slice_data(H264Decoder *h, H264Slice *sl)
{
    H264Frame *f = h->cur_pic;
    const int mb_width = h->mb_width;
    const int mb_height = h->mb_height;
    const int field_or_mbaff = f->mbaff || f->field_picture;
    H264BitReader *br = &sl->gb;

    if (f->cabac) {
        h264_br_align(br);
        if (h264_cabac_init(sl, br->buffer + (br->pos >> 3),
                            (h264_br_bits_left(br) + 7) >> 3) < 0)
            return -1;
        h264_init_cabac_states(h, sl);
    }

    for (;;) {
        int ret;
        int eos = 0;

        if (sl->mb_x + sl->mb_y * mb_width >= sl->next_slice_idx) {
            return 0;
        }

        sl->mb_xy = sl->mb_x + sl->mb_y * h->mb_stride;
        ret = f->cabac ? h264_decode_mb_cabac(h, sl) : h264_decode_mb_cavlc(h, sl);
        if (ret >= 0)
            h264_hl_decode_mb(h, sl);

        if (ret >= 0 && f->mbaff) {
            sl->mb_y++;
            sl->mb_xy = sl->mb_x + sl->mb_y * h->mb_stride;
            ret = f->cabac ? h264_decode_mb_cabac(h, sl) : h264_decode_mb_cavlc(h, sl);
            if (ret >= 0)
                h264_hl_decode_mb(h, sl);
            sl->mb_y--;
        }

        if (f->cabac)
            eos = h264_cabac_terminate(sl);

        if (ret < 0 || (f->cabac &&
                        sl->cabac.bytestream > sl->cabac.bytestream_end + 4)) {
            if (h264_dbg()->bits)
                fprintf(stderr, "[dec] mb %d %d ret=%d cabac=%d over=%ld\n",
                        sl->mb_x, sl->mb_y, ret, f->cabac,
                        (long)(sl->cabac.bytestream - sl->cabac.bytestream_end));
            return -1;
        }

        if (++sl->mb_x >= mb_width) {
            sl->mb_x = 0;
            sl->mb_y += 1 + field_or_mbaff;
            /* the loop filter trails the decoder by one MB row: the row above
             * the one just decoded is finished, so its samples stay untouched
             * for the intra prediction of the current row and at the same time
             * become usable for other pictures. With the split the row is only
             * announced here; whoever is idle next filters it, one row behind,
             * on the same schedule. */
            if (f->filter_split)
                h264_recon_report(h, f, sl->mb_y, 0);
            else
                h264_deblock_upto(h, f, f->field_picture ? h->picture_structure
                                                         : PICT_FRAME,
                                  sl->mb_y - 1 - field_or_mbaff);
            if (f->mbaff && sl->mb_y < mb_height)
                predict_field_decoding_flag(h, sl);
            if (!f->cabac && sl->mb_y >= mb_height)
                return 0;
            if (!f->cabac && !h264_br_more_rbsp_data(br) && sl->mb_skip_run <= 0) {
                return 0;
            }
        }

        if (f->cabac) {
            if (eos || sl->mb_y >= mb_height)
                return 0;
        } else {
            if (!h264_br_more_rbsp_data(br) && sl->mb_skip_run <= 0) {
                /* remaining slices may follow; end this one */
                return 0;
            }
            if (br->error)
                return -1;
        }
    }
}

/* ------------------------------------------------------------------ */
/* end of picture                                                      */
/* ------------------------------------------------------------------ */
/* Header bookkeeping. It has to happen in decode order and before the next
 * picture builds its reference lists, so the decoder owner does it right
 * after the slices of a picture are initialised - a worker may still be
 * filling in the pixels of this one. */
/* SEI metadata belongs to the picture whose access unit carries it, so it is
 * copied onto the frame while its headers are still being finished: the owner
 * thread does this in decode order, before the picture goes to a worker, and
 * the flags therefore do not depend on when the picture is dequeued.
 *
 * The field order follows the reference's rules: pic_struct from the picture
 * timing message wins over what the decoding process implies, a single field
 * picture is interlaced by definition, and telecined patterns only ask for
 * extra display time. */
static void h264_picture_sei_props(H264Decoder *h, H264Frame *f)
{
    const int field_or_mbaff = f->field_picture || f->mbaff;
    int interlaced = field_or_mbaff;
    int pt_ok;

    if (!f->sei_applied) {
        f->sei_applied = 1;
        f->recovery_frame_cnt = h->sei.recovery_frame_cnt;
        f->exact_match = h->sei.exact_match;
        f->broken_recovery = h->sei.broken_recovery;
        f->pt_present = 0;
        if (h->sei.pt_present && h->active_sps &&
            h264_sei_read_picture_timing(h->active_sps, h->sei.pt_payload,
                                         h->sei.pt_size, &f->pt) == 0)
            f->pt_present = 1;
        if (!f->pt_present) {
            f->pt.cpb_removal_delay = -1;
            f->pt.pic_struct = -1;
        }
    }

    pt_ok = f->pt_present && f->pt.pic_struct >= 0;
    f->repeat_pict = 0;
    if (pt_ok) {
        switch (f->pt.pic_struct) {
        case 0:
            break;
        case 1: case 2:                 /* one field of an interlaced pair */
            interlaced = 1;
            break;
        case 3: case 4:                 /* frame coded as two fields */
            interlaced = field_or_mbaff ? 1 : h->prev_interlaced_frame;
            break;
        case 5: case 6:                 /* soft telecine */
            f->repeat_pict = 1;
            break;
        case 7:
            f->repeat_pict = 2;         /* frame doubling */
            break;
        case 8:
            f->repeat_pict = 4;         /* frame tripling */
            break;
        }
        /* counting_type 1 (progressive) or 2/3 (interlaced) overrides the
         * guess for the patterns that still describe a whole frame */
        if ((f->pt.ct_type & 3) && f->pt.pic_struct <= 4)
            interlaced = !!(f->pt.ct_type & 2);
    }
    h->prev_interlaced_frame = interlaced;
    f->interlaced = interlaced;

    if (f->field_poc[0] != f->field_poc[1])
        f->top_field_first = f->field_poc[0] < f->field_poc[1];
    else if (pt_ok)
        f->top_field_first = f->pt.pic_struct == 3 || f->pt.pic_struct == 5;
    else
        f->top_field_first = interlaced;
}

void h264_picture_headers_end(H264Decoder *h)
{
    if (h->cur_pic)
        h264_picture_sei_props(h, h->cur_pic);
    h264_sei_reset(h);
    if (!h->droppable) {
        h264_execute_ref_pic_marking(h);
        h->poc.prev_poc_msb = h->poc.poc_msb;
        h->poc.prev_poc_lsb = h->poc.poc_lsb;
    }
    h->poc.prev_frame_num_offset = h->poc.frame_num_offset;
    h->poc.prev_frame_num = h->poc.frame_num;
    h->current_slice = 0;
}

/* A picture that stopped half way leaves macroblocks that were never written.
 * Pool slots are recycled, so those would show the previous occupant: paint
 * them mid gray and let the caller decide through b_decode_error. Field
 * macroblocks are filled with their own addressing so the partner of a mixed
 * MB pair keeps its pixels. */
static void fill_missing_mbs(H264Decoder *h, H264Frame *f)
{
    int mb_x, mb_y;
    const int cidc = f->chroma_format_idc;
    const int cw = cidc == 3 ? 16 : 8;
    const int ch = cidc == 1 ? 8 : 16;

    for (mb_y = 0; mb_y < h->mb_height; mb_y++) {
        for (mb_x = 0; mb_x < h->mb_width; mb_x++) {
            const int mb_xy = mb_x + mb_y * h->mb_stride;
            const int field = f->field_picture ? 1 :
                              (f->mbaff ? !!IS_INTERLACED(f->mb_type[mb_xy]) : 0);
            int linesize = f->linesize[0] << field;
            int uvlinesize = f->linesize[1] << field;
            const int bytes = f->bit_depth > 8 ? 2 : 1;
            uint8_t *dst = f->data[0] + mb_x * 16 * bytes + mb_y * 16 * f->linesize[0];
            uint8_t *cb = cidc ? f->data[1] + mb_x * cw * bytes + mb_y * ch * f->linesize[1] : NULL;
            uint8_t *cr = cidc ? f->data[2] + mb_x * cw * bytes + mb_y * ch * f->linesize[2] : NULL;
            int i;

            if (f->slice_table[mb_xy] != 0xFFFF)
                continue;
            if (field && (mb_y & 1)) {
                dst -= 15 * f->linesize[0];
                if (cidc) {
                    cb -= (ch - 1) * f->linesize[1];
                    cr -= (ch - 1) * f->linesize[2];
                }
            }
            if (bytes == 1) {
                for (i = 0; i < 16; i++)
                    memset(dst + i * linesize, 16, 16);
                if (cidc) {
                    for (i = 0; i < ch; i++) {
                        memset(cb + i * uvlinesize, 128, cw);
                        memset(cr + i * uvlinesize, 128, cw);
                    }
                }
            } else {
                const uint16_t y = 16 << (f->bit_depth - 8);
                const uint16_t c = 128 << (f->bit_depth - 8);
                for (i = 0; i < 16; i++) {
                    uint16_t *p = (uint16_t *)(dst + i * linesize);
                    int x;
                    for (x = 0; x < 16; x++)
                        p[x] = y;
                }
                if (cidc) {
                    for (i = 0; i < ch; i++) {
                        uint16_t *p = (uint16_t *)(cb + i * uvlinesize);
                        uint16_t *q = (uint16_t *)(cr + i * uvlinesize);
                        int x;
                        for (x = 0; x < cw; x++) {
                            p[x] = c;
                            q[x] = c;
                        }
                    }
                }
            }
        }
    }
}

/* Pixel bookkeeping: finish the loop filter and publish the picture. Runs on
 * the worker, in its copy of the decoder state. */
void h264_picture_pixels(H264Decoder *h, H264Frame *f, int structure)
{
    if (f->filter_split) {
        /* every row is reconstructed; the chain may still be a couple behind */
        h264_recon_report(h, f, h->mb_height, 0);
        h264_filter_wait_done(h, f);
    } else {
        h264_deblock_upto(h, f, structure, h->mb_height);
    }
    f->deblock_done = 1;
    if (h->error) {
        f->b_decode_error = 1;
        fill_missing_mbs(h, f);
    }
    h264_progress_report(h, f, INT_MAX,
                         f->field_picture && structure == PICT_BOTTOM_FIELD);
}

/* Output bookkeeping, in decode order on the decoder owner. */
void h264_picture_output(H264Decoder *h, H264Frame *f, int structure)
{
    if (!f->field_picture || (f->have_top_field && f->have_bottom_field)) {
        h264_queue_output(h, f);
        h264_frame_unref(h, f);
        if (h->cur_pic == f) {
            h->cur_pic = NULL;
            h->first_field = 0;
        }
    } else {
        h->first_field = 1;
    }
}

int h264_picture_end(H264Decoder *h)
{
    H264Frame *f = h->cur_pic;
    int structure;

    if (!f)
        return 0;
    structure = f->field_picture ? h->picture_structure : PICT_FRAME;

    h264_picture_headers_end(h);
    h264_picture_pixels(h, f, structure);
    h264_picture_output(h, f, structure);

    h->nb_slices = 0;
    return 0;
}

static int clip_int8(int v)
{
    return v < -128 ? -128 : v > 127 ? 127 : v;
}

void h264_implicit_weight_table(H264Decoder *h, H264Slice *sl, int field)
{
    int ref0, ref1, i, cur_poc, ref_start, ref_count0, ref_count1;

    for (i = 0; i < 2; i++) {
        sl->pwt.luma_weight_flag[i] = 0;
        sl->pwt.chroma_weight_flag[i] = 0;
    }
    if (field < 0) {
        if (h->picture_structure == PICT_FRAME)
            cur_poc = h->cur_pic->poc;
        else
            cur_poc = h->cur_pic->field_poc[h->picture_structure - 1];
        if (sl->ref_count[0] == 1 && sl->ref_count[1] == 1 && !h->cur_pic->mbaff &&
            sl->ref_list[0][0].poc + (int64_t)sl->ref_list[1][0].poc == 2LL * cur_poc) {
            sl->pwt.use_weight = 0;
            sl->pwt.use_weight_chroma = 0;
            return;
        }
        ref_start = 0;
        ref_count0 = sl->ref_count[0];
        ref_count1 = sl->ref_count[1];
    } else {
        cur_poc = h->cur_pic->field_poc[field];
        ref_start = 16;
        ref_count0 = 16 + 2 * sl->ref_count[0];
        ref_count1 = 16 + 2 * sl->ref_count[1];
    }

    sl->pwt.use_weight = 2;
    sl->pwt.use_weight_chroma = 2;
    sl->pwt.luma_log2_weight_denom = 5;
    sl->pwt.chroma_log2_weight_denom = 5;

    for (ref0 = ref_start; ref0 < ref_count0; ref0++) {
        int64_t poc0 = sl->ref_list[0][ref0].poc;
        for (ref1 = ref_start; ref1 < ref_count1; ref1++) {
            int w = 32;
            if (!sl->ref_list[0][ref0].parent->long_ref &&
                !sl->ref_list[1][ref1].parent->long_ref) {
                int poc1 = sl->ref_list[1][ref1].poc;
                int td = clip_int8((int)(poc1 - poc0));
                if (td) {
                    int tb = clip_int8(cur_poc - (int)poc0);
                    int tx = (16384 + (H264_ABS(td) >> 1)) / td;
                    int dist_scale_factor = (tb * tx + 32) >> 8;
                    if (dist_scale_factor >= -64 && dist_scale_factor <= 128)
                        w = 64 - dist_scale_factor;
                }
            }
            if (field < 0) {
                sl->pwt.implicit_weight[ref0][ref1][0] = w;
                sl->pwt.implicit_weight[ref0][ref1][1] = w;
            } else {
                sl->pwt.implicit_weight[ref0][ref1][field] = w;
            }
        }
    }
}
