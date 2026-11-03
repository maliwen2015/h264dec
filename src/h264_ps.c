/*
 * SPS/PPS parsing and quantizer table generation.
 *
 * Derived from ITU-T H.264 and FFmpeg's h264_ps.c (LGPL-2.1+).
 */
#include "h264_internal.h"

static const uint8_t default_scaling4[2][16] = {
    {  6, 13, 20, 28, 13, 20, 28, 32,
      20, 28, 32, 37, 28, 32, 37, 42 },
    { 10, 14, 20, 24, 14, 20, 24, 27,
      20, 24, 27, 30, 24, 27, 30, 34 }
};

static const uint8_t default_scaling8[2][64] = {
    {  6, 10, 13, 16, 18, 23, 25, 27,
      10, 11, 16, 18, 23, 25, 27, 29,
      13, 16, 18, 23, 25, 27, 29, 31,
      16, 18, 23, 25, 27, 29, 31, 33,
      18, 23, 25, 27, 29, 31, 33, 36,
      23, 25, 27, 29, 31, 33, 36, 38,
      25, 27, 29, 31, 33, 36, 38, 40,
      27, 29, 31, 33, 36, 38, 40, 42 },
    {  9, 13, 15, 17, 19, 21, 22, 24,
      13, 13, 17, 19, 21, 22, 24, 25,
      15, 17, 19, 21, 22, 24, 25, 27,
      17, 19, 21, 22, 24, 25, 27, 28,
      19, 21, 22, 24, 25, 27, 28, 30,
      21, 22, 24, 25, 27, 28, 30, 32,
      22, 24, 25, 27, 28, 30, 32, 33,
      24, 25, 27, 28, 30, 32, 33, 35 }
};

static const int level_max_dpb_mbs[][2] = {
    { 10, 396       },
    { 11, 900       },
    { 12, 2376      },
    { 13, 2376      },
    { 20, 2376      },
    { 21, 4752      },
    { 22, 8100      },
    { 30, 8100      },
    { 31, 18000     },
    { 32, 20480     },
    { 40, 32768     },
    { 41, 32768     },
    { 42, 34816     },
    { 50, 110400    },
    { 51, 184320    },
    { 52, 184320    },
};

static const int pixel_aspect[17][2] = {
    { 0, 1 }, { 1, 1 }, { 12, 11 }, { 10, 11 }, { 16, 11 }, { 40, 33 },
    { 24, 11 }, { 20, 11 }, { 32, 11 }, { 80, 33 }, { 18, 11 }, { 15, 11 },
    { 64, 33 }, { 160, 99 }, { 4, 3 }, { 3, 2 }, { 2, 1 },
};

/* The counts are atomic because the decoder owner takes a reference while it
 * parses a slice header - outside the pool lock, because header parsing is not
 * allowed to wait behind a reconstruction - at the same time as a worker drops
 * the references of the picture it has finished, inside the lock. A plain
 * ++/-- there loses an update and frees a parameter set that still has holders;
 * FFmpeg is safe here because its AVBufferRef counts are atomic. */
H264SPS *h264_sps_ref(H264SPS *sps, const char *why)
{
    if (sps)
        __atomic_add_fetch(&sps->refcount, 1, __ATOMIC_RELAXED);
    return sps;
}

void h264_sps_unref(H264SPS **psps, const char *why)
{
    H264SPS *sps = *psps;
    int n;

    if (!sps)
        return;
    n = __atomic_sub_fetch(&sps->refcount, 1, __ATOMIC_ACQ_REL);
    if (n < 0) {
        fprintf(stderr, "[h264dec] SPS refcount underflow (id %d, by %s)\n",
                sps->sps_id, why);
        *psps = NULL;
        return;
    }
    if (n == 0)
        free(sps);
    *psps = NULL;
}

H264PPS *h264_pps_ref(H264PPS *pps, const char *why)
{
    if (pps)
        __atomic_add_fetch(&pps->refcount, 1, __ATOMIC_RELAXED);
    return pps;
}

void h264_pps_unref(H264PPS **ppps, const char *why)
{
    H264PPS *pps = *ppps;
    int n;

    if (!pps)
        return;
    n = __atomic_sub_fetch(&pps->refcount, 1, __ATOMIC_ACQ_REL);
    if (n < 0) {
        fprintf(stderr, "[h264dec] PPS refcount underflow (id %d, by %s)\n",
                pps->pps_id, why);
        *ppps = NULL;
        return;
    }
    if (n == 0) {
        h264_sps_unref(&pps->sps, "pps link");
        free(pps);
    }
    *ppps = NULL;
}

static int decode_scaling_list(H264BitReader *br, uint8_t *factors, int size,
                               const uint8_t *jvt_list, const uint8_t *fallback_list,
                               uint16_t *mask, int pos)
{
    int i, last = 8, next = 8;
    const uint8_t *scan = size == 16 ? h264_zigzag_scan : h264_zigzag_direct;
    int present = (int)h264_br_get_bits1(br);

    *mask |= (uint16_t)(present << pos);
    if (!present) {
        memcpy(factors, fallback_list, size);
        return 0;
    }

    for (i = 0; i < size; i++) {
        if (next) {
            int v = h264_br_get_se(br);
            if (v < -128 || v > 127)
                return -1;
            next = (last + v) & 0xff;
        }
        if (!i && !next) {
            memcpy(factors, jvt_list, size);
            break;
        }
        last = factors[scan[i]] = next ? next : last;
    }
    return br->error ? -1 : 0;
}

static int decode_scaling_matrices(H264BitReader *br, const H264SPS *sps,
                                   const H264PPS *pps, int is_sps, int present_flag,
                                   uint16_t *mask,
                                   uint8_t (*scaling_matrix4)[16],
                                   uint8_t (*scaling_matrix8)[64])
{
    int fallback_sps = !is_sps && sps->scaling_matrix_present;
    const uint8_t *fallback[4] = {
        fallback_sps ? sps->scaling_matrix4[0] : default_scaling4[0],
        fallback_sps ? sps->scaling_matrix4[3] : default_scaling4[1],
        fallback_sps ? sps->scaling_matrix8[0] : default_scaling8[0],
        fallback_sps ? sps->scaling_matrix8[3] : default_scaling8[1]
    };
    int ret = 0;

    *mask = 0;
    if (!present_flag)
        return 0;

    ret |= decode_scaling_list(br, scaling_matrix4[0], 16, default_scaling4[0], fallback[0], mask, 0);
    ret |= decode_scaling_list(br, scaling_matrix4[1], 16, default_scaling4[0], scaling_matrix4[0], mask, 1);
    ret |= decode_scaling_list(br, scaling_matrix4[2], 16, default_scaling4[0], scaling_matrix4[1], mask, 2);
    ret |= decode_scaling_list(br, scaling_matrix4[3], 16, default_scaling4[1], fallback[1], mask, 3);
    ret |= decode_scaling_list(br, scaling_matrix4[4], 16, default_scaling4[1], scaling_matrix4[3], mask, 4);
    ret |= decode_scaling_list(br, scaling_matrix4[5], 16, default_scaling4[1], scaling_matrix4[4], mask, 5);

    if (is_sps || pps->transform_8x8_mode) {
        ret |= decode_scaling_list(br, scaling_matrix8[0], 64, default_scaling8[0], fallback[2], mask, 6);
        ret |= decode_scaling_list(br, scaling_matrix8[3], 64, default_scaling8[1], fallback[3], mask, 7);
        if (sps->chroma_format_idc == 3) {
            ret |= decode_scaling_list(br, scaling_matrix8[1], 64, default_scaling8[0], scaling_matrix8[0], mask,  8);
            ret |= decode_scaling_list(br, scaling_matrix8[4], 64, default_scaling8[1], scaling_matrix8[3], mask,  9);
            ret |= decode_scaling_list(br, scaling_matrix8[2], 64, default_scaling8[0], scaling_matrix8[1], mask, 10);
            ret |= decode_scaling_list(br, scaling_matrix8[5], 64, default_scaling8[1], scaling_matrix8[4], mask, 11);
        }
    }

    if (ret < 0 || br->error)
        return -1;
    return 1;
}

static int decode_hrd_parameters(H264BitReader *br, H264SPS *sps)
{
    int cpb_cnt = (int)h264_br_get_ue(br) + 1;
    int i;

    if (cpb_cnt > 32)
        return -1;

    h264_br_skip(br, 4); /* bit_rate_scale */
    h264_br_skip(br, 4); /* cpb_size_scale */
    for (i = 0; i < cpb_cnt; i++) {
        h264_br_get_ue(br); /* bit_rate_value_minus1 */
        h264_br_get_ue(br); /* cpb_size_value_minus1 */
        h264_br_get_bits1(br); /* cbr_flag */
    }
    h264_br_skip(br, 5); /* initial_cpb_removal_delay_length_minus1 */
    /* the picture timing SEI reads its delays back with these widths */
    sps->cpb_removal_delay_length = (int)h264_br_get_bits(br, 5) + 1;
    sps->dpb_output_delay_length = (int)h264_br_get_bits(br, 5) + 1;
    sps->time_offset_length = (int)h264_br_get_bits(br, 5);
    return br->error ? -1 : 0;
}

static int decode_vui(H264BitReader *br, H264SPS *sps)
{
    int aspect_ratio_info_present = (int)h264_br_get_bits1(br);

    if (aspect_ratio_info_present) {
        int idc = (int)h264_br_get_bits(br, 8);
        if (idc < 17) {
            sps->sar_num = pixel_aspect[idc][0];
            sps->sar_den = pixel_aspect[idc][1];
        } else if (idc == 255) {
            sps->sar_num = (int)h264_br_get_bits(br, 16);
            sps->sar_den = (int)h264_br_get_bits(br, 16);
            if (!sps->sar_num || !sps->sar_den) {
                sps->sar_num = 0;
                sps->sar_den = 1;
            }
        }
    }

    if (h264_br_get_bits1(br))  /* overscan_info_present_flag */
        h264_br_get_bits1(br);  /* overscan_appropriate_flag */

    if (h264_br_get_bits1(br)) { /* video_signal_type_present_flag */
        h264_br_skip(br, 3);     /* video_format */
        h264_br_get_bits1(br);   /* video_full_range_flag */
        if (h264_br_get_bits1(br)) /* colour_description_present_flag */
            h264_br_skip(br, 24);
    }

    if (h264_br_get_bits1(br)) { /* chroma_loc_info_present_flag */
        h264_br_get_ue(br);
        h264_br_get_ue(br);
    }

    sps->timing_info_present_flag = (int)h264_br_get_bits1(br);
    if (sps->timing_info_present_flag) {
        sps->num_units_in_tick = h264_br_get_bits(br, 32);
        sps->time_scale = h264_br_get_bits(br, 32);
        sps->fixed_frame_rate_flag = (int)h264_br_get_bits1(br);
        if (!sps->num_units_in_tick || !sps->time_scale)
            sps->timing_info_present_flag = 0;
    }

    sps->nal_hrd_parameters_present_flag = (int)h264_br_get_bits1(br);
    if (sps->nal_hrd_parameters_present_flag) {
        if (decode_hrd_parameters(br, sps) < 0)
            return -1;
    }
    sps->vcl_hrd_parameters_present_flag = (int)h264_br_get_bits1(br);
    if (sps->vcl_hrd_parameters_present_flag) {
        if (decode_hrd_parameters(br, sps) < 0)
            return -1;
    }
    /* low_delay_hrd_flag sits between the HRD parameters and the pic_struct
     * flag, so leaving it out would shift every bit read after it */
    if (sps->nal_hrd_parameters_present_flag ||
        sps->vcl_hrd_parameters_present_flag)
        h264_br_get_bits1(br); /* low_delay_hrd_flag */

    sps->pic_struct_present_flag = (int)h264_br_get_bits1(br);

    if (h264_br_bits_left(br) <= 0)
        return 0;

    sps->bitstream_restriction_flag = (int)h264_br_get_bits1(br);
    if (sps->bitstream_restriction_flag) {
        h264_br_get_bits1(br);   /* motion_vectors_over_pic_boundaries_flag */
        h264_br_get_ue(br);      /* max_bytes_per_pic_denom */
        h264_br_get_ue(br);      /* max_bits_per_mb_denom */
        h264_br_get_ue(br);      /* log2_max_mv_length_horizontal */
        h264_br_get_ue(br);      /* log2_max_mv_length_vertical */
        sps->num_reorder_frames = (int)h264_br_get_ue(br);
        sps->max_dec_frame_buffering = (int)h264_br_get_ue(br);
        if (sps->num_reorder_frames > 16)
            sps->num_reorder_frames = 16;
        if (sps->max_dec_frame_buffering > 16)
            sps->max_dec_frame_buffering = 16;
    }

    return br->error ? -1 : 0;
}

static void compute_vui_defaults(H264SPS *sps)
{
    if (sps->sar_den == 0)
        sps->sar_den = 1;
    if (!sps->nal_hrd_parameters_present_flag &&
        !sps->vcl_hrd_parameters_present_flag)
        sps->time_offset_length = 24;   /* what the reference assumes, and the
                                           picture timing SEI is written with
                                           this width when no HRD is signalled */
}

int h264_decode_sps(H264Decoder *h, const uint8_t *rbsp, int size_bits, int nal_type)
{
    H264BitReader br;
    H264SPS *sps;
    int profile_idc, level_idc;
    int sps_id;
    int i;
    int ret = -1;
    (void)nal_type;

    h264_br_init(&br, rbsp, size_bits);

    sps = (H264SPS *)calloc(1, sizeof(*sps));
    if (!sps)
        return -1;
    sps->refcount = 1;

    profile_idc = (int)h264_br_get_bits(&br, 8);
    sps->constraint_set_flags =
        (int)h264_br_get_bits(&br, 6); /* constraint_set0..5 */
    h264_br_skip(&br, 2);
    level_idc = (int)h264_br_get_bits(&br, 8);
    sps_id = (int)h264_br_get_ue(&br);

    if (sps_id >= H264_MAX_SPS_COUNT)
        goto fail;

    sps->sps_id = sps_id;
    sps->profile_idc = profile_idc;
    sps->level_idc = level_idc;
    sps->chroma_format_idc = 1;
    sps->bit_depth_luma = 8;
    sps->bit_depth_chroma = 8;

    memset(sps->scaling_matrix4, 16, sizeof(sps->scaling_matrix4));
    memset(sps->scaling_matrix8, 16, sizeof(sps->scaling_matrix8));

    if (profile_idc == 100 || profile_idc == 110 || profile_idc == 122 ||
        profile_idc == 244 || profile_idc ==  44 || profile_idc ==  83 ||
        profile_idc ==  86 || profile_idc == 118 || profile_idc == 128 ||
        profile_idc == 138 || profile_idc == 144) {
        sps->chroma_format_idc = (int)h264_br_get_ue(&br);
        if (sps->chroma_format_idc > 3)
            goto fail;
        if (sps->chroma_format_idc == 3) {
            int separate_colour_plane = (int)h264_br_get_bits1(&br);
            if (separate_colour_plane)
                goto fail;
        }
        sps->bit_depth_luma = (int)h264_br_get_ue(&br) + 8;
        sps->bit_depth_chroma = (int)h264_br_get_ue(&br) + 8;
        if (sps->bit_depth_luma != sps->bit_depth_chroma ||
            (sps->bit_depth_luma != 8 && sps->bit_depth_luma != 10))
            goto fail;
        sps->transform_bypass = (int)h264_br_get_bits1(&br);
        {
            int present = (int)h264_br_get_bits1(&br);
            uint16_t mask;
            int r = decode_scaling_matrices(&br, sps, NULL, 1, present, &mask,
                                            sps->scaling_matrix4, sps->scaling_matrix8);
            if (r < 0)
                goto fail;
            sps->scaling_matrix_present = r;
        }
    }

    if ((sps->bit_depth_luma != 8 && sps->bit_depth_luma != 10) ||
        (sps->chroma_format_idc != 1 && sps->bit_depth_luma != 10))
        goto fail;

    sps->log2_max_frame_num = (int)h264_br_get_ue(&br) + 4;
    if (sps->log2_max_frame_num < 4 || sps->log2_max_frame_num > 16)
        goto fail;

    sps->poc_type = (int)h264_br_get_ue(&br);
    if (sps->poc_type == 0) {
        sps->log2_max_poc_lsb = (int)h264_br_get_ue(&br) + 4;
        if (sps->log2_max_poc_lsb > 16)
            goto fail;
    } else if (sps->poc_type == 1) {
        sps->delta_pic_order_always_zero_flag = (int)h264_br_get_bits1(&br);
        sps->offset_for_non_ref_pic = h264_br_get_se(&br);
        sps->offset_for_top_to_bottom_field = h264_br_get_se(&br);
        sps->poc_cycle_length = (int)h264_br_get_ue(&br);
        if ((unsigned)sps->poc_cycle_length >= H264_ARRAY_ELEMS(sps->offset_for_ref_frame))
            goto fail;
        for (i = 0; i < sps->poc_cycle_length; i++)
            sps->offset_for_ref_frame[i] = h264_br_get_se(&br);
    } else if (sps->poc_type != 2) {
        goto fail;
    }

    sps->ref_frame_count = (int)h264_br_get_ue(&br);
    if (sps->ref_frame_count > H264_MAX_REF_FRAMES)
        goto fail;
    sps->gaps_in_frame_num_allowed_flag = (int)h264_br_get_bits1(&br);
    sps->mb_width = (int)h264_br_get_ue(&br) + 1;
    sps->mb_height = (int)h264_br_get_ue(&br) + 1;
    if (sps->mb_width <= 0 || sps->mb_height <= 0 ||
        sps->mb_width > 512 || sps->mb_height > 512)
        goto fail;

    sps->frame_mbs_only_flag = (int)h264_br_get_bits1(&br);
    sps->mb_height *= 2 - sps->frame_mbs_only_flag;
    sps->mb_aff = sps->frame_mbs_only_flag ? 0 : (int)h264_br_get_bits1(&br);
    sps->direct_8x8_inference_flag = (int)h264_br_get_bits1(&br);

    sps->crop = (int)h264_br_get_bits1(&br);
    if (sps->crop) {
        unsigned crop_left = h264_br_get_ue(&br);
        unsigned crop_right = h264_br_get_ue(&br);
        unsigned crop_top = h264_br_get_ue(&br);
        unsigned crop_bottom = h264_br_get_ue(&br);
        int width = 16 * sps->mb_width;
        int height = 16 * sps->mb_height;
        const int vsub = sps->chroma_format_idc == 1;
        const int hsub = sps->chroma_format_idc == 1 || sps->chroma_format_idc == 2;
        const int step_x = 1 << hsub;
        const int step_y = (2 - sps->frame_mbs_only_flag) << vsub;
        /* sums in 64 bits: the individual values come straight from ue(v) and
         * can be large enough that an unsigned 32-bit sum wraps and passes the
         * check, after which the per-side multiplication would overflow int */
        const uint64_t crop_x = (uint64_t)crop_left + crop_right;
        const uint64_t crop_y = (uint64_t)crop_top + crop_bottom;

        if (crop_x * step_x >= (uint64_t)width ||
            crop_y * step_y >= (uint64_t)height)
            goto fail;
        sps->crop_left = (int)crop_left * step_x;
        sps->crop_right = (int)crop_right * step_x;
        sps->crop_top = (int)crop_top * step_y;
        sps->crop_bottom = (int)crop_bottom * step_y;
    }

    sps->vui_parameters_present_flag = (int)h264_br_get_bits1(&br);
    if (sps->vui_parameters_present_flag) {
        if (decode_vui(&br, sps) < 0)
            goto fail;
    }
    compute_vui_defaults(sps);

    if (br.error)
        goto fail;

    if (!sps->bitstream_restriction_flag) {
        sps->num_reorder_frames = H264_MAX_REF_FRAMES - 1;
        sps->max_dec_frame_buffering = H264_MAX_REF_FRAMES;
        if (sps->ref_frame_count) {
            for (i = 0; i < (int)H264_ARRAY_ELEMS(level_max_dpb_mbs); i++) {
                if (level_max_dpb_mbs[i][0] == sps->level_idc) {
                    int v = level_max_dpb_mbs[i][1] / (sps->mb_width * sps->mb_height);
                    sps->num_reorder_frames = H264_MIN(v, sps->num_reorder_frames);
                    sps->max_dec_frame_buffering = H264_MIN(v, sps->max_dec_frame_buffering);
                    break;
                }
            }
        }
    }

    sps->valid = 1;
    h264_sps_unref(&h->sps_list[sps_id], "sps replace");
    h->sps_list[sps_id] = sps;
    return 0;

fail:
    fprintf(stderr, "[h264dec] unsupported or invalid SPS (profile %d, chroma %d, depth %d)\n",
            profile_idc, sps->chroma_format_idc, sps->bit_depth_luma);
    h264_sps_unref(&sps, "sps fail");
    return ret;
}

static void init_dequant4_coeff_table(H264PPS *pps)
{
    int i, q, x;
    const int depth = pps->sps ? pps->sps->bit_depth_luma : 8;
    const int max_qp = 51 + 6 * (depth - 8);

    for (i = 0; i < 6; i++) {
        for (q = 0; q <= max_qp; q++) {
            int shift = h264_quant_div6[q] + 2;
            int idx = h264_quant_rem6[q];
            for (x = 0; x < 16; x++)
                pps->dequant4_coeff[i][q][(x >> 2) | ((x << 2) & 0xF)] =
                    ((uint32_t)h264_dequant4_coeff_init[idx][(x & 1) + ((x >> 2) & 1)] *
                     pps->scaling_matrix4[i][x]) << shift;
        }
    }
}

static void init_dequant8_coeff_table(H264PPS *pps)
{
    int i, q, x;
    const int depth = pps->sps ? pps->sps->bit_depth_luma : 8;
    const int max_qp = 51 + 6 * (depth - 8);

    for (i = 0; i < 6; i++) {
        for (q = 0; q <= max_qp; q++) {
            int shift = h264_quant_div6[q];
            int idx = h264_quant_rem6[q];
            for (x = 0; x < 64; x++)
                pps->dequant8_coeff[i][q][(x >> 3) | ((x & 7) << 3)] =
                    ((uint32_t)h264_dequant8_coeff_init[idx][h264_dequant8_coeff_init_scan[((x >> 1) & 12) | (x & 3)]] *
                     pps->scaling_matrix8[i][x]) << shift;
        }
    }
}

void h264_pps_init_dequant(H264PPS *pps)
{
    init_dequant4_coeff_table(pps);
    if (pps->transform_8x8_mode)
        init_dequant8_coeff_table(pps);

    if (pps->sps && pps->sps->transform_bypass) {
        int i, x;
        for (i = 0; i < 6; i++)
            for (x = 0; x < 16; x++)
                pps->dequant4_coeff[i][0][x] = 1 << 6;
        if (pps->transform_8x8_mode)
            for (i = 0; i < 6; i++)
                for (x = 0; x < 64; x++)
                    pps->dequant8_coeff[i][0][x] = 1 << 6;
    }
}

static void build_qp_table(H264PPS *pps, int t, int index)
{
    const int depth = pps->sps ? pps->sps->bit_depth_luma : 8;
    const int max_qp = 51 + 6 * (depth - 8);
    int i;
    for (i = 0; i <= max_qp; i++)
        pps->chroma_qp_table[t][i] =
            h264_chroma_qp[depth - 8][H264_CLIP(i + index, 0, max_qp)];
}

int h264_decode_pps(H264Decoder *h, const uint8_t *rbsp, int size_bits)
{
    H264BitReader br;
    H264PPS *pps;
    int pps_id, sps_id;
    int ret = -1;

    h264_br_init(&br, rbsp, size_bits);

    pps_id = (int)h264_br_get_ue(&br);
    if (pps_id >= H264_MAX_PPS_COUNT)
        return -1;
    sps_id = (int)h264_br_get_ue(&br);
    if (sps_id >= H264_MAX_SPS_COUNT || !h->sps_list[sps_id]) {
        fprintf(stderr, "[h264dec] PPS references missing SPS %d\n", sps_id);
        return -1;
    }

    pps = (H264PPS *)calloc(1, sizeof(*pps));
    if (!pps)
        return -1;
    pps->refcount = 1;
    pps->pps_id = pps_id;
    pps->sps_id = sps_id;
    pps->sps = h264_sps_ref(h->sps_list[sps_id], "pps link");

    pps->cabac = (int)h264_br_get_bits1(&br);
    pps->pic_order_present = (int)h264_br_get_bits1(&br);
    pps->slice_group_count = (int)h264_br_get_ue(&br) + 1;
    if (pps->slice_group_count > 1) {
        fprintf(stderr, "[h264dec] FMO (slice groups) is not supported\n");
        goto fail;
    }
    pps->ref_count[0] = (int)h264_br_get_ue(&br) + 1;
    pps->ref_count[1] = (int)h264_br_get_ue(&br) + 1;
    if (pps->ref_count[0] > 32 || pps->ref_count[1] > 32)
        goto fail;

    pps->weighted_pred = (int)h264_br_get_bits1(&br);
    pps->weighted_bipred_idc = (int)h264_br_get_bits(&br, 2);
    pps->init_qp = (int)h264_br_get_se(&br) + 26 +
                   6 * (pps->sps->bit_depth_luma - 8);
    pps->init_qs = (int)h264_br_get_se(&br) + 26 +
                   6 * (pps->sps->bit_depth_luma - 8);
    pps->chroma_qp_index_offset[0] = h264_br_get_se(&br);
    if (pps->chroma_qp_index_offset[0] < -12 || pps->chroma_qp_index_offset[0] > 12)
        goto fail;

    pps->deblocking_filter_parameters_present = (int)h264_br_get_bits1(&br);
    pps->constrained_intra_pred = (int)h264_br_get_bits1(&br);
    pps->redundant_pic_cnt_present = (int)h264_br_get_bits1(&br);

    pps->transform_8x8_mode = 0;
    memcpy(pps->scaling_matrix4, pps->sps->scaling_matrix4, sizeof(pps->scaling_matrix4));
    memcpy(pps->scaling_matrix8, pps->sps->scaling_matrix8, sizeof(pps->scaling_matrix8));

    if (h264_br_more_rbsp_data(&br)) {
        int present;
        uint16_t mask;
        pps->transform_8x8_mode = (int)h264_br_get_bits1(&br);
        present = (int)h264_br_get_bits1(&br);
        if (decode_scaling_matrices(&br, pps->sps, pps, 0, present, &mask,
                                    pps->scaling_matrix4, pps->scaling_matrix8) < 0)
            goto fail;
        pps->chroma_qp_index_offset[1] = h264_br_get_se(&br);
        if (pps->chroma_qp_index_offset[1] < -12 || pps->chroma_qp_index_offset[1] > 12)
            goto fail;
    } else {
        pps->chroma_qp_index_offset[1] = pps->chroma_qp_index_offset[0];
    }

    if (br.error)
        goto fail;

    if (pps->init_qp > 51 + 6 * (pps->sps->bit_depth_luma - 8) || pps->init_qp < 0)
        pps->init_qp = H264_CLIP(pps->init_qp, 0,
                                 51 + 6 * (pps->sps->bit_depth_luma - 8));

    build_qp_table(pps, 0, pps->chroma_qp_index_offset[0]);
    build_qp_table(pps, 1, pps->chroma_qp_index_offset[1]);
    h264_pps_init_dequant(pps);
    pps->chroma_qp_diff = pps->chroma_qp_index_offset[0] != pps->chroma_qp_index_offset[1];

    pps->valid = 1;
    h264_pps_unref(&h->pps_list[pps_id], "pps replace");
    h->pps_list[pps_id] = pps;
    return 0;

fail:
    fprintf(stderr, "[h264dec] invalid PPS\n");
    h264_pps_unref(&pps, "pps fail");
    return ret;
}

