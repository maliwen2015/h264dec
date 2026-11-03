/*
 * Public decoder API and NAL unit dispatch.
 */
#include "h264_internal.h"

/* Slice objects carry a large per-slice cache set (34 KB); recycling them (and
 * their RBSP buffer) keeps a one-slice-per-picture stream from doing a
 * calloc/free of that plus three more allocations for every picture. The cap
 * bounds what an unusual many-slice stream can retain, and an oversized RBSP is
 * dropped on return so a single huge NAL does not pin its buffer forever. */
#define H264_SLICE_POOL_MAX 64
#define H264_SLICE_RBSP_KEEP (128 * 1024)

static void slice_release(H264Slice *sl)
{
    H264Decoder *h;

    if (!sl)
        return;
    h264_sps_unref(&sl->sps, "slice");
    h264_pps_unref(&sl->pps, "slice");
    h = sl->h;
    if (h) {
        h264_mutex_lock(&h->slice_pool_m);
        if (h->slice_pool_count < H264_SLICE_POOL_MAX) {
            if (sl->rbsp_alloc > H264_SLICE_RBSP_KEEP) {
                free(sl->rbsp);
                sl->rbsp = NULL;
                sl->rbsp_alloc = 0;
            }
            sl->next_free = h->slice_free;
            h->slice_free = sl;
            h->slice_pool_count++;
            h264_mutex_unlock(&h->slice_pool_m);
            return;
        }
        h264_mutex_unlock(&h->slice_pool_m);
    }
    free(sl->rbsp);
    free(sl);
}

static H264Slice *slice_get(H264Decoder *h)
{
    H264Slice *sl = NULL;

    h264_mutex_lock(&h->slice_pool_m);
    if (h->slice_free) {
        sl = h->slice_free;
        h->slice_free = sl->next_free;
        h->slice_pool_count--;
    }
    h264_mutex_unlock(&h->slice_pool_m);
    if (sl) {
        uint8_t *rbsp = sl->rbsp;
        const int rbsp_alloc = sl->rbsp_alloc;
        memset(sl, 0, sizeof(*sl));
        sl->rbsp = rbsp;
        sl->rbsp_alloc = rbsp_alloc;
    } else {
        sl = (H264Slice *)calloc(1, sizeof(*sl));
        if (!sl)
            return NULL;
    }
    sl->h = h;
    return sl;
}

static void slice_pool_free(H264Decoder *h)
{
    H264Slice *sl = h->slice_free;

    while (sl) {
        H264Slice *next = sl->next_free;
        free(sl->rbsp);
        free(sl);
        sl = next;
    }
    h->slice_free = NULL;
    h->slice_pool_count = 0;
}

/* One scratch buffer for the parameter-set and first_mb peeks; those parses
 * happen on the decoder owner, one at a time, and never retain the bytes. */
uint8_t *h264_nal_scratch(H264Decoder *h, int need)
{
    if (h->nal_buf_alloc < need) {
        uint8_t *p = (uint8_t *)realloc(h->nal_buf, (size_t)need);
        if (!p)
            return NULL;
        h->nal_buf = p;
        h->nal_buf_alloc = need;
    }
    return h->nal_buf;
}

static void free_au_slices(H264Decoder *h)
{
    int i;
    for (i = 0; i < h->nb_slices; i++) {
        slice_release(h->slices[i]);
        h->slices[i] = NULL;
    }
    h->nb_slices = 0;
}

static void release_job_slices(H264Job *job)
{
    int i;
    for (i = 0; i < job->nb_slices; i++) {
        slice_release(job->slices[i]);
        job->slices[i] = NULL;
    }
    job->nb_slices = 0;
}

/* ------------------------------------------------------------------ */
/* picture job for frame-level threading                               */
/* ------------------------------------------------------------------ */
/* A picture keeps its reference pictures alive while its pixels are being
 * decoded, because the reference picture marking of the next pictures is
 * already applied by then. */
static void job_refs(H264Decoder *h, H264Job *job, int take)
{
    const H264Frame *seen[H264_MAX_PICTURE_COUNT];
    int n = 0, i, list, r, j;

    for (i = 0; i < job->nb_slices; i++) {
        H264Slice *sl = job->slices[i];
        for (list = 0; list < 2; list++)
            for (r = 0; r < H264_MAX_LIST_SIZE; r++) {
                const H264Frame *p = sl->ref_list[list][r].parent;
                if (!p)
                    continue;
                for (j = 0; j < n; j++)
                    if (seen[j] == p)
                        break;
                if (j < n)
                    continue;
                if (n < (int)(sizeof(seen) / sizeof(seen[0])))
                    seen[n++] = p;
                /* only the lifetime counter changes, never the picture */
                if (take)
                    h264_frame_ref((H264Frame *)p);
                else
                    h264_frame_unref(h, (H264Frame *)p);
            }
    }
}

int h264_run_job(H264Decoder *cl, H264Job *job)
{
    H264Frame *f = job->f;
    int i, ret = 0;

    if (job->scratch_failed) {
        cl->error = 1;
        ret = -1;
    } else {
        for (i = 0; i < job->nb_slices; i++) {
            H264Slice *sl = job->slices[i];
            int r = h264_decode_slice_data(cl, sl);
            if (r < 0) {
                cl->error = 1;
                ret = -1;
                break;
            }
        }
    }
    /* the reference pictures are no longer needed once the reconstruction is
     * done, so release them before waiting for the commit turn: the pool has
     * to serve the pictures that are in flight as well */
    h264_decoder_lock(cl);
    job_refs(cl, job, 0);
    h264_decoder_unlock(cl);
    h264_picture_pixels(cl, f, job->structure);
    return ret;
}

void h264_commit_job(H264Decoder *h, H264Job *job)
{
    if (job->result < 0)
        h->au_error = 1;
    h264_picture_output(h, job->f, job->structure);
    release_job_slices(job);
}

static int dispatch_access_unit(H264Decoder *h)
{
    H264Job job;
    int i, run_inline;

    memset(&job, 0, sizeof(job));
    if (!h->cur_pic) {
        free_au_slices(h);
        return -1;
    }
    job.f = h->cur_pic;
    job.structure = h->cur_pic->field_picture ? h->picture_structure : PICT_FRAME;
    job.nb_slices = h->nb_slices;
    for (i = 0; i < h->nb_slices; i++)
        job.slices[i] = h->slices[i];
    h->nb_slices = 0;

    /* field pictures are not pipelined: the two fields of a frame share one
     * picture, so the second one has to wait for the first */
    run_inline = !h->threaded || h->cur_pic->field_picture;
    if (run_inline && h->threaded)
        h264_job_wait_all(h);
    if (!run_inline) {
        H264Job *slot = h264_job_acquire(h);
        const int state = slot->state;
        const int64_t seq = slot->seq;
        *slot = job;
        slot->state = state;
        slot->seq = seq;
        /* rows of this picture's loop filter go on the shared list instead of
         * being done here; with the filter switched off there is nothing to
         * hand over, and the row still has to be published for the waiters */
        h->cur_pic->filter_split = !h->param.disable_lf;
        /* hold the pictures this job reads before the marking of this picture
         * can release them */
        job_refs(h, slot, 1);
        h264_picture_headers_end(h);
        /* the job owns the picture from here on; the commit queues its output */
        h->cur_pic = NULL;
        h->first_field = 0;
        h264_job_submit(h, slot);
        return 0;
    }

    job_refs(h, &job, 1);
    h264_picture_headers_end(h);
    h264_decoder_unlock(h);      /* pixel work never holds the pool lock */
    job.result = h264_run_job(h, &job);
    h264_decoder_lock(h);
    h264_commit_job(h, &job);
    return job.result;
}

static int process_access_unit(H264Decoder *h)
{
    int i, ret = 0;

    if (!h->nb_slices)
        return 0;

    h264_decoder_lock(h);
    if (h264_activate_ps(h, h->slices[0]) < 0) {
        h->error = 1;
        ret = -1;
        goto done;
    }
    if (h264_field_start(h, h->slices[0]) < 0) {
        h->error = 1;
        ret = -1;
        goto done;
    }
    for (i = 0; i < h->nb_slices; i++) {
        H264Slice *sl = h->slices[i];
        if (h264_slice_init(h, sl) < 0) {
            h->error = 1;
            ret = -1;
            break;
        }
    }
    if (ret == 0) {
        for (i = 0; i < h->nb_slices; i++) {
            H264Slice *sl = h->slices[i];
            int best = h->mb_width * h->mb_height;
            int j;
            int my_idx = sl->mb_x + sl->mb_y * h->mb_width;
            for (j = 0; j < h->nb_slices; j++) {
                int idx = h->slices[j]->mb_x + h->slices[j]->mb_y * h->mb_width;
                if (j != i && idx > my_idx && idx < best)
                    best = idx;
            }
            sl->next_slice_idx = best;
        }
        ret = dispatch_access_unit(h);
    } else if (h->cur_pic) {
        /* the picture has a frame buffer but no decodable slices: still do the
         * bookkeeping so that the DPB and the output order stay in step */
        int structure = h->cur_pic->field_picture ? h->picture_structure : PICT_FRAME;
        h264_picture_headers_end(h);
        h264_decoder_unlock(h);
        h264_picture_pixels(h, h->cur_pic, structure);
        h264_decoder_lock(h);
        h264_picture_output(h, h->cur_pic, structure);
    }
done:
    free_au_slices(h);
    h->cur_first_mb = 0;
    h264_decoder_unlock(h);
    return ret;
}

static int parse_nal_into_slice(H264Decoder *h, const uint8_t *nal, int len,
                                int nal_type, int ref_idc, H264Slice **out)
{
    H264Slice *sl;
    uint8_t *rbsp;
    int rbsp_size;
    H264NAL info;

    if (len < 1)
        return -1;
    sl = slice_get(h);
    if (!sl)
        return -1;
    if (sl->rbsp_alloc < len + 16) {
        uint8_t *nb = (uint8_t *)realloc(sl->rbsp, (size_t)len + 16);
        if (!nb) {
            slice_release(sl);
            return -1;
        }
        sl->rbsp = nb;
        sl->rbsp_alloc = len + 16;
    }
    rbsp = sl->rbsp;
    rbsp_size = h264_unescape_rbsp(rbsp, len + 16, nal + 1, len - 1);
    if (rbsp_size < 0) {
        slice_release(sl);
        return -1;
    }
    sl->rbsp_size = rbsp_size;
    h264_br_init(&sl->gb, rbsp, rbsp_size * 8);

    info.type = nal_type;
    info.ref_idc = ref_idc;
    info.data = nal;
    info.size = len;
    if (h264_decode_slice_header(h, sl, &info, ref_idc, nal_type) < 0) {
        fprintf(stderr, "[h264dec] invalid slice header (type %d)\n", nal_type);
        slice_release(sl);
        return -1;
    }
    if (sl->redundant_pic_count > 0) {
        /* a redundant picture is an alternative decoding of the same macroblock
         * area; the primary one wins, so drop this */
        slice_release(sl);
        return 1;
    }
    *out = sl;
    return 0;
}

void* h264_dec_open(h264_param_t* param)
{
    H264Decoder *h;

    h264_cavlc_init();

    h = (H264Decoder *)calloc(1, sizeof(H264Decoder));
    if (!h)
        return NULL;
    if (param)
        h->param = *param;
    if (h->param.threads < 0)
        h->param.threads = 0;
    /* 0 is Annex-B, 1..4 are length-prefixed units; anything else would make
     * the splitter read a length it cannot represent */
    if (h->param.nal_length_size < 0 || h->param.nal_length_size > 4) {
        free(h);
        return NULL;
    }
    h->info.frame_rate_den = 1;
    h->max_reorder = 0;
    h->x264_build = -1;
    h264_sei_reset(h);
    h->poc.prev_poc_msb = 1 << 16;
    h->poc.prev_poc_lsb = -1;
    h->poc.prev_frame_num = 0;
    h264_mutex_init(&h->mutex);
    h264_cond_init(&h->cond);
    h->mtx = &h->mutex;
    h->cv = &h->cond;
    h264_mutex_init(&h->filter_q.m);
    h264_cond_init(&h->filter_q.claim_c);
    h264_mutex_init(&h->slice_pool_m);
    h->fp = &h->filter_q;
    if (h264_thread_init(h, h->param.threads) < 0) {
        h264_mutex_destroy(&h->slice_pool_m);
        h264_mutex_destroy(&h->filter_q.m);
        h264_cond_destroy(&h->filter_q.claim_c);
        h264_cond_destroy(&h->cond);
        h264_mutex_destroy(&h->mutex);
        free(h);
        return NULL;
    }
    return h;
}

static int append_slice(H264Decoder *h, H264Slice *sl)
{
    if (h->nb_slices >= H264_MAX_SLICES) {
        fprintf(stderr, "[h264dec] too many slices\n");
        return -1;
    }
    h->slices[h->nb_slices++] = sl;
    return 0;
}

int h264_dec_send_packet(void* decoder, h264_packet_t* packet)
{
    H264Decoder *h = (H264Decoder *)decoder;
    int pos = 0;
    const uint8_t *nal;
    int len;
    int ret = 0;

    if (!h || !packet || !packet->data || packet->len <= 0)
        return H264_DEC_ERROR;
    h->packet_pts = packet->pts;
    h->packet_dts = packet->dts;
    h->error = 0;
    /* a picture timing or recovery message describes the unit it sits in, so
     * do not let it carry over to the next one */
    h264_sei_reset(h);

    while (h->param.nal_length_size
               ? h264_split_avcc(packet->data, packet->len, &pos, &nal, &len,
                                 h->param.nal_length_size)
               : h264_split_annexb(packet->data, packet->len, &pos, &nal, &len)) {
        int type, ref_idc;
        if (len < 1)
            continue;
        type = nal[0] & 0x1f;
        ref_idc = (nal[0] >> 5) & 3;

        if (type == 7 || type == 8) {
            uint8_t *rbsp = h264_nal_scratch(h, len + 16);
            int n;
            if (!rbsp) {
                ret = H264_DEC_ERROR;
                break;
            }
            n = h264_unescape_rbsp(rbsp, len + 16, nal + 1, len - 1);
            if (n < 0 || (type == 7 ? h264_decode_sps(h, rbsp, n * 8, type)
                                    : h264_decode_pps(h, rbsp, n * 8)) < 0) {
                ret = H264_DEC_ERROR;
                break;
            }
        } else if (type == 6) {
            h264_decode_sei(h, nal, len);
        } else if (type == 9) {
            if (h->nb_slices) {
                int r = process_access_unit(h);
                if (r < 0)
                    ret = H264_DEC_ERROR;
            }
        } else if (type == 1 || type == 5) {
            H264Slice *sl = NULL;
            unsigned first_mb;
            H264BitReader tmp;
            uint8_t *rbsp = h264_nal_scratch(h, len + 16);
            int n;
            if (!rbsp) {
                ret = H264_DEC_ERROR;
                break;
            }
            n = h264_unescape_rbsp(rbsp, len + 16, nal + 1, len - 1);
            if (n < 0) {
                ret = H264_DEC_ERROR;
                break;
            }
            h264_br_init(&tmp, rbsp, n * 8);
            first_mb = h264_br_get_ue(&tmp);

            if (first_mb == 0 && h->nb_slices) {
                if (process_access_unit(h) < 0)
                    ret = H264_DEC_ERROR;
            }
            {
                int pr = parse_nal_into_slice(h, nal, len, type, ref_idc, &sl);
                if (pr < 0) {
                    ret = H264_DEC_ERROR;
                    break;
                }
                if (pr > 0)
                    continue;           /* redundant slice, already released */
            }
            if (append_slice(h, sl) < 0) {
                slice_release(sl);
                ret = H264_DEC_ERROR;
                break;
            }
        } else if (type >= 2 && type <= 4) {
            fprintf(stderr, "[h264dec] data partitioning is not supported\n");
            ret = H264_DEC_ERROR;
            break;
        }
    }

    if (ret == 0 && h->nb_slices) {
        if (process_access_unit(h) < 0)
            ret = H264_DEC_ERROR;
    }
    h->eof = 0;
    /* a picture that failed on a worker is only known once it commits; the
     * per picture flag b_decode_error stays the authoritative signal */
    if (ret == 0 && h->au_error)
        ret = H264_DEC_ERROR;
    h->au_error = 0;
    return ret;
}

static void fill_picture(H264Decoder *h, H264Frame *f, h264_picture_t *pic)
{
    const int cx = f->crop_left;
    const int cy = f->crop_top;
    int i;

    memset(pic, 0, sizeof(*pic));
    pic->planes[0] = f->data[0] + cy * f->linesize[0] + cx * (f->bit_depth > 8 ? 2 : 1);
    pic->widths[0] = f->disp_width;
    pic->lines[0] = f->disp_height;
    if (f->chroma_format_idc) {
        const int bytes = f->bit_depth > 8 ? 2 : 1;
        const int sx = f->chroma_shift_x, sy = f->chroma_shift_y;
        pic->planes[1] = f->data[1] + (cy >> sy) * f->linesize[1] + (cx >> sx) * bytes;
        pic->planes[2] = f->data[2] + (cy >> sy) * f->linesize[2] + (cx >> sx) * bytes;
        pic->widths[1] = pic->widths[2] = f->disp_width >> sx;
        pic->lines[1] = pic->lines[2] = f->disp_height >> sy;
        pic->num_planes = 3;
    } else {
        pic->num_planes = 1;
    }
    for (i = 0; i < 3; i++)
        pic->strides[i] = f->linesize[i];
    pic->pic_order_count = f->poc;
    pic->type = f->type == H264_SLICE_I ? 1 : f->type == H264_SLICE_P ? 2 : 3;
    pic->qp = f->qp;
    pic->pts = f->pts;
    pic->dts = f->dts;
    pic->bytes_per_sample = f->bit_depth > 8 ? 2 : 1;
    pic->bit_depth = f->bit_depth;
    pic->b_decode_error = f->b_decode_error;
    pic->top_field_first = f->top_field_first;
    pic->interlaced = f->interlaced;
    pic->repeat_pict = f->repeat_pict;
    pic->pic_struct = f->pt_present ? f->pt.pic_struct : -1;
    pic->cpb_removal_delay = f->pt_present ? f->pt.cpb_removal_delay : -1;
    pic->dpb_output_delay = f->pt_present ? f->pt.dpb_output_delay : 0;
    pic->recovery_frame_cnt = f->recovery_frame_cnt;
    pic->broken_recovery = f->broken_recovery;
    pic->timecode_valid = f->pt_present && f->pt.tc_valid;
    pic->timecode_hours = f->pt.tc_hours;
    pic->timecode_minutes = f->pt.tc_minutes;
    pic->timecode_seconds = f->pt.tc_seconds;
    pic->timecode_frames = f->pt.tc_frames;
    pic->timecode_drop = f->pt.tc_drop;
    pic->dec_frame = f;
    pic->opaque = NULL;
}

int h264_dec_recv_frame(void* decoder, h264_stream_info* headerset, h264_picture_t* out_frame)
{
    H264Decoder *h = (H264Decoder *)decoder;
    H264Frame *f;

    if (!h)
        return H264_DEC_ERROR;
    if (headerset)
        *headerset = h->info;
    if (!out_frame)
        return H264_DEC_DEFAULT;

    h264_decoder_lock(h);
    if (h->ready_count <= 0) {
        h264_decoder_unlock(h);
        return H264_DEC_DEFAULT;
    }
    f = h->ready[0];
    memmove(&h->ready[0], &h->ready[1], (h->ready_count - 1) * sizeof(H264Frame *));
    h->ready_count--;
    h264_decoder_unlock(h);

    fill_picture(h, f, out_frame);   /* the caller owns one reference to f */
    return H264_DEC_GOT_FRAME;
}

int h264_dec_flush(void* decoder, h264_stream_info* headerset, h264_picture_t* out_frame)
{
    H264Decoder *h = (H264Decoder *)decoder;
    H264Frame *f = NULL;
    int code = H264_DEC_END;

    if (!h)
        return H264_DEC_ERROR;
    if (headerset)
        *headerset = h->info;

    h264_decoder_lock(h);
    if (!h->eof) {
        h264_job_wait_all(h);
        if (h->cur_pic) {
            f = h->cur_pic;
            h264_queue_output(h, f);
            h264_frame_unref(h, f);
            h->cur_pic = NULL;
            h->first_field = 0;
        }
        h264_flush_frames(h);
        h->eof = 1;
    }

    if (h->ready_count > 0 && out_frame) {
        f = h->ready[0];
        memmove(&h->ready[0], &h->ready[1], (h->ready_count - 1) * sizeof(H264Frame *));
        h->ready_count--;
        code = H264_DEC_GOT_FRAME;
    }
    h264_decoder_unlock(h);

    if (code == H264_DEC_GOT_FRAME && out_frame)
        fill_picture(h, f, out_frame);
    return code;
}

void h264_dec_frame_unref(void* decoder, h264_picture_t* out_frame)
{
    H264Decoder *h = (H264Decoder *)decoder;
    if (!h || !out_frame || !out_frame->dec_frame)
        return;
    h264_decoder_lock(h);
    h264_frame_unref(h, (H264Frame *)out_frame->dec_frame);
    h264_decoder_unlock(h);
    out_frame->dec_frame = NULL;
}

void h264_dec_close(void* decoder)
{
    H264Decoder *h = (H264Decoder *)decoder;
    int i;

    if (!h)
        return;

    h264_thread_close(h);
    free_au_slices(h);
    if (h->cur_pic) {
        h264_frame_unref(h, h->cur_pic);
        h->cur_pic = NULL;
    }
    h264_clear_refs(h);
    h264_flush_frames(h);
    while (h->ready_count > 0) {
        H264Frame *f = h->ready[0];
        memmove(&h->ready[0], &h->ready[1], (h->ready_count - 1) * sizeof(H264Frame *));
        h->ready_count--;
        h264_frame_unref(h, f);
    }
    h264_pool_free(h);
    slice_pool_free(h);
    free(h->scratch_edge);
    free(h->scratch_bipred);

    h264_sps_unref(&h->active_sps, "close");
    h264_pps_unref(&h->active_pps, "close");
    for (i = 0; i < H264_MAX_SPS_COUNT; i++)
        h264_sps_unref(&h->sps_list[i], "close");
    for (i = 0; i < H264_MAX_PPS_COUNT; i++)
        h264_pps_unref(&h->pps_list[i], "close");

    free(h->mb2b_xy);
    free(h->mb2br_xy);
    free(h->intra4x4_pred_mode);
    free(h->nal_buf);
    h264_mutex_destroy(&h->slice_pool_m);
    h264_mutex_destroy(&h->mutex);
    h264_cond_destroy(&h->cond);
    h264_mutex_destroy(&h->filter_q.m);
    h264_cond_destroy(&h->filter_q.claim_c);
    free(h);
}
