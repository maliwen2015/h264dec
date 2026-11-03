/*
 * H.264 CABAC decoding (8-bit 4:2:0).
 * Engine ported from FFmpeg's cabac_functions.h / cabac.c and the syntax
 * decoder from h264_cabac.c (LGPL-2.1+); table data is in h264_cabac_data.c.
 */
#include "h264_internal.h"

/* the reference indexes its left neighbours as [LBOT]=0, [LTOP]=1; ours uses
 * [0]=top-ish and [1]=bottom-ish, so only the swap helper is shared */
#define LEFT(i) ((i) ^ 1)

#define H264_CABAC_NORM_SHIFT  (h264_cabac_tables + H264_NORM_SHIFT_OFFSET)
#define H264_CABAC_LPS_RANGE   (h264_cabac_tables + H264_LPS_RANGE_OFFSET)
#define H264_CABAC_MLPS_STATE  (h264_cabac_tables + H264_MLPS_STATE_OFFSET)
#define H264_CABAC_LAST_COEFF_8x8 \
    (h264_cabac_tables + H264_LAST_COEFF_FLAG_OFFSET_8x8_OFFSET)

static inline int cabac_byte(const H264CABAC *c, int off)
{
    const uint8_t *p = c->bytestream + off;
    return p < c->bytestream_end ? *p : 0;
}

static void cabac_refill(H264CABAC *c)
{
    c->low += (cabac_byte(c, 0) << 9) + (cabac_byte(c, 1) << 1);
    c->low -= CABAC_MASK;
    c->bytestream += CABAC_BITS / 8;
}

static void cabac_refill2(H264CABAC *c)
{
    unsigned x = c->low ^ (c->low - 1);
    int i = 7 - H264_CABAC_NORM_SHIFT[x >> (CABAC_BITS - 1)];

    x = -CABAC_MASK;
    x += (cabac_byte(c, 0) << 9) + (cabac_byte(c, 1) << 1);
    c->low += x << i;
    c->bytestream += CABAC_BITS / 8;
}

static inline void cabac_renorm_once(H264CABAC *c)
{
    int shift = (uint32_t)(c->range - 0x100) >> 31;
    c->range <<= shift;
    c->low <<= shift;
    if (!(uint16_t)c->low)
        cabac_refill(c);
}

static inline int get_cabac(H264CABAC *c, uint8_t *const state)
{
    int s = *state;
    int RangeLPS = H264_CABAC_LPS_RANGE[(unsigned)(2 * (c->range & 0xC0) + s)];
    int bit, lps_mask;

    c->range -= RangeLPS;
    lps_mask = ((c->range << (CABAC_BITS + 1)) - c->low) >> 31;

    c->low -= (c->range << (CABAC_BITS + 1)) & lps_mask;
    c->range += (RangeLPS - c->range) & lps_mask;

    s ^= lps_mask;
    *state = (H264_CABAC_MLPS_STATE + 128)[s];
    bit = s & 1;

    /* the index is known positive; the unsigned spelling keeps gcc from
     * sign-extending it before the table load */
    lps_mask = H264_CABAC_NORM_SHIFT[(unsigned)c->range];
    c->range <<= lps_mask;
    c->low <<= lps_mask;
    /* the mask test is a 16-bit compare, i.e. test %dx,%dx, rather than a
     * mov+and pair */
    if (!(uint16_t)c->low)
        cabac_refill2(c);
    return bit;
}

/* the syntax decoder is written against the inlined engine; keeping the
 * noinline spelling from the reference makes the port a straight copy */
#define get_cabac_noinline get_cabac

static inline int get_cabac_bypass(H264CABAC *c)
{
    int range;
    c->low += c->low;

    if (!(uint16_t)c->low)
        cabac_refill(c);

    range = c->range << (CABAC_BITS + 1);
    if (c->low < range)
        return 0;
    c->low -= range;
    return 1;
}

static inline int get_cabac_bypass_sign(H264CABAC *c, int val)
{
    int range, mask;
    c->low += c->low;

    if (!(uint16_t)c->low)
        cabac_refill(c);

    range = c->range << (CABAC_BITS + 1);
    c->low -= range;
    mask = c->low >> 31;
    range &= mask;
    c->low += range;
    return (val ^ mask) - mask;
}

/** @return the number of bytes read, or 0 if the slice has not ended */
static inline int get_cabac_terminate(H264CABAC *c)
{
    c->range -= 2;
    if (c->low < c->range << (CABAC_BITS + 1)) {
        cabac_renorm_once(c);
        return 0;
    }
    return c->bytestream - c->bytestream_start;
}

int h264_cabac_init(H264Slice *sl, const uint8_t *buf, int size)
{
    H264CABAC *c = &sl->cabac;
    const uint8_t *p = buf;

    if (size < 1)
        return -1;

    c->bytestream_start = c->bytestream = buf;
    c->bytestream_end = buf + size;

    c->low = (cabac_byte(c, 0) << 18) + (cabac_byte(c, 1) << 10);
    p += 2;
    /* keep the read pointer on a 2-byte boundary, as the refill loads assume */
    if (((uintptr_t)p & 1) == 0) { 
        c->low += 1 << 9;
    } else {
        c->low += (p < c->bytestream_end ? *p : 0) * 4 + 2;
        p++;
    }
    c->bytestream = p;
    c->range = 0x1FE;
    if (h264_dbg()->bits)
        fprintf(stderr, "CINIT low=%d range=%d size=%d b=%d,%d,%d,%d\n",
                c->low, c->range, size,
                size > 0 ? buf[0] : 0, size > 1 ? buf[1] : 0,
                size > 2 ? buf[2] : 0, size > 3 ? buf[3] : 0);
    if ((c->range << (CABAC_BITS + 1)) < c->low)
        return -1;
    return 0;
}

int h264_cabac_terminate(H264Slice *sl)
{
    return get_cabac_terminate(&sl->cabac);
}

void h264_init_cabac_states(H264Decoder *h, H264Slice *sl)
{
    const int8_t (*tab)[2];
    const int slice_qp = H264_CLIP(sl->qscale - 6 * (h->active_sps->bit_depth_luma - 8), 0, 51);
    int i;

    if (sl->slice_type_nos == H264_SLICE_I)
        tab = h264_cabac_context_init_I;
    else
        tab = h264_cabac_context_init_PB[sl->cabac_init_idc];

    for (i = 0; i < 1024; i++) {
        int pre = 2 * (((tab[i][0] * slice_qp) >> 4) + tab[i][1]) - 127;

        pre ^= pre >> 31;
        if (pre > 124)
            pre = 124 + (pre & 1);

        sl->cabac_state[i] = (uint8_t)pre;
    }
}

/* block size in samples per chroma format, 8-bit */
static const int h264_cabac_mb_sizes[4] = { 256, 384, 512, 768 };

/* intra prediction mode of the top-left neighbour of 4x4 block n, as decoded
 * from the two neighbouring cache slots (DC when either is unavailable) */
static inline int h264_pred_intra_mode(H264Slice *sl, int n)
{
    const int index8 = h264_scan8[n];
    const int left = sl->intra4x4_pred_mode_cache[index8 - 1];
    const int top = sl->intra4x4_pred_mode_cache[index8 - 8];
    const int min = H264_MIN(left, top);

    return min < 0 ? DC_PRED : min;
}

/* 8x8 transform is allowed unless a sub-partition of the MB forbids it */
static inline int h264_get_dct8x8_allowed(H264Decoder *h, H264Slice *sl)
{
    if (h->active_sps->direct_8x8_inference_flag)
        return !(AV_RN64A(sl->sub_mb_type) &
                 ((MB_TYPE_16x8 | MB_TYPE_8x16 | MB_TYPE_8x8) *
                  0x0001000100010001ULL));
    return !(AV_RN64A(sl->sub_mb_type) &
             ((MB_TYPE_16x8 | MB_TYPE_8x16 | MB_TYPE_8x8 | MB_TYPE_DIRECT2) *
              0x0001000100010001ULL));
}

int h264_decode_mb_cabac(H264Decoder *h, H264Slice *sl);

/* ------------------------------------------------------------------ */
/* MB syntax: ported from FFmpeg's h264_cabac.c (LGPL-2.1+)             */
/* ------------------------------------------------------------------ */

static inline uint16_t pack8to16(unsigned a, unsigned b)
{
    return (a & 0xFF) + (b << 8);
}

static int decode_cabac_field_decoding_flag(H264Decoder *h, H264Slice *sl)
{
    const int mbb_xy = sl->mb_xy - 2*h->mb_stride;

    unsigned long ctx = 0;

    ctx += sl->mb_field_decoding_flag & !!sl->mb_x; //for FMO:(s->current_picture.mb_type[mba_xy] >> 7) & (h->cur_pic->slice_table[mba_xy] == h->slice_num);
    ctx += (h->cur_pic->mb_type[mbb_xy] >> 7) & (h->cur_pic->slice_table[mbb_xy] == sl->slice_num);

    return get_cabac_noinline( &sl->cabac, &(sl->cabac_state+70)[ctx] );
}

static int decode_cabac_intra_mb_type(H264Slice *sl,
                                      int ctx_base, int intra_slice)
{
    uint8_t *state= &sl->cabac_state[ctx_base];
    int mb_type;

    if(intra_slice){
        int ctx=0;
        if (sl->left_type[0] & (MB_TYPE_INTRA16x16|MB_TYPE_INTRA_PCM))
            ctx++;
        if (sl->top_type        & (MB_TYPE_INTRA16x16|MB_TYPE_INTRA_PCM))
            ctx++;
        if( get_cabac_noinline( &sl->cabac, &state[ctx] ) == 0 )
            return 0;   /* I4x4 */
        state += 2;
    }else{
        if( get_cabac_noinline( &sl->cabac, state ) == 0 )
            return 0;   /* I4x4 */
    }

    if( get_cabac_terminate( &sl->cabac ) )
        return 25;  /* PCM */

    mb_type = 1; /* I16x16 */
    mb_type += 12 * get_cabac_noinline( &sl->cabac, &state[1] ); /* cbp_luma != 0 */
    if( get_cabac_noinline( &sl->cabac, &state[2] ) ) /* cbp_chroma */
        mb_type += 4 + 4 * get_cabac_noinline( &sl->cabac, &state[2+intra_slice] );
    mb_type += 2 * get_cabac_noinline( &sl->cabac, &state[3+intra_slice] );
    mb_type += 1 * get_cabac_noinline( &sl->cabac, &state[3+2*intra_slice] );
    return mb_type;
}

static int decode_cabac_mb_skip(H264Decoder *h, H264Slice *sl,
                                int mb_x, int mb_y)
{
    int mba_xy, mbb_xy;
    int ctx = 0;

    if (h->cur_pic->mbaff) { //FIXME merge with the stuff in fill_caches?
        int mb_xy = mb_x + (mb_y&~1)*h->mb_stride;
        mba_xy = mb_xy - 1;
        if( (mb_y&1)
            && h->cur_pic->slice_table[mba_xy] == sl->slice_num
            && MB_FIELD(sl) == !!IS_INTERLACED( h->cur_pic->mb_type[mba_xy] ) )
            mba_xy += h->mb_stride;
        if (MB_FIELD(sl)) {
            mbb_xy = mb_xy - h->mb_stride;
            if( !(mb_y&1)
                && h->cur_pic->slice_table[mbb_xy] == sl->slice_num
                && IS_INTERLACED( h->cur_pic->mb_type[mbb_xy] ) )
                mbb_xy -= h->mb_stride;
        }else
            mbb_xy = mb_x + (mb_y-1)*h->mb_stride;
    }else{
        int mb_xy = sl->mb_xy;
        mba_xy = mb_xy - 1;
        mbb_xy = mb_xy - (h->mb_stride << h->cur_pic->field_picture);
    }

    if( h->cur_pic->slice_table[mba_xy] == sl->slice_num && !IS_SKIP(h->cur_pic->mb_type[mba_xy] ))
        ctx++;
    if( h->cur_pic->slice_table[mbb_xy] == sl->slice_num && !IS_SKIP(h->cur_pic->mb_type[mbb_xy] ))
        ctx++;

    if (sl->slice_type_nos == H264_SLICE_B)
        ctx += 13;
    return get_cabac_noinline( &sl->cabac, &sl->cabac_state[11+ctx] );
}

static int decode_cabac_mb_intra4x4_pred_mode(H264Slice *sl, int pred_mode)
{
    int mode = 0;

    if( get_cabac( &sl->cabac, &sl->cabac_state[68] ) )
        return pred_mode;

    mode += 1 * get_cabac( &sl->cabac, &sl->cabac_state[69] );
    mode += 2 * get_cabac( &sl->cabac, &sl->cabac_state[69] );
    mode += 4 * get_cabac( &sl->cabac, &sl->cabac_state[69] );

    return mode + ( mode >= pred_mode );
}

static int decode_cabac_mb_chroma_pre_mode(H264Decoder *h, H264Slice *sl)
{
    const int mba_xy = sl->left_mb_xy[0];
    const int mbb_xy = sl->top_mb_xy;

    int ctx = 0;

    /* No need to test for IS_INTRA4x4 and IS_INTRA16x16, as we set chroma_pred_mode_table to 0 */
    if (sl->left_type[0] && h->cur_pic->chroma_pred_mode[mba_xy] != 0)
        ctx++;

    if (sl->top_type        && h->cur_pic->chroma_pred_mode[mbb_xy] != 0)
        ctx++;

    if( get_cabac_noinline( &sl->cabac, &sl->cabac_state[64+ctx] ) == 0 )
        return 0;

    if( get_cabac_noinline( &sl->cabac, &sl->cabac_state[64+3] ) == 0 )
        return 1;
    if( get_cabac_noinline( &sl->cabac, &sl->cabac_state[64+3] ) == 0 )
        return 2;
    else
        return 3;
}

static int decode_cabac_mb_cbp_luma(H264Slice *sl)
{
    int cbp_b, cbp_a, ctx, cbp = 0;

    cbp_a = sl->left_cbp;
    cbp_b = sl->top_cbp;

    ctx = !(cbp_a & 0x02) + 2 * !(cbp_b & 0x04);
    cbp += get_cabac_noinline(&sl->cabac, &sl->cabac_state[73 + ctx]);
    ctx = !(cbp   & 0x01) + 2 * !(cbp_b & 0x08);
    cbp += get_cabac_noinline(&sl->cabac, &sl->cabac_state[73 + ctx]) << 1;
    ctx = !(cbp_a & 0x08) + 2 * !(cbp   & 0x01);
    cbp += get_cabac_noinline(&sl->cabac, &sl->cabac_state[73 + ctx]) << 2;
    ctx = !(cbp   & 0x04) + 2 * !(cbp   & 0x02);
    cbp += get_cabac_noinline(&sl->cabac, &sl->cabac_state[73 + ctx]) << 3;
    return cbp;
}
static int decode_cabac_mb_cbp_chroma(H264Slice *sl)
{
    int ctx;
    int cbp_a, cbp_b;

    cbp_a = (sl->left_cbp>>4)&0x03;
    cbp_b = (sl-> top_cbp>>4)&0x03;

    ctx = 0;
    if( cbp_a > 0 ) ctx++;
    if( cbp_b > 0 ) ctx += 2;
    if( get_cabac_noinline( &sl->cabac, &sl->cabac_state[77 + ctx] ) == 0 )
        return 0;

    ctx = 4;
    if( cbp_a == 2 ) ctx++;
    if( cbp_b == 2 ) ctx += 2;
    return 1 + get_cabac_noinline( &sl->cabac, &sl->cabac_state[77 + ctx] );
}

static int decode_cabac_p_mb_sub_type(H264Slice *sl)
{
    if( get_cabac( &sl->cabac, &sl->cabac_state[21] ) )
        return 0;   /* 8x8 */
    if( !get_cabac( &sl->cabac, &sl->cabac_state[22] ) )
        return 1;   /* 8x4 */
    if( get_cabac( &sl->cabac, &sl->cabac_state[23] ) )
        return 2;   /* 4x8 */
    return 3;       /* 4x4 */
}
static int decode_cabac_b_mb_sub_type(H264Slice *sl)
{
    int type;
    if( !get_cabac( &sl->cabac, &sl->cabac_state[36] ) )
        return 0;   /* B_Direct_8x8 */
    if( !get_cabac( &sl->cabac, &sl->cabac_state[37] ) )
        return 1 + get_cabac( &sl->cabac, &sl->cabac_state[39] ); /* B_L0_8x8, B_L1_8x8 */
    type = 3;
    if( get_cabac( &sl->cabac, &sl->cabac_state[38] ) ) {
        if( get_cabac( &sl->cabac, &sl->cabac_state[39] ) )
            return 11 + get_cabac( &sl->cabac, &sl->cabac_state[39] ); /* B_L1_4x4, B_Bi_4x4 */
        type += 4;
    }
    type += 2*get_cabac( &sl->cabac, &sl->cabac_state[39] );
    type +=   get_cabac( &sl->cabac, &sl->cabac_state[39] );
    return type;
}

static int decode_cabac_mb_ref(H264Slice *sl, int list, int n)
{
    int refa = sl->ref_cache[list][h264_scan8[n] - 1];
    int refb = sl->ref_cache[list][h264_scan8[n] - 8];
    int ref  = 0;
    int ctx  = 0;

    if (sl->slice_type_nos == H264_SLICE_B) {
        if( refa > 0 && !(sl->direct_cache[h264_scan8[n] - 1]&(MB_TYPE_DIRECT2>>1)) )
            ctx++;
        if( refb > 0 && !(sl->direct_cache[h264_scan8[n] - 8]&(MB_TYPE_DIRECT2>>1)) )
            ctx += 2;
    } else {
        if( refa > 0 )
            ctx++;
        if( refb > 0 )
            ctx += 2;
    }

    while( get_cabac( &sl->cabac, &sl->cabac_state[54+ctx] ) ) {
        ref++;
        ctx = (ctx>>2)+4;
        if(ref >= 32 /*h->ref_list[list]*/){
            return -1;
        }
    }
    return ref;
}

static int decode_cabac_mb_mvd(H264Slice *sl, int ctxbase, int amvd, int *mvda)
{
    int mvd;

    /* the INT_MIN path below returns without writing it, and the macroblock
     * decode that inlines this would then hand the value to the mvd cache */
    *mvda = 0;


    if(!get_cabac(&sl->cabac, &sl->cabac_state[ctxbase+((amvd-3)>>(31-1))+((amvd-33)>>(31-1))+2])){
//    if(!get_cabac(&sl->cabac, &sl->cabac_state[ctxbase+(amvd>2)+(amvd>32)])){
        *mvda= 0;
        return 0;
    }

    mvd= 1;
    ctxbase+= 3;
    while( mvd < 9 && get_cabac( &sl->cabac, &sl->cabac_state[ctxbase] ) ) {
        if( mvd < 4 )
            ctxbase++;
        mvd++;
    }

    if( mvd >= 9 ) {
        int k = 3;
        while( get_cabac_bypass( &sl->cabac ) ) {
            mvd += 1 << k;
            k++;
            if(k>24){
                                return INT_MIN;
            }
        }
        while( k-- ) {
            mvd += get_cabac_bypass( &sl->cabac )<<k;
        }
        *mvda=mvd < 70 ? mvd : 70;
    }else
        *mvda=mvd;
    return get_cabac_bypass_sign( &sl->cabac, -mvd );
}

#define DECODE_CABAC_MB_MVD(sl, list,  n )\
{\
    int amvd0 = sl->mvd_cache[list][h264_scan8[n] - 1][0] +\
                sl->mvd_cache[list][h264_scan8[n] - 8][0];\
    int amvd1 = sl->mvd_cache[list][h264_scan8[n] - 1][1] +\
                sl->mvd_cache[list][h264_scan8[n] - 8][1];\
\
    int mxd = decode_cabac_mb_mvd(sl, 40, amvd0, &mpx);\
    int myd = decode_cabac_mb_mvd(sl, 47, amvd1, &mpy);\
    if (mxd == INT_MIN || myd == INT_MIN) \
        return -1; \
    mx += mxd;\
    my += myd;\
}

static inline int get_cabac_cbf_ctx(H264Slice *sl,
                                              int cat, int idx, int max_coeff,
                                              int is_dc)
{
    int nza, nzb;
    int ctx = 0;
    static const uint16_t base_ctx[14] = {85,89,93,97,101,1012,460,464,468,1016,472,476,480,1020};

    if( is_dc ) {
        if( cat == 3 ) {
            idx -= CHROMA_DC_BLOCK_INDEX;
            nza = (sl->left_cbp>>(6+idx))&0x01;
            nzb = (sl-> top_cbp>>(6+idx))&0x01;
        } else {
            idx -= LUMA_DC_BLOCK_INDEX;
            nza = sl->left_cbp&(0x100<<idx);
            nzb = sl-> top_cbp&(0x100<<idx);
        }
    } else {
        nza = sl->non_zero_count_cache[h264_scan8[idx] - 1];
        nzb = sl->non_zero_count_cache[h264_scan8[idx] - 8];
    }

    if( nza > 0 )
        ctx++;

    if( nzb > 0 )
        ctx += 2;

    return base_ctx[cat] + ctx;
}

H264_INLINE void
decode_cabac_residual_internal(H264Decoder *h, H264Slice *sl,
                               int16_t *block,
                               int cat, int n, const uint8_t *scantable,
                               const uint32_t *qmul, int max_coeff,
                               int is_dc, int chroma422)
{
    static const int significant_coeff_flag_offset[2][14] = {
      { 105+0, 105+15, 105+29, 105+44, 105+47, 402, 484+0, 484+15, 484+29, 660, 528+0, 528+15, 528+29, 718 },
      { 277+0, 277+15, 277+29, 277+44, 277+47, 436, 776+0, 776+15, 776+29, 675, 820+0, 820+15, 820+29, 733 }
    };
    static const int last_coeff_flag_offset[2][14] = {
      { 166+0, 166+15, 166+29, 166+44, 166+47, 417, 572+0, 572+15, 572+29, 690, 616+0, 616+15, 616+29, 748 },
      { 338+0, 338+15, 338+29, 338+44, 338+47, 451, 864+0, 864+15, 864+29, 699, 908+0, 908+15, 908+29, 757 }
    };
    static const int coeff_abs_level_m1_offset[14] = {
        227+0, 227+10, 227+20, 227+30, 227+39, 426, 952+0, 952+10, 952+20, 708, 982+0, 982+10, 982+20, 766
    };
    static const uint8_t significant_coeff_flag_offset_8x8[2][63] = {
      { 0, 1, 2, 3, 4, 5, 5, 4, 4, 3, 3, 4, 4, 4, 5, 5,
        4, 4, 4, 4, 3, 3, 6, 7, 7, 7, 8, 9,10, 9, 8, 7,
        7, 6,11,12,13,11, 6, 7, 8, 9,14,10, 9, 8, 6,11,
       12,13,11, 6, 9,14,10, 9,11,12,13,11,14,10,12 },
      { 0, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 7, 7, 8, 4, 5,
        6, 9,10,10, 8,11,12,11, 9, 9,10,10, 8,11,12,11,
        9, 9,10,10, 8,11,12,11, 9, 9,10,10, 8,13,13, 9,
        9,10,10, 8,13,13, 9, 9,10,10,14,14,14,14,14 }
    };
    static const uint8_t sig_coeff_offset_dc[7] = { 0, 0, 1, 1, 2, 2, 2 };
    /* node ctx: 0..3: abslevel1 (with abslevelgt1 == 0).
     * 4..7: abslevelgt1 + 3 (and abslevel1 doesn't matter).
     * map node ctx => cabac ctx for level=1 */
    static const uint8_t coeff_abs_level1_ctx[8] = { 1, 2, 3, 4, 0, 0, 0, 0 };
    /* map node ctx => cabac ctx for level>1 */
    static const uint8_t coeff_abs_levelgt1_ctx[2][8] = {
        { 5, 5, 5, 5, 6, 7, 8, 9 },
        { 5, 5, 5, 5, 6, 7, 8, 8 }, // 422/dc case
    };
    static const uint8_t coeff_abs_level_transition[2][8] = {
    /* update node ctx after decoding a level=1 */
        { 1, 2, 3, 3, 4, 5, 6, 7 },
    /* update node ctx after decoding a level>1 */
        { 4, 4, 4, 4, 5, 6, 7, 7 }
    };

    int index[64];

    int last;
    int coeff_count = 0;
    int node_ctx = 0;
    /* the engine runs this block out of locals: a store through block[] or
     * sl-> may alias sl->cabac, so every bin was reloading low and range from
     * memory, but nothing outside can reach a local. 2.2% at one slot. */
    H264CABAC cab = sl->cabac;

    uint8_t *significant_coeff_ctx_base;
    uint8_t *last_coeff_ctx_base;
    uint8_t *abs_level_m1_ctx_base;


    significant_coeff_ctx_base = sl->cabac_state
        + significant_coeff_flag_offset[MB_FIELD(sl)][cat];
    last_coeff_ctx_base = sl->cabac_state
        + last_coeff_flag_offset[MB_FIELD(sl)][cat];
    abs_level_m1_ctx_base = sl->cabac_state
        + coeff_abs_level_m1_offset[cat];

    if( !is_dc && max_coeff == 64 ) {
#define DECODE_SIGNIFICANCE( coefs, sig_off, last_off ) \
        for(last= 0; last < coefs; last++) { \
            uint8_t *sig_ctx = significant_coeff_ctx_base + sig_off; \
            if( get_cabac( &cab, sig_ctx )) { \
                uint8_t *last_ctx = last_coeff_ctx_base + last_off; \
                index[coeff_count++] = last; \
                if( get_cabac( &cab, last_ctx ) ) { \
                    last= max_coeff; \
                    break; \
                } \
            } \
        }\
        if( last == max_coeff -1 ) {\
            index[coeff_count++] = last;\
        }
        
        const uint8_t *sig_off = significant_coeff_flag_offset_8x8[MB_FIELD(sl)];
        DECODE_SIGNIFICANCE(63, sig_off[last], H264_CABAC_LAST_COEFF_8x8[last]);
    } else {
        if (is_dc && chroma422) {
            DECODE_SIGNIFICANCE(7, sig_coeff_offset_dc[last], sig_coeff_offset_dc[last]);
        } else {
            DECODE_SIGNIFICANCE(max_coeff - 1, last, last);
        }
    }

    if( is_dc ) {
        if( cat == 3 )
            h->cur_pic->cbp_table[sl->mb_xy] |= 0x40 << (n - CHROMA_DC_BLOCK_INDEX);
        else
            h->cur_pic->cbp_table[sl->mb_xy] |= 0x100 << (n - LUMA_DC_BLOCK_INDEX);
        sl->non_zero_count_cache[h264_scan8[n]] = coeff_count;
    } else {
        if( max_coeff == 64 )
            h264_fill_rectangle(&sl->non_zero_count_cache[h264_scan8[n]], 2, 2, 8, coeff_count, 1);
        else {
            sl->non_zero_count_cache[h264_scan8[n]] = coeff_count;
        }
    }

#define STORE_BLOCK(type) \
    do { \
        uint8_t *ctx = coeff_abs_level1_ctx[node_ctx] + abs_level_m1_ctx_base; \
 \
        int j= scantable[index[--coeff_count]]; \
 \
        if( get_cabac( &cab, ctx ) == 0 ) { \
            node_ctx = coeff_abs_level_transition[0][node_ctx]; \
            if( is_dc ) { \
                ((type*)block)[j] = get_cabac_bypass_sign( &cab, -1); \
            }else{ \
                ((type*)block)[j] = (get_cabac_bypass_sign( &cab, -qmul[j]) + 32) >> 6; \
            } \
        } else { \
            unsigned coeff_abs = 2; \
            ctx = coeff_abs_levelgt1_ctx[is_dc && chroma422][node_ctx] + abs_level_m1_ctx_base; \
            node_ctx = coeff_abs_level_transition[1][node_ctx]; \
\
            while( coeff_abs < 15 && get_cabac( &cab, ctx ) ) { \
                coeff_abs++; \
            } \
\
            if( coeff_abs >= 15 ) { \
                int j = 0; \
                while (get_cabac_bypass(&cab) && j < 16+7) { \
                    j++; \
                } \
\
                coeff_abs=1; \
                while( j-- ) { \
                    coeff_abs += coeff_abs + get_cabac_bypass( &cab ); \
                } \
                coeff_abs+= 14U; \
            } \
\
            if( is_dc ) { \
                ((type*)block)[j] = get_cabac_bypass_sign( &cab, -coeff_abs ); \
            }else{ \
                ((type*)block)[j] = ((int)(get_cabac_bypass_sign( &cab, -coeff_abs ) * qmul[j] + 32)) >> 6; \
            } \
        } \
    } while ( coeff_count );

    if (sl->pixel_shift)
        STORE_BLOCK(int32_t)
    else
        STORE_BLOCK(int16_t)
    sl->cabac = cab;
}

static  void decode_cabac_residual_dc_internal(H264Decoder *h,
                                                          H264Slice *sl,
                                                          int16_t *block,
                                                          int cat, int n,
                                                          const uint8_t *scantable,
                                                          int max_coeff)
{
    decode_cabac_residual_internal(h, sl, block, cat, n, scantable, NULL, max_coeff, 1, 0);
}

static  void decode_cabac_residual_dc_internal_422(H264Decoder *h,
                                                              H264Slice *sl,
                                                              int16_t *block,
                                                              int cat, int n,
                                                              const uint8_t *scantable,
                                                              int max_coeff)
{
    decode_cabac_residual_internal(h, sl, block, cat, n, scantable, NULL, max_coeff, 1, 1);
}

static  void decode_cabac_residual_nondc_internal(H264Decoder *h,
                                                             H264Slice *sl,
                                                             int16_t *block,
                                                             int cat, int n,
                                                             const uint8_t *scantable,
                                                             const uint32_t *qmul,
                                                             int max_coeff)
{
    decode_cabac_residual_internal(h, sl, block, cat, n, scantable, qmul, max_coeff, 0, 0);
}

/* cat: 0-> DC 16x16  n = 0
 *      1-> AC 16x16  n = luma4x4idx
 *      2-> Luma4x4   n = luma4x4idx
 *      3-> DC Chroma n = iCbCr
 *      4-> AC Chroma n = 16 + 4 * iCbCr + chroma4x4idx
 *      5-> Luma8x8   n = 4 * luma8x8idx */

/* Partially inline the CABAC residual decode: inline the coded block flag.
 * This has very little impact on binary size and improves performance
 * because it allows improved constant propagation into get_cabac_cbf_ctx,
 * as well as because most blocks have zero CBFs. */

static inline void decode_cabac_residual_dc(H264Decoder *h,
                                                      H264Slice *sl,
                                                      int16_t *block,
                                                      int cat, int n,
                                                      const uint8_t *scantable,
                                                      int max_coeff)
{
    /* read coded block flag */
    if( get_cabac( &sl->cabac, &sl->cabac_state[get_cabac_cbf_ctx(sl, cat, n, max_coeff, 1)]) == 0 ) {
        sl->non_zero_count_cache[h264_scan8[n]] = 0;
        return;
    }
    decode_cabac_residual_dc_internal(h, sl, block, cat, n, scantable, max_coeff);
}

H264_INLINE void
decode_cabac_residual_dc_422(H264Decoder *h, H264Slice *sl,
                             int16_t *block,
                             int cat, int n, const uint8_t *scantable,
                             int max_coeff)
{
    /* read coded block flag */
    if (get_cabac(&sl->cabac, &sl->cabac_state[get_cabac_cbf_ctx(sl, cat, n, max_coeff, 1)]) == 0) {
        sl->non_zero_count_cache[h264_scan8[n]] = 0;
        return;
    }
    decode_cabac_residual_dc_internal_422(h, sl, block, cat, n, scantable, max_coeff);
}

static inline void decode_cabac_residual_nondc(H264Decoder *h,
                                                         H264Slice *sl,
                                                         int16_t *block,
                                                         int cat, int n,
                                                         const uint8_t *scantable,
                                                         const uint32_t *qmul,
                                                         int max_coeff)
{
    /* read coded block flag */
    if( (cat != 5 || h->active_sps->chroma_format_idc == 3) &&
        get_cabac( &sl->cabac, &sl->cabac_state[get_cabac_cbf_ctx(sl, cat, n, max_coeff, 0)]) == 0) {
        if( max_coeff == 64 ) {
            h264_fill_rectangle(&sl->non_zero_count_cache[h264_scan8[n]], 2, 2, 8, 0, 1);
        } else {
            sl->non_zero_count_cache[h264_scan8[n]] = 0;
        }
        return;
    }
    decode_cabac_residual_nondc_internal(h, sl, block, cat, n, scantable, qmul, max_coeff);
}

static inline void decode_cabac_luma_residual(H264Decoder *h, H264Slice *sl,
                                                        const uint8_t *scan, const uint8_t *scan8x8,
                                                        int pixel_shift, int mb_type, int cbp, int p)
{
    static const uint8_t ctx_cat[4][3] = {{0,6,10},{1,7,11},{2,8,12},{5,9,13}};
    const uint32_t *qmul;
    int i8x8, i4x4;
    int qscale = p == 0 ? sl->qscale : sl->chroma_qp[p - 1];
    if( IS_INTRA16x16( mb_type ) ) {
        AV_ZERO128(sl->mb_luma_dc[p]+0);
        AV_ZERO128(sl->mb_luma_dc[p]+8);
        AV_ZERO128(sl->mb_luma_dc[p]+16);
        AV_ZERO128(sl->mb_luma_dc[p]+24);
        decode_cabac_residual_dc(h, sl, sl->mb_luma_dc[p], ctx_cat[0][p], LUMA_DC_BLOCK_INDEX+p, scan, 16);

        if( cbp&15 ) {
            qmul = h->active_pps->dequant4_coeff[p][qscale];
            for( i4x4 = 0; i4x4 < 16; i4x4++ ) {
                const int index = 16*p + i4x4;
                decode_cabac_residual_nondc(h, sl, sl->mb + (16*index << pixel_shift), ctx_cat[1][p], index, scan + 1, qmul, 15);
            }
        } else {
            h264_fill_rectangle(&sl->non_zero_count_cache[h264_scan8[16*p]], 4, 4, 8, 0, 1);
        }
    } else {
        int cqm = (IS_INTRA( mb_type ) ? 0:3) + p;
        for( i8x8 = 0; i8x8 < 4; i8x8++ ) {
            if( cbp & (1<<i8x8) ) {
                if( IS_8x8DCT(mb_type) ) {
                    const int index = 16*p + 4*i8x8;
                    decode_cabac_residual_nondc(h, sl, sl->mb + (16*index << pixel_shift), ctx_cat[3][p], index,
                                                scan8x8, h->active_pps->dequant8_coeff[cqm][qscale], 64);
                } else {
                    qmul = h->active_pps->dequant4_coeff[cqm][qscale];
                    for( i4x4 = 0; i4x4 < 4; i4x4++ ) {
                        const int index = 16*p + 4*i8x8 + i4x4;
                        decode_cabac_residual_nondc(h, sl, sl->mb + (16*index << pixel_shift), ctx_cat[2][p], index, scan, qmul, 16);
                    }
                }
            } else {
                h264_fill_rectangle(&sl->non_zero_count_cache[h264_scan8[4*i8x8+16*p]], 2, 2, 8, 0, 1);
            }
        }
    }
}

/**
 * Decode a macroblock.
 * @return 0 if OK, ER_AC_ERROR / ER_DC_ERROR / ER_MV_ERROR if an error is noticed
 */
/* The syntax helpers and the residual decoder are left to gcc's inliner. The
 * flatten attribute that used to force them all in measured 0.995x on both
 * bench clips once the rest of the entropy path had been tuned, and the object
 * is 10% smaller without it; the residual decoder keeps low and range in its
 * own local across the coefficient stores, which is where that locality pays. */
int h264_decode_mb_cabac(H264Decoder *h, H264Slice *sl)
{
    const H264SPS *sps = h->active_sps;
    int mb_xy;
    uint32_t mb_type;
    int partition_count, cbp = 0;
    int dct8x8_allowed = h->active_pps->transform_8x8_mode;
    const int decode_chroma = sps->chroma_format_idc == 1 || sps->chroma_format_idc == 2;
    const int pixel_shift = sl->pixel_shift;

    mb_xy = sl->mb_xy = sl->mb_x + sl->mb_y*h->mb_stride;

    if (sl->slice_type_nos != H264_SLICE_I) {
        int skip;
        /* a skipped mb needs the aff flag from the following mb */
        if (h->cur_pic->mbaff && (sl->mb_y & 1) == 1 && sl->prev_mb_skipped)
            skip = sl->next_mb_skipped;
        else
            skip = decode_cabac_mb_skip(h, sl, sl->mb_x, sl->mb_y );
        /* read skip flags */
        if( skip ) {
            if (h->cur_pic->mbaff && (sl->mb_y & 1) == 0) {
                h->cur_pic->mb_type[mb_xy] = MB_TYPE_SKIP;
                sl->next_mb_skipped = decode_cabac_mb_skip(h, sl, sl->mb_x, sl->mb_y+1 );
                if(!sl->next_mb_skipped)
                    sl->mb_mbaff = sl->mb_field_decoding_flag = decode_cabac_field_decoding_flag(h, sl);
            }

            h264_decode_mb_skip(h, sl);

            h->cur_pic->cbp_table[mb_xy] = 0;
            h->cur_pic->chroma_pred_mode[mb_xy] = 0;
            sl->last_qscale_diff = 0;

            return 0;

        }
    }
    if (h->cur_pic->mbaff) {
        if ((sl->mb_y & 1) == 0)
            sl->mb_mbaff =
            sl->mb_field_decoding_flag = decode_cabac_field_decoding_flag(h, sl);
    }

    sl->prev_mb_skipped = 0;

    h264_fill_decode_neighbors(h, sl, -(MB_FIELD(sl)));

    if (sl->slice_type_nos == H264_SLICE_B) {
        int ctx = 0;

        if (!IS_DIRECT(sl->left_type[0] - 1))
            ctx++;
        if (!IS_DIRECT(sl->top_type - 1))
            ctx++;

        if( !get_cabac_noinline( &sl->cabac, &sl->cabac_state[27+ctx] ) ){
            mb_type= 0; /* B_Direct_16x16 */
        }else if( !get_cabac_noinline( &sl->cabac, &sl->cabac_state[27+3] ) ) {
            mb_type= 1 + get_cabac_noinline( &sl->cabac, &sl->cabac_state[27+5] ); /* B_L[01]_16x16 */
        }else{
            int bits;
            bits = get_cabac_noinline( &sl->cabac, &sl->cabac_state[27+4] ) << 3;
            bits+= get_cabac_noinline( &sl->cabac, &sl->cabac_state[27+5] ) << 2;
            bits+= get_cabac_noinline( &sl->cabac, &sl->cabac_state[27+5] ) << 1;
            bits+= get_cabac_noinline( &sl->cabac, &sl->cabac_state[27+5] );
            if( bits < 8 ){
                mb_type= bits + 3; /* B_Bi_16x16 through B_L1_L0_16x8 */
            }else if( bits == 13 ){
                mb_type = decode_cabac_intra_mb_type(sl, 32, 0);
                goto decode_intra_mb;
            }else if( bits == 14 ){
                mb_type= 11; /* B_L1_L0_8x16 */
            }else if( bits == 15 ){
                mb_type= 22; /* B_8x8 */
            }else{
                bits= ( bits<<1 ) + get_cabac_noinline( &sl->cabac, &sl->cabac_state[27+5] );
                mb_type= bits - 4; /* B_L0_Bi_* through B_Bi_Bi_* */
            }
        }
            partition_count = h264_b_mb_type_info[mb_type].partition_count;
            mb_type         = h264_b_mb_type_info[mb_type].type;
    } else if (sl->slice_type_nos == H264_SLICE_P) {
        if( get_cabac_noinline( &sl->cabac, &sl->cabac_state[14] ) == 0 ) {
            /* P-type */
            if( get_cabac_noinline( &sl->cabac, &sl->cabac_state[15] ) == 0 ) {
                /* P_L0_D16x16, P_8x8 */
                mb_type= 3 * get_cabac_noinline( &sl->cabac, &sl->cabac_state[16] );
            } else {
                /* P_L0_D8x16, P_L0_D16x8 */
                mb_type= 2 - get_cabac_noinline( &sl->cabac, &sl->cabac_state[17] );
            }
            partition_count = h264_p_mb_type_info[mb_type].partition_count;
            mb_type         = h264_p_mb_type_info[mb_type].type;
        } else {
            mb_type = decode_cabac_intra_mb_type(sl, 17, 0);
            goto decode_intra_mb;
        }
    } else {
        mb_type = decode_cabac_intra_mb_type(sl, 3, 1);
        if (sl->slice_type == H264_SLICE_SI && mb_type)
            mb_type--;
decode_intra_mb:
        partition_count = 0;
        cbp                      = h264_i_mb_type_info[mb_type].cbp;
        sl->intra16x16_pred_mode = h264_i_mb_type_info[mb_type].pred_mode;
        mb_type                  = h264_i_mb_type_info[mb_type].type;
    }
    if (MB_FIELD(sl))
        mb_type |= MB_TYPE_INTERLACED;

    h->cur_pic->slice_table[mb_xy] = sl->slice_num;

    if(IS_INTRA_PCM(mb_type)) {
        const int mb_size = h264_cabac_mb_sizes[sps->chroma_format_idc] *
                            sps->bit_depth_luma >> 3;
        const uint8_t *ptr;
        int ret;

        // We assume these blocks are very rare so we do not optimize it.
        // FIXME The two following lines get the bitstream position in the cabac
        // decode, I think it should be done by a function in cabac.h (or cabac.c).
        ptr= sl->cabac.bytestream;
        if(sl->cabac.low&0x1) ptr--;
        if(CABAC_BITS==16){
            if(sl->cabac.low&0x1FF) ptr--;
        }

        // The pixels are stored in the same order as levels in h->mb array.
        if ((int) (sl->cabac.bytestream_end - ptr) < mb_size)
            return -1;
        sl->intra_pcm_ptr = ptr;
        ptr += mb_size;

        ret = h264_cabac_init(sl, ptr, sl->cabac.bytestream_end - ptr);
        if (ret < 0)
            return ret;

        // All blocks are present
        h->cur_pic->cbp_table[mb_xy] = 0xf7ef;
        h->cur_pic->chroma_pred_mode[mb_xy] = 0;
        // In deblocking, the quantizer is 0
        h->cur_pic->qscale_table[mb_xy] = 0;
        // All coeffs are present
        memset(h->cur_pic->nnz[mb_xy], 16, 48);
        h->cur_pic->mb_type[mb_xy] = mb_type;
        sl->last_qscale_diff = 0;
        return 0;
    }

    h264_fill_decode_caches(h, sl, mb_type);

    if( IS_INTRA( mb_type ) ) {
        int i, pred_mode;
        if( IS_INTRA4x4( mb_type ) ) {
            if (dct8x8_allowed && get_cabac_noinline(&sl->cabac, &sl->cabac_state[399 + sl->neighbor_transform_size])) {
                mb_type |= MB_TYPE_8x8DCT;
                for( i = 0; i < 16; i+=4 ) {
                    int pred = h264_pred_intra_mode(sl, i);
                    int mode = decode_cabac_mb_intra4x4_pred_mode(sl, pred);
                    h264_fill_rectangle(&sl->intra4x4_pred_mode_cache[h264_scan8[i]], 2, 2, 8, mode, 1);
                }
            } else {
                for( i = 0; i < 16; i++ ) {
                    int pred = h264_pred_intra_mode(sl, i);
                    sl->intra4x4_pred_mode_cache[h264_scan8[i]] = decode_cabac_mb_intra4x4_pred_mode(sl, pred);

                }
            }
            h264_write_back_intra_pred_mode(h, sl);
            if (h264_check_intra4x4_pred_mode(sl->intra4x4_pred_mode_cache,
                                                 sl->top_samples_available, sl->left_samples_available) < 0 )
                return -1;
        } else {
            sl->intra16x16_pred_mode = h264_check_intra_pred_mode(sl->top_samples_available,
                                                                     sl->left_samples_available, sl->intra16x16_pred_mode, 0);
            if (sl->intra16x16_pred_mode < 0) return -1;
        }
        if(decode_chroma){
            h->cur_pic->chroma_pred_mode[mb_xy] =
            pred_mode                        = decode_cabac_mb_chroma_pre_mode(h, sl);

            pred_mode= h264_check_intra_pred_mode(sl->top_samples_available,
                                                     sl->left_samples_available, pred_mode, 1 );
            if( pred_mode < 0 ) return -1;
            sl->chroma_pred_mode = pred_mode;
        } else {
            sl->chroma_pred_mode = DC_128_PRED8x8;
        }
    } else if( partition_count == 4 ) {
        int i, j, sub_partition_count[4], list, ref[2][4];

        if (sl->slice_type_nos == H264_SLICE_B ) {
            for( i = 0; i < 4; i++ ) {
                sl->sub_mb_type[i] = decode_cabac_b_mb_sub_type(sl);
                sub_partition_count[i] = h264_b_sub_mb_type_info[sl->sub_mb_type[i]].partition_count;
                sl->sub_mb_type[i]     = h264_b_sub_mb_type_info[sl->sub_mb_type[i]].type;
            }
            if (IS_DIRECT(sl->sub_mb_type[0] | sl->sub_mb_type[1] |
                          sl->sub_mb_type[2] | sl->sub_mb_type[3])) {
                h264_pred_direct_motion(h, sl, &mb_type);
                sl->ref_cache[0][h264_scan8[4]] =
                sl->ref_cache[1][h264_scan8[4]] =
                sl->ref_cache[0][h264_scan8[12]] =
                sl->ref_cache[1][h264_scan8[12]] = PART_NOT_AVAILABLE;
                    for( i = 0; i < 4; i++ )
                        h264_fill_rectangle(&sl->direct_cache[h264_scan8[4*i]], 2, 2, 8, (sl->sub_mb_type[i] >> 1) & 0xFF, 1);
            }
        } else {
            for( i = 0; i < 4; i++ ) {
                sl->sub_mb_type[i] = decode_cabac_p_mb_sub_type(sl);
                sub_partition_count[i] = h264_p_sub_mb_type_info[sl->sub_mb_type[i]].partition_count;
                sl->sub_mb_type[i]     = h264_p_sub_mb_type_info[sl->sub_mb_type[i]].type;
            }
        }

        for( list = 0; list < sl->list_count; list++ ) {
                for( i = 0; i < 4; i++ ) {
                    if(IS_DIRECT(sl->sub_mb_type[i])) continue;
                    if(IS_DIR(sl->sub_mb_type[i], 0, list)){
                        int rc = sl->ref_count[list] << MB_MBAFF(sl);
                        if (rc > 1) {
                            ref[list][i] = decode_cabac_mb_ref(sl, list, 4 * i);
                            if (ref[list][i] >= rc) {
                                                                return -1;
                            }
                        }else
                            ref[list][i] = 0;
                    } else {
                        ref[list][i] = -1;
                    }
                    sl->ref_cache[list][h264_scan8[4 * i] + 1] =
                    sl->ref_cache[list][h264_scan8[4 * i] + 8] = sl->ref_cache[list][h264_scan8[4 * i] + 9] = ref[list][i];
                }
        }

        if(dct8x8_allowed)
            dct8x8_allowed = h264_get_dct8x8_allowed(h, sl);

        for (list = 0; list < sl->list_count; list++) {
            for(i=0; i<4; i++){
                sl->ref_cache[list][h264_scan8[4 * i]] = sl->ref_cache[list][h264_scan8[4 * i] + 1];
                if(IS_DIRECT(sl->sub_mb_type[i])){
                    h264_fill_rectangle(sl->mvd_cache[list][h264_scan8[4*i]], 2, 2, 8, 0, 2);
                    continue;
                }

                if(IS_DIR(sl->sub_mb_type[i], 0, list) && !IS_DIRECT(sl->sub_mb_type[i])){
                    const int sub_mb_type= sl->sub_mb_type[i];
                    const int block_width= (sub_mb_type & (MB_TYPE_16x16|MB_TYPE_16x8)) ? 2 : 1;
                    for(j=0; j<sub_partition_count[i]; j++){
                        int mpx, mpy;
                        int mx, my;
                        const int index= 4*i + block_width*j;
                        int16_t (* mv_cache)[2] = &sl->mv_cache[list][ h264_scan8[index] ];
                        uint8_t (* mvd_cache)[2]= &sl->mvd_cache[list][ h264_scan8[index] ];
                        h264_pred_motion(h, sl, index, block_width, list, sl->ref_cache[list][ h264_scan8[index] ], &mx, &my);
                        DECODE_CABAC_MB_MVD(sl, list, index)

                        if(IS_SUB_8X8(sub_mb_type)){
                            mv_cache[ 1 ][0]=
                            mv_cache[ 8 ][0]= mv_cache[ 9 ][0]= mx;
                            mv_cache[ 1 ][1]=
                            mv_cache[ 8 ][1]= mv_cache[ 9 ][1]= my;

                            mvd_cache[ 1 ][0]=
                            mvd_cache[ 8 ][0]= mvd_cache[ 9 ][0]= mpx;
                            mvd_cache[ 1 ][1]=
                            mvd_cache[ 8 ][1]= mvd_cache[ 9 ][1]= mpy;
                        }else if(IS_SUB_8X4(sub_mb_type)){
                            mv_cache[ 1 ][0]= mx;
                            mv_cache[ 1 ][1]= my;

                            mvd_cache[ 1 ][0]=  mpx;
                            mvd_cache[ 1 ][1]= mpy;
                        }else if(IS_SUB_4X8(sub_mb_type)){
                            mv_cache[ 8 ][0]= mx;
                            mv_cache[ 8 ][1]= my;

                            mvd_cache[ 8 ][0]= mpx;
                            mvd_cache[ 8 ][1]= mpy;
                        }
                        mv_cache[ 0 ][0]= mx;
                        mv_cache[ 0 ][1]= my;

                        mvd_cache[ 0 ][0]= mpx;
                        mvd_cache[ 0 ][1]= mpy;
                    }
                }else{
                    h264_fill_rectangle(sl->mv_cache [list][ h264_scan8[4*i] ], 2, 2, 8, 0, 4);
                    h264_fill_rectangle(sl->mvd_cache[list][ h264_scan8[4*i] ], 2, 2, 8, 0, 2);
                }
            }
        }
    } else if( IS_DIRECT(mb_type) ) {
        h264_pred_direct_motion(h, sl, &mb_type);
        h264_fill_rectangle(sl->mvd_cache[0][h264_scan8[0]], 4, 4, 8, 0, 2);
        h264_fill_rectangle(sl->mvd_cache[1][h264_scan8[0]], 4, 4, 8, 0, 2);
        dct8x8_allowed &= sps->direct_8x8_inference_flag;
    } else {
        int list, i;
        if(IS_16X16(mb_type)){
            for (list = 0; list < sl->list_count; list++) {
                if(IS_DIR(mb_type, 0, list)){
                    int ref;
                    int rc = sl->ref_count[list] << MB_MBAFF(sl);
                    if (rc > 1) {
                        ref= decode_cabac_mb_ref(sl, list, 0);
                        if (ref >= rc) {
                                                        return -1;
                        }
                    }else
                        ref=0;
                    h264_fill_rectangle(&sl->ref_cache[list][ h264_scan8[0] ], 4, 4, 8, ref, 1);
                }
            }
            for (list = 0; list < sl->list_count; list++) {
                if(IS_DIR(mb_type, 0, list)){
                    int mx,my,mpx,mpy;
                    h264_pred_motion(h, sl, 0, 4, list, sl->ref_cache[list][ h264_scan8[0] ], &mx, &my);
                    DECODE_CABAC_MB_MVD(sl, list, 0)

                    h264_fill_rectangle(sl->mvd_cache[list][ h264_scan8[0] ], 4, 4, 8, pack8to16(mpx,mpy), 2);
                    h264_fill_rectangle(sl->mv_cache[list][ h264_scan8[0] ], 4, 4, 8, H264_PACK16TO32(mx,my), 4);
                }
            }
        }
        else if(IS_16X8(mb_type)){
            for (list = 0; list < sl->list_count; list++) {
                    for(i=0; i<2; i++){
                        if(IS_DIR(mb_type, i, list)){
                            int ref;
                            int rc = sl->ref_count[list] << MB_MBAFF(sl);
                            if (rc > 1) {
                                ref= decode_cabac_mb_ref(sl, list, 8 * i);
                                if (ref >= rc) {
                                                                        return -1;
                                }
                            }else
                                ref=0;
                            h264_fill_rectangle(&sl->ref_cache[list][ h264_scan8[0] + 16*i ], 4, 2, 8, ref, 1);
                        }else
                            h264_fill_rectangle(&sl->ref_cache[list][ h264_scan8[0] + 16*i ], 4, 2, 8, (LIST_NOT_USED&0xFF), 1);
                    }
            }
            for (list = 0; list < sl->list_count; list++) {
                for(i=0; i<2; i++){
                    if(IS_DIR(mb_type, i, list)){
                        int mx,my,mpx,mpy;
                        h264_pred_16x8_motion(h, sl, 8*i, list, sl->ref_cache[list][h264_scan8[0] + 16*i], &mx, &my);
                        DECODE_CABAC_MB_MVD(sl, list, 8*i)

                        h264_fill_rectangle(sl->mvd_cache[list][ h264_scan8[0] + 16*i ], 4, 2, 8, pack8to16(mpx,mpy), 2);
                        h264_fill_rectangle(sl->mv_cache[list][ h264_scan8[0] + 16*i ], 4, 2, 8, H264_PACK16TO32(mx,my), 4);
                    }else{
                        h264_fill_rectangle(sl->mvd_cache[list][ h264_scan8[0] + 16*i ], 4, 2, 8, 0, 2);
                        h264_fill_rectangle(sl->mv_cache[list][ h264_scan8[0] + 16*i ], 4, 2, 8, 0, 4);
                    }
                }
            }
        }else{
            for (list = 0; list < sl->list_count; list++) {
                    for(i=0; i<2; i++){
                        if(IS_DIR(mb_type, i, list)){ //FIXME optimize
                            int ref;
                            int rc = sl->ref_count[list] << MB_MBAFF(sl);
                            if (rc > 1) {
                                ref = decode_cabac_mb_ref(sl, list, 4 * i);
                                if (ref >= rc) {
                                                                        return -1;
                                }
                            }else
                                ref=0;
                            h264_fill_rectangle(&sl->ref_cache[list][ h264_scan8[0] + 2*i ], 2, 4, 8, ref, 1);
                        }else
                            h264_fill_rectangle(&sl->ref_cache[list][ h264_scan8[0] + 2*i ], 2, 4, 8, (LIST_NOT_USED&0xFF), 1);
                    }
            }
            for (list = 0; list < sl->list_count; list++) {
                for(i=0; i<2; i++){
                    if(IS_DIR(mb_type, i, list)){
                        int mx,my,mpx,mpy;
                        h264_pred_8x16_motion(h, sl, i*4, list, sl->ref_cache[list][ h264_scan8[0] + 2*i ], &mx, &my);
                        DECODE_CABAC_MB_MVD(sl, list, 4*i)

                        h264_fill_rectangle(sl->mvd_cache[list][ h264_scan8[0] + 2*i ], 2, 4, 8, pack8to16(mpx,mpy), 2);
                        h264_fill_rectangle(sl->mv_cache[list][ h264_scan8[0] + 2*i ], 2, 4, 8, H264_PACK16TO32(mx,my), 4);
                    }else{
                        h264_fill_rectangle(sl->mvd_cache[list][ h264_scan8[0] + 2*i ], 2, 4, 8, 0, 2);
                        h264_fill_rectangle(sl->mv_cache[list][ h264_scan8[0] + 2*i ], 2, 4, 8, 0, 4);
                    }
                }
            }
        }
    }

   if( IS_INTER( mb_type ) ) {
        h->cur_pic->chroma_pred_mode[mb_xy] = 0;
        h264_write_back_motion(h, sl, mb_type);
   }

    if( !IS_INTRA16x16( mb_type ) ) {
        cbp  = decode_cabac_mb_cbp_luma(sl);
        if(decode_chroma)
            cbp |= decode_cabac_mb_cbp_chroma(sl) << 4;
    } else {
        if (!decode_chroma && cbp>15) {
                        return -1;
        }
    }

    h->cur_pic->cbp_table[mb_xy] = sl->cbp = cbp;

    if( dct8x8_allowed && (cbp&15) && !IS_INTRA( mb_type ) ) {
        mb_type |= MB_TYPE_8x8DCT * get_cabac_noinline(&sl->cabac, &sl->cabac_state[399 + sl->neighbor_transform_size]);
    }

    /* It would be better to do this in fill_decode_caches, but we don't know
     * the transform mode of the current macroblock there. */
    if (h->active_sps->chroma_format_idc == 3 && IS_8x8DCT(mb_type)) {
        int i;
        uint8_t *nnz_cache = sl->non_zero_count_cache;
        if ((unsigned)h->x264_build < 151U) {
            for (i = 0; i < 2; i++) {
                if (sl->left_type[i] && !IS_8x8DCT(sl->left_type[i])) {
                    nnz_cache[3 + 8 *  1 + 2 * 8 * i] =
                    nnz_cache[3 + 8 *  2 + 2 * 8 * i] =
                    nnz_cache[3 + 8 *  6 + 2 * 8 * i] =
                    nnz_cache[3 + 8 *  7 + 2 * 8 * i] =
                    nnz_cache[3 + 8 * 11 + 2 * 8 * i] =
                    nnz_cache[3 + 8 * 12 + 2 * 8 * i] = IS_INTRA(mb_type) ? 64 : 0;
                }
            }
            if (sl->top_type && !IS_8x8DCT(sl->top_type)) {
                uint32_t top_empty = !IS_INTRA(mb_type) ? 0 : 0x40404040u;
                memcpy(&nnz_cache[4 + 8 *  0], &top_empty, 4);
                memcpy(&nnz_cache[4 + 8 *  5], &top_empty, 4);
                memcpy(&nnz_cache[4 + 8 * 10], &top_empty, 4);
            }
        } else {
            for (i = 0; i < 2; i++) {
                if (sl->left_type[i] && !IS_8x8DCT(sl->left_type[i])) {
                    nnz_cache[3 + 8 *  1 + 2 * 8 * i] =
                    nnz_cache[3 + 8 *  2 + 2 * 8 * i] =
                    nnz_cache[3 + 8 *  6 + 2 * 8 * i] =
                    nnz_cache[3 + 8 *  7 + 2 * 8 * i] =
                    nnz_cache[3 + 8 * 11 + 2 * 8 * i] =
                    nnz_cache[3 + 8 * 12 + 2 * 8 * i] =
                        !IS_INTRA_PCM(sl->left_type[i]) ? 0 : 64;
                }
            }
            if (sl->top_type && !IS_8x8DCT(sl->top_type)) {
                uint32_t top_empty = !IS_INTRA_PCM(sl->top_type) ? 0 : 0x40404040u;
                memcpy(&nnz_cache[4 + 8 *  0], &top_empty, 4);
                memcpy(&nnz_cache[4 + 8 *  5], &top_empty, 4);
                memcpy(&nnz_cache[4 + 8 * 10], &top_empty, 4);
            }
        }
    }
    h->cur_pic->mb_type[mb_xy] = mb_type;

    if( cbp || IS_INTRA16x16( mb_type ) ) {
        const uint8_t *scan, *scan8x8;
        const uint32_t *qmul;

        // decode_cabac_mb_dqp
        if(get_cabac_noinline( &sl->cabac, &sl->cabac_state[60 + (sl->last_qscale_diff != 0)])){
            int val = 1;
            int ctx= 2;
            const int max_qp = 51 + 6*(sps->bit_depth_luma-8);

            while( get_cabac_noinline( &sl->cabac, &sl->cabac_state[60 + ctx] ) ) {
                ctx= 3;
                val++;
                if(val > 2*max_qp){ //prevent infinite loop
                                        return -1;
                }
            }

            if( val&0x01 )
                val=   (val + 1)>>1 ;
            else
                val= -((val + 1)>>1);
            sl->last_qscale_diff = val;
            sl->qscale += val;
            if ((unsigned)sl->qscale > (unsigned)max_qp){
                if (sl->qscale < 0) sl->qscale += max_qp + 1;
                else                sl->qscale -= max_qp + 1;
            }
            sl->chroma_qp[0] = h264_get_chroma_qp(h->active_pps, 0, sl->qscale);
            sl->chroma_qp[1] = h264_get_chroma_qp(h->active_pps, 1, sl->qscale);
        }else
            sl->last_qscale_diff=0;

        if(IS_INTERLACED(mb_type)){
            scan8x8 = sl->qscale ? h->field_scan8x8 : h->field_scan8x8_q0;
            scan    = sl->qscale ? h->field_scan : h->field_scan_q0;
        }else{
            scan8x8 = sl->qscale ? h->zigzag_scan8x8 : h->zigzag_scan8x8_q0;
            scan    = sl->qscale ? h->zigzag_scan : h->zigzag_scan_q0;
        }

        decode_cabac_luma_residual(h, sl, scan, scan8x8, pixel_shift, mb_type, cbp, 0);
        if (h->active_sps->chroma_format_idc == 3) {
            decode_cabac_luma_residual(h, sl, scan, scan8x8, pixel_shift, mb_type, cbp, 1);
            decode_cabac_luma_residual(h, sl, scan, scan8x8, pixel_shift, mb_type, cbp, 2);
        } else if ((h->active_sps->chroma_format_idc == 2)) {
            if( cbp&0x30 ){
                int c;
                for (c = 0; c < 2; c++)
                    decode_cabac_residual_dc_422(h, sl, sl->mb + ((256 + 16*16*c) << pixel_shift), 3,
                                                 CHROMA_DC_BLOCK_INDEX + c,
                                                 h264_chroma422_dc_scan, 8);
            }

            if( cbp&0x20 ) {
                int c, i, i8x8;
                for( c = 0; c < 2; c++ ) {
                    int16_t *mb = sl->mb + (16*(16 + 16*c) << pixel_shift);
                    qmul = h->active_pps->dequant4_coeff[c+1+(IS_INTRA( mb_type ) ? 0:3)][sl->chroma_qp[c]];
                    for (i8x8 = 0; i8x8 < 2; i8x8++) {
                        for (i = 0; i < 4; i++) {
                            const int index = 16 + 16 * c + 8*i8x8 + i;
                            decode_cabac_residual_nondc(h, sl, mb, 4, index, scan + 1, qmul, 15);
                            mb += 16<<pixel_shift;
                        }
                    }
                }
            } else {
                h264_fill_rectangle(&sl->non_zero_count_cache[h264_scan8[16]], 4, 4, 8, 0, 1);
                h264_fill_rectangle(&sl->non_zero_count_cache[h264_scan8[32]], 4, 4, 8, 0, 1);
            }
        } else /* yuv420 */ {
            if( cbp&0x30 ){
                int c;
                for (c = 0; c < 2; c++)
                    decode_cabac_residual_dc(h, sl, sl->mb + ((256 + 16 * 16 * c) << pixel_shift),
                                             3, CHROMA_DC_BLOCK_INDEX + c, h264_chroma_dc_scan, 4);
            }

            if( cbp&0x20 ) {
                int c, i;
                for( c = 0; c < 2; c++ ) {
                    qmul = h->active_pps->dequant4_coeff[c+1+(IS_INTRA( mb_type ) ? 0:3)][sl->chroma_qp[c]];
                    for( i = 0; i < 4; i++ ) {
                        const int index = 16 + 16 * c + i;
                        decode_cabac_residual_nondc(h, sl, sl->mb + (16*index << pixel_shift), 4, index, scan + 1, qmul, 15);
                    }
                }
            } else {
                h264_fill_rectangle(&sl->non_zero_count_cache[h264_scan8[16]], 4, 4, 8, 0, 1);
                h264_fill_rectangle(&sl->non_zero_count_cache[h264_scan8[32]], 4, 4, 8, 0, 1);
            }
        }
    } else {
        h264_fill_rectangle(&sl->non_zero_count_cache[h264_scan8[ 0]], 4, 4, 8, 0, 1);
        h264_fill_rectangle(&sl->non_zero_count_cache[h264_scan8[16]], 4, 4, 8, 0, 1);
        h264_fill_rectangle(&sl->non_zero_count_cache[h264_scan8[32]], 4, 4, 8, 0, 1);
        sl->last_qscale_diff = 0;
    }

    h->cur_pic->qscale_table[mb_xy] = sl->qscale;
    h264_write_back_non_zero_count(h, sl);

    return 0;
}
