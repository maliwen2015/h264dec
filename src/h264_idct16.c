/*
 * H.264 inverse transforms (8-bit).
 * Ported from FFmpeg's h264idct_template.c (LGPL-2.1+), matching its
 * integer behaviour exactly.
 */
#include "h264_internal.h"

static inline uint16_t clip10(int v)
{
    return (uint16_t)(v < 0 ? 0 : v > 1023 ? 1023 : v);
}

void h264_idct_add_16(uint16_t *dst, int32_t *block, int stride)
{
    int i;
    block[0] += 1 << 5;
    for (i = 0; i < 4; i++) {
        const uint32_t z0 = (uint32_t)block[i + 4 * 0] + (uint32_t)block[i + 4 * 2];
        const uint32_t z1 = (uint32_t)block[i + 4 * 0] - (uint32_t)block[i + 4 * 2];
        const uint32_t z2 = (uint32_t)(block[i + 4 * 1] >> 1) - (uint32_t)block[i + 4 * 3];
        const uint32_t z3 = (uint32_t)block[i + 4 * 1] + (uint32_t)(block[i + 4 * 3] >> 1);

        block[i + 4 * 0] = (int32_t)(z0 + z3);
        block[i + 4 * 1] = (int32_t)(z1 + z2);
        block[i + 4 * 2] = (int32_t)(z1 - z2);
        block[i + 4 * 3] = (int32_t)(z0 - z3);
    }
    for (i = 0; i < 4; i++) {
        const uint32_t z0 = (uint32_t)block[0 + 4 * i] + (uint32_t)block[2 + 4 * i];
        const uint32_t z1 = (uint32_t)block[0 + 4 * i] - (uint32_t)block[2 + 4 * i];
        const uint32_t z2 = (uint32_t)(block[1 + 4 * i] >> 1) - (uint32_t)block[3 + 4 * i];
        const uint32_t z3 = (uint32_t)block[1 + 4 * i] + (uint32_t)(block[3 + 4 * i] >> 1);

        dst[i + 0 * stride] = clip10(dst[i + 0 * stride] + ((int)(z0 + z3) >> 6));
        dst[i + 1 * stride] = clip10(dst[i + 1 * stride] + ((int)(z1 + z2) >> 6));
        dst[i + 2 * stride] = clip10(dst[i + 2 * stride] + ((int)(z1 - z2) >> 6));
        dst[i + 3 * stride] = clip10(dst[i + 3 * stride] + ((int)(z0 - z3) >> 6));
    }
    memset(block, 0, 16 * sizeof(int32_t));
}

void h264_idct_dc_add_16(uint16_t *dst, int32_t *block, int stride)
{
    int i, dc = (block[0] + 32) >> 6;
    block[0] = 0;
    for (i = 0; i < 4; i++) {
        dst[0] = clip10(dst[0] + dc);
        dst[1] = clip10(dst[1] + dc);
        dst[2] = clip10(dst[2] + dc);
        dst[3] = clip10(dst[3] + dc);
        dst += stride;
    }
}

void h264_idct8_add_16(uint16_t *dst, int32_t *block, int stride)
{
    int i;
    block[0] += 32;
    for (i = 0; i < 8; i++) {
        const uint32_t a0 = (uint32_t)block[i + 0 * 8] + (uint32_t)block[i + 4 * 8];
        const uint32_t a2 = (uint32_t)block[i + 0 * 8] - (uint32_t)block[i + 4 * 8];
        const uint32_t a4 = (uint32_t)(block[i + 2 * 8] >> 1) - (uint32_t)block[i + 6 * 8];
        const uint32_t a6 = (uint32_t)(block[i + 6 * 8] >> 1) + (uint32_t)block[i + 2 * 8];

        const uint32_t b0 = a0 + a6;
        const uint32_t b2 = a2 + a4;
        const uint32_t b4 = a2 - a4;
        const uint32_t b6 = a0 - a6;

        const int a1 = -(int)block[i + 3 * 8] + (uint32_t)block[i + 5 * 8] - block[i + 7 * 8] - (block[i + 7 * 8] >> 1);
        const int a3 =  (int)block[i + 1 * 8] + (uint32_t)block[i + 7 * 8] - block[i + 3 * 8] - (block[i + 3 * 8] >> 1);
        const int a5 = -(int)block[i + 1 * 8] + (uint32_t)block[i + 7 * 8] + block[i + 5 * 8] + (block[i + 5 * 8] >> 1);
        const int a7 =  (int)block[i + 3 * 8] + (uint32_t)block[i + 5 * 8] + block[i + 1 * 8] + (block[i + 1 * 8] >> 1);

        const int b1 = (a7 >> 2) + (uint32_t)a1;
        const int b3 = (uint32_t)a3 + (a5 >> 2);
        const int b5 = (a3 >> 2) - (uint32_t)a5;
        const int b7 = (uint32_t)a7 - (a1 >> 2);

        block[i + 0 * 8] = (int32_t)(b0 + b7);
        block[i + 7 * 8] = (int32_t)(b0 - b7);
        block[i + 1 * 8] = (int32_t)(b2 + b5);
        block[i + 6 * 8] = (int32_t)(b2 - b5);
        block[i + 2 * 8] = (int32_t)(b4 + b3);
        block[i + 5 * 8] = (int32_t)(b4 - b3);
        block[i + 3 * 8] = (int32_t)(b6 + b1);
        block[i + 4 * 8] = (int32_t)(b6 - b1);
    }
    for (i = 0; i < 8; i++) {
        const uint32_t a0 = (uint32_t)block[0 + i * 8] + (uint32_t)block[4 + i * 8];
        const uint32_t a2 = (uint32_t)block[0 + i * 8] - (uint32_t)block[4 + i * 8];
        const uint32_t a4 = (uint32_t)(block[2 + i * 8] >> 1) - (uint32_t)block[6 + i * 8];
        const uint32_t a6 = (uint32_t)(block[6 + i * 8] >> 1) + (uint32_t)block[2 + i * 8];

        const uint32_t b0 = a0 + a6;
        const uint32_t b2 = a2 + a4;
        const uint32_t b4 = a2 - a4;
        const uint32_t b6 = a0 - a6;

        const int a1 = -(int)block[3 + i * 8] + block[5 + i * 8] - block[7 + i * 8] - (block[7 + i * 8] >> 1);
        const int a3 =  (int)block[1 + i * 8] + block[7 + i * 8] - block[3 + i * 8] - (block[3 + i * 8] >> 1);
        const int a5 = -(int)block[1 + i * 8] + block[7 + i * 8] + block[5 + i * 8] + (block[5 + i * 8] >> 1);
        const int a7 =  (int)block[3 + i * 8] + block[5 + i * 8] + block[1 + i * 8] + (block[1 + i * 8] >> 1);

        const uint32_t b1 = (uint32_t)(a7 >> 2) + (uint32_t)a1;
        const uint32_t b3 = (uint32_t)a3 + (uint32_t)(a5 >> 2);
        const uint32_t b5 = (uint32_t)(a3 >> 2) - (uint32_t)a5;
        const uint32_t b7 = (uint32_t)a7 - (uint32_t)(a1 >> 2);

        dst[i + 0 * stride] = clip10(dst[i + 0 * stride] + ((int)(b0 + b7) >> 6));
        dst[i + 1 * stride] = clip10(dst[i + 1 * stride] + ((int)(b2 + b5) >> 6));
        dst[i + 2 * stride] = clip10(dst[i + 2 * stride] + ((int)(b4 + b3) >> 6));
        dst[i + 3 * stride] = clip10(dst[i + 3 * stride] + ((int)(b6 + b1) >> 6));
        dst[i + 4 * stride] = clip10(dst[i + 4 * stride] + ((int)(b6 - b1) >> 6));
        dst[i + 5 * stride] = clip10(dst[i + 5 * stride] + ((int)(b4 - b3) >> 6));
        dst[i + 6 * stride] = clip10(dst[i + 6 * stride] + ((int)(b2 - b5) >> 6));
        dst[i + 7 * stride] = clip10(dst[i + 7 * stride] + ((int)(b0 - b7) >> 6));
    }
    memset(block, 0, 64 * sizeof(int32_t));
}

void h264_idct8_dc_add_16(uint16_t *dst, int32_t *block, int stride)
{
    int i, j, dc = (block[0] + 32) >> 6;
    block[0] = 0;
    for (j = 0; j < 8; j++) {
        for (i = 0; i < 8; i++)
            dst[i] = clip10(dst[i] + dc);
        dst += stride;
    }
}

void h264_idct_add16_16(uint16_t *dst, const int *block_offset, int32_t *block,
                     int stride, const uint8_t nnzc[5 * 8])
{
    int i;
    for (i = 0; i < 16; i++) {
        int nnz = nnzc[h264_scan8[i]];
        if (nnz) {
            if (nnz == 1 && block[i * 16])
                h264_idct_dc_add_16(dst + block_offset[i], block + i * 16, stride);
            else
                h264_idct_add_16(dst + block_offset[i], block + i * 16, stride);
        }
    }
}

void h264_idct_add16intra_16(uint16_t *dst, const int *block_offset, int32_t *block,
                          int stride, const uint8_t nnzc[5 * 8])
{
    int i;
    for (i = 0; i < 16; i++) {
        if (nnzc[h264_scan8[i]])
            h264_idct_add_16(dst + block_offset[i], block + i * 16, stride);
        else if (block[i * 16])
            h264_idct_dc_add_16(dst + block_offset[i], block + i * 16, stride);
    }
}

void h264_idct8_add4_16(uint16_t *dst, const int *block_offset, int32_t *block,
                     int stride, const uint8_t nnzc[5 * 8])
{
    int i;
    for (i = 0; i < 16; i += 4) {
        int nnz = nnzc[h264_scan8[i]];
        if (nnz) {
            if (nnz == 1 && block[i * 16])
                h264_idct8_dc_add_16(dst + block_offset[i], block + i * 16, stride);
            else
                h264_idct8_add_16(dst + block_offset[i], block + i * 16, stride);
        }
    }
}

void h264_idct_add8_16(uint16_t **dest, const int *block_offset, int32_t *block,
                    int stride, const uint8_t nnzc[15 * 8])
{
    int i, j;
    for (j = 1; j < 3; j++) {
        for (i = j * 16; i < j * 16 + 4; i++) {
            if (nnzc[h264_scan8[i]])
                h264_idct_add_16(dest[j - 1] + block_offset[i], block + i * 16, stride);
            else if (block[i * 16])
                h264_idct_dc_add_16(dest[j - 1] + block_offset[i], block + i * 16, stride);
        }
    }
}

void h264_idct_add8_422_16(uint16_t **dest, const int *block_offset, int32_t *block,
                           int stride, const uint8_t nnzc[15 * 8])
{
    int i, j;
    for (j = 1; j < 3; j++) {
        for (i = j * 16; i < j * 16 + 4; i++) {
            if (nnzc[h264_scan8[i]])
                h264_idct_add_16(dest[j - 1] + block_offset[i], block + i * 16, stride);
            else if (block[i * 16])
                h264_idct_dc_add_16(dest[j - 1] + block_offset[i], block + i * 16, stride);
        }
    }
    for (j = 1; j < 3; j++) {
        for (i = j * 16 + 4; i < j * 16 + 8; i++) {
            if (nnzc[h264_scan8[i + 4]])
                h264_idct_add_16(dest[j - 1] + block_offset[i + 4], block + i * 16, stride);
            else if (block[i * 16])
                h264_idct_dc_add_16(dest[j - 1] + block_offset[i + 4], block + i * 16, stride);
        }
    }
}

void h264_luma_dc_dequant_idct_16(int32_t *output, int32_t *input, int qmul)
{
    int i;
    static const uint16_t x_offset[4] = { 0, 2 * 16, 8 * 16, 10 * 16 };
    int temp[16];

    for (i = 0; i < 4; i++) {
        const int z0 = input[4 * i + 0] + input[4 * i + 1];
        const int z1 = input[4 * i + 0] - input[4 * i + 1];
        const int z2 = input[4 * i + 2] - input[4 * i + 3];
        const int z3 = input[4 * i + 2] + input[4 * i + 3];

        temp[4 * i + 0] = z0 + z3;
        temp[4 * i + 1] = z0 - z3;
        temp[4 * i + 2] = z1 - z2;
        temp[4 * i + 3] = z1 + z2;
    }
    for (i = 0; i < 4; i++) {
        const int offset = x_offset[i];
        const uint32_t z0 = (uint32_t)temp[4 * 0 + i] + (uint32_t)temp[4 * 2 + i];
        const uint32_t z1 = (uint32_t)temp[4 * 0 + i] - (uint32_t)temp[4 * 2 + i];
        const uint32_t z2 = (uint32_t)temp[4 * 1 + i] - (uint32_t)temp[4 * 3 + i];
        const uint32_t z3 = (uint32_t)temp[4 * 1 + i] + (uint32_t)temp[4 * 3 + i];

        output[16 * 0 + offset] = (int32_t)((int32_t)((z0 + z3) * (uint32_t)qmul + 128u) >> 8);
        output[16 * 1 + offset] = (int32_t)((int32_t)((z1 + z2) * (uint32_t)qmul + 128u) >> 8);
        output[16 * 4 + offset] = (int32_t)((int32_t)((z1 - z2) * (uint32_t)qmul + 128u) >> 8);
        output[16 * 5 + offset] = (int32_t)((int32_t)((z0 - z3) * (uint32_t)qmul + 128u) >> 8);
    }
}

void h264_chroma_dc_dequant_idct_16(int32_t *block, int qmul)
{
    const int stride = 16 * 2;
    const int xStride = 16;
    uint32_t a, b, c, d, e;

    a = (uint32_t)block[stride * 0 + xStride * 0];
    b = (uint32_t)block[stride * 0 + xStride * 1];
    c = (uint32_t)block[stride * 1 + xStride * 0];
    d = (uint32_t)block[stride * 1 + xStride * 1];

    e = a - b;
    a = a + b;
    b = c - d;
    c = c + d;

    block[stride * 0 + xStride * 0] = (int32_t)((int32_t)((a + c) * (uint32_t)qmul) >> 7);
    block[stride * 0 + xStride * 1] = (int32_t)((int32_t)((e + b) * (uint32_t)qmul) >> 7);
    block[stride * 1 + xStride * 0] = (int32_t)((int32_t)((a - c) * (uint32_t)qmul) >> 7);
    block[stride * 1 + xStride * 1] = (int32_t)((int32_t)((e - b) * (uint32_t)qmul) >> 7);
}

void h264_chroma422_dc_dequant_idct_16(int32_t *block, int qmul)
{
    const int stride = 16 * 2;
    const int xStride = 16;
    int i;
    uint32_t temp[8];
    static const uint8_t x_offset[2] = { 0, 16 };

    for (i = 0; i < 4; i++) {
        temp[2 * i + 0] = (uint32_t)block[stride * i + xStride * 0] +
                          (uint32_t)block[stride * i + xStride * 1];
        temp[2 * i + 1] = (uint32_t)block[stride * i + xStride * 0] -
                          (uint32_t)block[stride * i + xStride * 1];
    }

    for (i = 0; i < 2; i++) {
        const int offset = x_offset[i];
        const uint32_t z0 = temp[2 * 0 + i] + temp[2 * 2 + i];
        const uint32_t z1 = temp[2 * 0 + i] - temp[2 * 2 + i];
        const uint32_t z2 = temp[2 * 1 + i] - temp[2 * 3 + i];
        const uint32_t z3 = temp[2 * 1 + i] + temp[2 * 3 + i];

        block[stride * 0 + offset] = (int32_t)((int32_t)((z0 + z3) * (uint32_t)qmul + 128u) >> 8);
        block[stride * 1 + offset] = (int32_t)((int32_t)((z1 + z2) * (uint32_t)qmul + 128u) >> 8);
        block[stride * 2 + offset] = (int32_t)((int32_t)((z1 - z2) * (uint32_t)qmul + 128u) >> 8);
        block[stride * 3 + offset] = (int32_t)((int32_t)((z0 - z3) * (uint32_t)qmul + 128u) >> 8);
    }
}

/* Lossless (qpprime_y_zero_transform_bypass): the residual is added straight
 * onto the prediction with 8-bit wraparound, which is exact because the
 * dequant tables for qp 0 are the identity (level * 64 + 32) >> 6. */
void h264_add_pixels4_16(uint16_t *dst, int32_t *block, int stride)
{
    int i;
    int32_t *src = block;

    for (i = 0; i < 4; i++) {
        dst[0] += (unsigned)src[0];
        dst[1] += (unsigned)src[1];
        dst[2] += (unsigned)src[2];
        dst[3] += (unsigned)src[3];
        dst += stride;
        src += 4;
    }
    memset(block, 0, 16 * sizeof(int32_t));
}

void h264_add_pixels8_16(uint16_t *dst, int32_t *block, int stride)
{
    int i, j;
    int32_t *src = block;

    for (i = 0; i < 8; i++) {
        for (j = 0; j < 8; j++)
            dst[j] += (unsigned)src[j];
        dst += stride;
        src += 8;
    }
    memset(block, 0, 64 * sizeof(int32_t));
}

