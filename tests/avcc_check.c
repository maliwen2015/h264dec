/*
 * Length prefixed (AVCC) input check: rewrite an Annex-B stream as
 * 4-byte-length-prefixed NALs in memory, decode that through the same API and
 * write the YUV, which must match the Annex-B decode byte for byte.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "h264dec.h"

static int start_code_len(const unsigned char *d, int i, int size)
{
    if (i + 3 > size || d[i] || d[i + 1])
        return 0;
    if (d[i + 2] == 1)
        return 3;
    if (i + 4 <= size && d[i + 2] == 0 && d[i + 3] == 1)
        return 4;
    return 0;
}

static void write_pic(FILE *o, const h264_picture_t *pic)
{
    int i;
    for (i = 0; i < pic->lines[0]; i++)
        fwrite(pic->planes[0] + i * pic->strides[0], 1, pic->widths[0], o);
    for (i = 0; i < pic->lines[1]; i++)
        fwrite(pic->planes[1] + i * pic->strides[1], 1, pic->widths[1], o);
    for (i = 0; i < pic->lines[2]; i++)
        fwrite(pic->planes[2] + i * pic->strides[2], 1, pic->widths[2], o);
}

static void drain(void *dec, FILE *o, h264_stream_info *info, int *frames)
{
    h264_picture_t pic;
    while (h264_dec_recv_frame(dec, info, &pic) == H264_DEC_GOT_FRAME) {
        write_pic(o, &pic);
        h264_dec_frame_unref(dec, &pic);
        (*frames)++;
    }
}

int main(int argc, char **argv)
{
    FILE *f;
    long size;
    unsigned char *in, *avcc;
    int pos = 0, out = 0, nals = 0;
    h264_param_t param;
    h264_packet_t pkt;
    h264_stream_info info;
    h264_picture_t pic;
    void *dec;
    int frames = 0, nb_au = 0, *au;

    if (argc < 3) {
        printf("usage: %s <in.h264> <out.yuv>\n", argv[0]);
        return 2;
    }
    f = fopen(argv[1], "rb");
    if (!f) {
        printf("cannot open %s\n", argv[1]);
        return 2;
    }
    fseek(f, 0, SEEK_END);
    size = ftell(f);
    fseek(f, 0, SEEK_SET);
    in = (unsigned char *)malloc((size_t)size + 1);
    if (fread(in, 1, (size_t)size, f) != (size_t)size) {
        printf("short read\n");
        return 2;
    }
    fclose(f);

    /* every NAL grows by a 4 byte length, and a NAL is at least one byte after
     * a three byte start code, so twice the input size is room enough */
    avcc = (unsigned char *)malloc((size_t)size * 2 + 64);
    au = (int *)malloc(((size_t)size / 4 + 2) * sizeof(int));
    if (!avcc || !au) {
        printf("out of memory\n");
        return 2;
    }
    au[0] = 0;        /* the leading parameter sets ride in the first packet */
    nb_au = 1;
    while (pos < size) {
        int sc = start_code_len(in, pos, (int)size);
        int start, i;
        long len;
        unsigned char *p;

        if (!sc) {
            pos++;
            continue;
        }
        start = pos + sc;
        while (start < size && in[start] == 0 && start < size)
            start++;                      /* extra zeros of a 4 byte code */
        /* the next start code ends this NAL, minus its trailing zeros */
        len = size - start;
        for (i = start; i + 2 < size; i++) {
            if (start_code_len(in, i, (int)size)) {
                len = i - start;
                break;
            }
        }
        while (len > 0 && in[start + len - 1] == 0)
            len--;
        if (len <= 0) {
            pos = start;
            continue;
        }
        if (((in[start] & 0x1f) == 1 || (in[start] & 0x1f) == 5) &&
            (in[start + 1] & 0x80) && out != au[nb_au - 1])
            au[nb_au++] = out;     /* first slice of an access unit */
        p = avcc + out;
        p[0] = (unsigned char)((len >> 24) & 0xff);
        p[1] = (unsigned char)((len >> 16) & 0xff);
        p[2] = (unsigned char)((len >> 8) & 0xff);
        p[3] = (unsigned char)(len & 0xff);
        memcpy(p + 4, in + start, (size_t)len);
        out += 4 + (int)len;
        nals++;
        pos = (int)(start + len);
    }

    memset(&param, 0, sizeof(param));
    param.nal_length_size = 4;
    param.threads = 1;
    dec = h264_dec_open(&param);
    if (!dec) {
        printf("cannot open decoder\n");
        return 2;
    }
    {
        FILE *o = fopen(argv[2], "wb");
        int k;
        if (!o) {
            printf("cannot write %s\n", argv[2]);
            return 2;
        }
        memset(&pkt, 0, sizeof(pkt));
        for (k = 0; k < nb_au; k++) {
            int begin = au[k];
            int end = (k + 1 < nb_au) ? au[k + 1] : out;
            pkt.data = avcc + begin;
            pkt.len = end - begin;
            if (h264_dec_send_packet(dec, &pkt) < 0)
                printf("avcc: access unit %d failed\n", k);
            drain(dec, o, &info, &frames);
        }
        while (h264_dec_flush(dec, &info, &pic) == H264_DEC_GOT_FRAME) {
            write_pic(o, &pic);
            h264_dec_frame_unref(dec, &pic);
            frames++;
        }
        fclose(o);
    }
    h264_dec_close(dec);
    printf("%s: %d NALs, %d frames (%dx%d)\n", argv[1], nals, frames,
           info.width, info.height);
    free(in);
    free(avcc);
    return frames > 0 ? 0 : 1;
}
