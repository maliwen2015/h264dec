/*
 * Print the picture metadata the decoder exports, one CSV line per picture,
 * in the order the decoder hands them out. tests/meta.sh compares the field
 * order columns against ffprobe, which derives them independently.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "h264dec.h"

static int start_code_len(const uint8_t *d, int i, int size)
{
    if (i + 3 > size || d[i] || d[i + 1])
        return 0;
    if (d[i + 2] == 1)
        return 3;
    if (i + 4 <= size && d[i + 2] == 0 && d[i + 3] == 1)
        return 4;
    return 0;
}

/* ue(0) is the single one bit, so a slice whose first payload bit is set
 * starts at macroblock zero and therefore begins an access unit */
static int is_au_start(const uint8_t *d, int off, int end)
{
    int type = d[off] & 0x1f;
    return (type == 1 || type == 5) && off + 1 < end && (d[off + 1] & 0x80);
}

/* Offsets where the access units begin, each one pulling in the parameter
 * sets and SEI messages that precede its first slice. */
static int is_slice_nal(const uint8_t *d, int off)
{
    int type = d[off] & 0x1f;
    return type >= 1 && type <= 5;
}

static int find_access_units(const uint8_t *data, int size, int **out)
{
    int cap = 64, nb = 0, n = 0, i;
    int *pos = (int *)malloc((size_t)cap * sizeof(int));   /* NAL header byte */
    int *scs = (int *)malloc((size_t)cap * sizeof(int));   /* start code */
    int *starts;

    if (!pos || !scs) {
        free(pos);
        free(scs);
        *out = NULL;
        return 0;
    }
    for (i = 0; i + 3 <= size; i++) {
        int sc = start_code_len(data, i, size);
        if (!sc || i + sc >= size)
            continue;
        if (nb && pos[nb - 1] == i + sc)
            continue;                    /* 4-byte code seen again as 3-byte */
        if (nb == cap) {
            int *t;
            cap *= 2;
            t = (int *)realloc(pos, (size_t)cap * sizeof(int));
            if (!t)
                break;
            pos = t;
            t = (int *)realloc(scs, (size_t)cap * sizeof(int));
            if (!t)
                break;
            scs = t;
        }
        pos[nb] = i + sc;
        scs[nb] = i;
        nb++;
    }
    starts = (int *)malloc((size_t)(nb + 1) * sizeof(int));
    if (!starts) {
        free(pos);
        free(scs);
        *out = NULL;
        return 0;
    }
    for (i = 0; i < nb; i++) {
        int end = (i + 1 < nb) ? pos[i + 1] : size;
        int j = i;
        if (!is_au_start(data, pos[i], end))
            continue;
        while (j > 0 && !is_slice_nal(data, pos[j - 1]))
            j--;
        if (n == 0 || starts[n - 1] != scs[j])
            starts[n++] = scs[j];
    }
    free(pos);
    free(scs);
    *out = starts;
    return n;
}
static void dump(FILE *out, int *n, const h264_picture_t *pic)
{
    fprintf(out, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
            (*n)++, pic->pic_order_count, pic->type, pic->interlaced,
            pic->top_field_first, pic->repeat_pict, pic->pic_struct,
            pic->cpb_removal_delay, pic->dpb_output_delay,
            pic->recovery_frame_cnt, pic->broken_recovery,
            pic->timecode_valid, pic->timecode_hours, pic->timecode_minutes,
            pic->timecode_seconds, pic->timecode_frames, pic->timecode_drop);
}

int main(int argc, char **argv)
{
    const char *file = argv[1];
    int threads = argc > 2 ? atoi(argv[2]) : 1;
    int order = argc > 3 ? atoi(argv[3]) : 1;      /* 1 = decode order */
    FILE *f;
    long size;
    uint8_t *buf;
    int *au, nb_au, i, n = 0;
    h264_param_t param;
    h264_packet_t pkt;
    h264_stream_info info;
    h264_picture_t pic;
    void *dec;

    if (argc < 2) {
        printf("usage: %s <in.h264> [threads] [output_order]\n", argv[0]);
        return 2;
    }
    f = fopen(file, "rb");
    if (!f) {
        printf("cannot open %s\n", file);
        return 2;
    }
    fseek(f, 0, SEEK_END);
    size = ftell(f);
    fseek(f, 0, SEEK_SET);
    buf = (unsigned char *)malloc((size_t)size + 1);
    if (!buf || fread(buf, 1, (size_t)size, f) != (size_t)size) {
        printf("short read\n");
        return 2;
    }
    fclose(f);

    nb_au = find_access_units(buf, (int)size, &au);
    if (nb_au <= 0 || !au) {
        printf("no access unit found\n");
        return 1;
    }

    memset(&param, 0, sizeof(param));
    param.threads = threads;
    param.output_order = order;
    dec = h264_dec_open(&param);
    if (!dec) {
        printf("cannot open decoder\n");
        return 2;
    }
    memset(&pkt, 0, sizeof(pkt));
    for (i = 0; i < nb_au; i++) {
        int end = (i + 1 < nb_au) ? au[i + 1] : (int)size;
        pkt.data = buf + au[i];
        pkt.len = end - au[i];
        if (h264_dec_send_packet(dec, &pkt) < 0)
            fprintf(stderr, "access unit at %d failed\n", au[i]);
        while (h264_dec_recv_frame(dec, &info, &pic) == H264_DEC_GOT_FRAME) {
            dump(stdout, &n, &pic);
            h264_dec_frame_unref(dec, &pic);
        }
    }
    while (h264_dec_flush(dec, &info, &pic) == H264_DEC_GOT_FRAME) {
        dump(stdout, &n, &pic);
        h264_dec_frame_unref(dec, &pic);
    }
    h264_dec_close(dec);
    free(buf);
    free(au);
    fprintf(stderr, "%d pictures\n", n);
    return n > 0 ? 0 : 1;
}
