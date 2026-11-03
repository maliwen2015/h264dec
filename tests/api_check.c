/*
 * Public API contract test.
 *
 * The bit-exact suite and the fuzz harness drive the decoder the way the example
 * does: send, drain, unref. This file does the things an integrator really does
 * instead - call the entry points out of order, hand over empty or garbage
 * input, keep pictures in hand and give them back late - and requires the
 * library to answer with a documented return code, keep the pictures it does
 * hand out distinct, and produce exactly the same output whether a caller
 * releases each frame at once or cycles a handful of them.
 *
 *   build/api_check [stream.h264] [threads]
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "h264dec.h"

static int fails;

static void check(int ok, const char *fmt, ...)
{
    char what[160];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(what, sizeof(what), fmt, ap);
    va_end(ap);
    printf("%-48s %s\n", what, ok ? "ok" : "FAIL");
    if (!ok)
        fails++;
    fflush(stdout);
}

static uint8_t *read_file(const char *path, long *size_out)
{
    FILE *fp = fopen(path, "rb");
    long size;
    uint8_t *buf;

    if (!fp)
        return NULL;
    fseek(fp, 0, SEEK_END);
    size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    buf = (uint8_t *)malloc((size_t)size ? (size_t)size : 1);
    if (!buf || fread(buf, 1, (size_t)size, fp) != (size_t)size) {
        free(buf);
        fclose(fp);
        return NULL;
    }
    fclose(fp);
    *size_out = size;
    return buf;
}

static int send_all(void *dec, const uint8_t *data, int len, int64_t pts)
{
    h264_packet_t p;

    memset(&p, 0, sizeof(p));
    p.data = data;
    p.len = len;
    p.pts = pts;
    p.dts = pts;
    return h264_dec_send_packet(dec, &p);
}

/* one NAL per packet: the decoder finds the access-unit boundaries itself, so
 * this is the same legal call pattern just fed in the smallest pieces */
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

static uint32_t fnv1a(uint32_t h, const uint8_t *p, size_t n)
{
    size_t i;

    for (i = 0; i < n; i++) {
        h ^= p[i];
        h *= 0x01000193u;
    }
    return h;
}

/* pixels of one picture, hashed the way a caller would see them */
static uint32_t hash_frame(const h264_picture_t *pic)
{
    uint32_t h = 2166136261u;
    int i, p;

    for (p = 0; p < 3; p++)
        for (i = 0; i < pic->lines[p]; i++)
            h = fnv1a(h, pic->planes[p] + (size_t)i * pic->strides[p],
                      (size_t)pic->widths[p]);
    return fnv1a(h, (const uint8_t *)&pic->pic_order_count, sizeof(pic->pic_order_count));
}

/* The packet is the unit the decoder finalises a picture on, so a slice of a
 * multi-slice picture may not be sent on its own; these are the same access-unit
 * boundaries example_decoder.c computes. */
static int is_slice_nal(const uint8_t *d, int off)
{
    int type = d[off] & 0x1f;
    return type >= 1 && type <= 5;
}

static int is_au_first_slice(const uint8_t *d, int off, int end)
{
    int type = d[off] & 0x1f;
    return (type == 1 || type == 5) && off + 1 < end && (d[off + 1] & 0x80);
}

/* offsets at which the access units of the stream begin */
static int find_access_units(const uint8_t *data, int size, int **out)
{
    int cap = 16, nb = 0, n = 0, i;
    int *pos = (int *)malloc((size_t)cap * sizeof(int));
    int *scs = (int *)malloc((size_t)cap * sizeof(int));
    int *starts = (int *)malloc((size_t)(cap + 1) * sizeof(int));

    if (!pos || !scs || !starts) {
        free(pos); free(scs); free(starts);
        *out = NULL;
        return 0;
    }
    for (i = 0; i + 3 <= size; i++) {
        int sc = start_code_len(data, i, size);

        if (!sc || i + sc >= size)
            continue;
        if (nb && pos[nb - 1] == i + sc)
            continue;
        if (nb == cap) {
            int *t;
            cap *= 2;
            t = (int *)realloc(pos, (size_t)cap * sizeof(int));   if (!t) break; pos = t;
            t = (int *)realloc(scs, (size_t)cap * sizeof(int));   if (!t) break; scs = t;
            t = (int *)realloc(starts, (size_t)(cap + 1) * sizeof(int)); if (!t) break; starts = t;
        }
        pos[nb] = i + sc;
        scs[nb] = i;
        nb++;
    }
    for (i = 0; i < nb; i++) {
        int end = (i + 1 < nb) ? pos[i + 1] : size;
        int j = i;
        int need;

        if (!is_au_first_slice(data, pos[i], end))
            continue;
        while (j > 0 && !is_slice_nal(data, pos[j - 1]))
            j--;
        need = 1;
        if (n && starts[n - 1] == scs[j])
            need = 0;
        if (need)
            starts[n++] = scs[j];
    }
    free(pos);
    free(scs);
    *out = starts;
    return n;
}

#define HOLD_MAX 256
static h264_picture_t held[HOLD_MAX];

/*
 * Decode the stream while keeping at most `hold` pictures unreturned, and report
 * how many pictures came out and the hash of their pixels. hold == 0 is the
 * plain release-at-once path; anything above it has to agree with it exactly.
 */
static void run_mode(void *dec, const uint8_t *data, long size, int hold,
                     int *frames_out, uint32_t *hash_out)
{
    int frames = 0, live = 0, k, n_au;
    int *au;
    uint32_t hash = 2166136261u;

    n_au = find_access_units(data, (int)size, &au);
    if (!au) {
        *frames_out = 0;
        *hash_out = 0;
        return;
    }

    for (k = 0; k < n_au; k++) {
        int start = au[k];
        int end = (k + 1 < n_au) ? au[k + 1] : (int)size;
        h264_picture_t pic;
        h264_stream_info info;

        if (send_all(dec, data + start, end - start, start) < 0)
            break;

        for (;;) {
            memset(&pic, 0, sizeof(pic));
            if (h264_dec_recv_frame(dec, &info, &pic) != H264_DEC_GOT_FRAME)
                break;
            hash ^= hash_frame(&pic);
            frames++;
            if (hold <= 0) {
                h264_dec_frame_unref(dec, &pic);
                continue;
            }
            /* keep the newest `hold` pictures and return the oldest, which is
             * what a display pipeline that shows the last few frames does */
            if (live < hold) {
                held[live++] = pic;
            } else {
                h264_dec_frame_unref(dec, &held[0]);
                memmove(&held[0], &held[1], (size_t)(hold - 1) * sizeof(held[0]));
                held[hold - 1] = pic;
            }
        }
    }
    free(au);
    while (live > 0)
        h264_dec_frame_unref(dec, &held[--live]);
    {
        h264_stream_info info;
        h264_picture_t pic;

        while (h264_dec_flush(dec, &info, &pic) == H264_DEC_GOT_FRAME) {
            hash ^= hash_frame(&pic);
            frames++;
            h264_dec_frame_unref(dec, &pic);
        }
    }
    *frames_out = frames;
    *hash_out = hash;
}

int main(int argc, char **argv)
{
    const char *path = argc > 1 ? argv[1] : "../testdata/main_cabac.h264";
    const int threads = argc > 2 ? atoi(argv[2]) : 1;
    static uint8_t garbage[4096];
    h264_stream_info info;
    h264_picture_t pic;
    uint8_t *data;
    long size = 0;
    void *dec;
    int i;

    for (i = 0; i < (int)sizeof(garbage); i++)
        garbage[i] = (uint8_t)(i * 37 + (i / 7) * 91);

    /* --- argument and ordering handling ---------------------------------- */
    dec = h264_dec_open(NULL);
    check(dec != NULL, "open with a NULL parameter block");
    if (dec) {
        check(h264_dec_send_packet(dec, NULL) == H264_DEC_ERROR,
              "send_packet(NULL) is refused");
        check(send_all(dec, NULL, 0, 0) == H264_DEC_ERROR,
              "send_packet with no bytes is refused");
        memset(&pic, 0, sizeof(pic));
        check(h264_dec_recv_frame(dec, &info, &pic) == H264_DEC_DEFAULT,
              "recv_frame on a fresh decoder yields nothing");
        check(h264_dec_flush(dec, &info, &pic) == H264_DEC_END,
              "flush on a fresh decoder says end");
        check(h264_dec_flush(dec, &info, &pic) == H264_DEC_END,
              "flush twice still says end");
        check(h264_dec_send_packet(NULL, NULL) == H264_DEC_ERROR &&
              h264_dec_recv_frame(NULL, &info, &pic) == H264_DEC_ERROR &&
              h264_dec_flush(NULL, &info, &pic) == H264_DEC_ERROR,
              "the three calls survive a NULL decoder");
        h264_dec_frame_unref(NULL, &pic);
        h264_dec_close(NULL);
        h264_dec_close(dec);
    }

    /* invalid parameter values are answered at open, not acted on later */
    {
        static const int bad_sizes[] = { -1, 5, 100, 0x7fffffff };
        h264_param_t param;
        int j, ok = 1;

        for (j = 0; j < (int)(sizeof(bad_sizes) / sizeof(bad_sizes[0])); j++) {
            memset(&param, 0, sizeof(param));
            param.nal_length_size = bad_sizes[j];
            dec = h264_dec_open(&param);
            if (dec) {
                ok = 0;
                h264_dec_close(dec);
            }
        }
        check(ok, "invalid nal_length_size is refused at open");

        memset(&param, 0, sizeof(param));
        param.nal_length_size = 3;
        dec = h264_dec_open(&param);
        check(dec != NULL, "3-byte length prefixes are accepted");
        h264_dec_close(dec);
    }

    data = read_file(path, &size);
    if (!data) {
        printf("api_check: cannot read %s\n", path);
        return 1;
    }

    /* a flush with no output slot must report that it produced no picture,
     * not consume one and lose the caller's reference to it */
    {
        int *au = NULL;
        int n_au = find_access_units(data, (int)size, &au);

        if (n_au > 0) {
            int end = n_au > 1 ? au[1] : (int)size;

            dec = h264_dec_open(NULL);
            check(dec != NULL, "open for the empty-flush test");
            send_all(dec, data + au[0], end - au[0], 0);
            check(h264_dec_flush(dec, &info, NULL) != H264_DEC_GOT_FRAME,
                  "flush without an output slot consumes nothing");
            memset(&pic, 0, sizeof(pic));
            check(h264_dec_flush(dec, &info, &pic) == H264_DEC_GOT_FRAME,
                  "the queued picture survives a NULL flush");
            h264_dec_frame_unref(dec, &pic);
            h264_dec_close(dec);
        }
        free(au);
    }

    /* --- garbage in, documented answer out -------------------------------- */
    {
        h264_param_t param;

        memset(&param, 0, sizeof(param));
        param.threads = threads;
        dec = h264_dec_open(&param);
    }
    check(dec != NULL, "open with %d slots", threads);
    if (!dec) {
        free(data);
        return 1;
    }
    check(send_all(dec, garbage, (int)sizeof(garbage), 1) >= 0,
          "random bytes as one packet are handled, not fatal");
    memset(&pic, 0, sizeof(pic));
    check(h264_dec_recv_frame(dec, &info, &pic) == H264_DEC_DEFAULT,
          "nothing queued after garbage");
    h264_dec_close(dec);

    /* Annex-B bytes read as length-prefixed NALs are nonsense: some will look
     * like a parameter set and be rejected, which is an answer rather than a
     * crash */
    {
        h264_param_t param;
        int ret;
        void *avcc;

        memset(&param, 0, sizeof(param));
        param.threads = 1;
        param.nal_length_size = 4;
        avcc = h264_dec_open(&param);
        ret = send_all(avcc, data, (int)size, 0);
        check(avcc != NULL && (ret >= 0 || ret == H264_DEC_ERROR),
              "Annex-B bytes under a 4-byte prefix: answered");
        memset(&pic, 0, sizeof(pic));
        (void)h264_dec_recv_frame(avcc, &info, &pic);
        h264_dec_frame_unref(avcc, &pic);
        h264_dec_close(avcc);
    }

    /* --- holding pictures must not change the answer ---------------------- */
    {
        int f0 = 0, f4 = 0, f8 = 0;
        uint32_t h0 = 0, h4 = 0, h8 = 0;

        memset(&pic, 0, sizeof(pic));
        dec = h264_dec_open(NULL);
        run_mode(dec, data, size, 0, &f0, &h0);
        h264_dec_close(dec);

        dec = h264_dec_open(NULL);
        run_mode(dec, data, size, 4, &f4, &h4);
        h264_dec_close(dec);

        dec = h264_dec_open(NULL);
        run_mode(dec, data, size, 8, &f8, &h8);
        h264_dec_close(dec);

        check(f0 > 0, "plain path produced frames: %d", f0);
        check(f4 == f0 && f8 == f0, "holding 4 and 8 frames: %d and %d vs %d",
              f4, f8, f0);
        check(h4 == h0 && h8 == h0, "held frames carry the same pixels: %08x/%08x vs %08x",
              h4, h8, h0);
    }

    /* --- take everything and give nothing back ---------------------------- */
    {
        int held_n = 0, dup = 0, a, b;
        int got;
        h264_param_t param;

        memset(&param, 0, sizeof(param));
        param.threads = threads;
        param.output_order = 0;
        dec = h264_dec_open(&param);
        check(send_all(dec, data, (int)size, 100) >= 0, "the whole file in one packet");
        while (held_n < HOLD_MAX) {
            memset(&pic, 0, sizeof(pic));
            if (h264_dec_recv_frame(dec, &info, &pic) != H264_DEC_GOT_FRAME)
                break;
            if (!pic.planes[0] || pic.lines[0] <= 0) {
                check(0, "picture %d came back without a plane", held_n);
                break;
            }
            held[held_n++] = pic;
        }
        check(held_n > 0, "pictures produced while none returned: %d", held_n);
        /* the decoder must never hand out a buffer the caller still holds */
        for (a = 0; a < held_n; a++)
            for (b = a + 1; b < held_n; b++)
                if (held[a].planes[0] == held[b].planes[0])
                    dup++;
        check(dup == 0, "held buffers all distinct: %d collisions", dup);

        for (i = 0; i < held_n; i++)
            h264_dec_frame_unref(dec, &held[i]);
        got = 0;
        while (got < 64) {
            memset(&pic, 0, sizeof(pic));
            if (h264_dec_recv_frame(dec, &info, &pic) != H264_DEC_GOT_FRAME)
                break;
            h264_dec_frame_unref(dec, &pic);
            got++;
        }
        while (h264_dec_flush(dec, &info, &pic) == H264_DEC_GOT_FRAME)
            h264_dec_frame_unref(dec, &pic);
        check(h264_dec_flush(dec, &info, &pic) == H264_DEC_END,
              "queue drains after the buffers came back (%d more)", got);

        memset(&pic, 0, sizeof(pic));
        h264_dec_frame_unref(dec, &pic);
        check(1, "unref of a picture never received is a no-op");
        h264_dec_close(dec);
    }

    free(data);
    printf("api_check: %d failure(s)\n", fails);
    return fails != 0;
}
