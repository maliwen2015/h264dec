/*
 * Re-open soak test.
 *
 * Channel switching in a broadcast decoder means opening, draining and closing
 * it over and over while the process lives on, and a leak in the picture pool,
 * the parameter-set table or the worker pool never shows up in a single decode.
 * Each round decodes the whole file, closes the decoder and reports the working
 * set; the exit status comes from the growth between the second round and the
 * last, so the first round's allocations (which are real but happen once) do
 * not count against it.
 *
 *   build/soak <file.h264> [threads] [rounds] [growth-MB-limit] [disable_lf]
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "h264dec.h"

#ifdef _WIN32
#include <windows.h>
#include <psapi.h>

static double work_set_mb(void)
{
    PROCESS_MEMORY_COUNTERS pmc;
    if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc)))
        return (double)pmc.WorkingSetSize / (1024.0 * 1024.0);
    return 0.0;
}
#else
#include <unistd.h>

static double work_set_mb(void)
{
    /* resident set from /proc, in pages; falls back to 0 where /proc is absent */
    FILE *f = fopen("/proc/self/statm", "r");
    long pages = 0, page = 4096;
    if (f) {
        long total = 0, resident = 0;
        if (fscanf(f, "%ld %ld", &total, &resident) == 2)
            pages = resident;
        fclose(f);
    }
    if (pages == 0)
        return 0.0;
    return (double)pages * (double)page / (1024.0 * 1024.0);
}
#endif

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

static int is_slice_nal(const uint8_t *d, int off)
{
    int type = d[off] & 0x1f;
    return type >= 1 && type <= 5;
}

/* same access-unit rule as example_decoder.c: a slice opens a new access unit
 * when first_mb_in_slice is 0, which is the first bit of its slice header */
static int is_au_first_slice(const uint8_t *d, int off, int end)
{
    int type = d[off] & 0x1f;
    return (type == 1 || type == 5) && off + 1 < end && (d[off + 1] & 0x80);
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
    buf = (uint8_t *)malloc((size_t)size);
    if (!buf || fread(buf, 1, (size_t)size, fp) != (size_t)size) {
        free(buf);
        fclose(fp);
        return NULL;
    }
    fclose(fp);
    *size_out = size;
    return buf;
}

/* one open / decode / drain / close cycle; returns the frame count or -1 */
static int one_round(const uint8_t *data, long size, int threads, int disable_lf,
                     long *frames_out)
{
    h264_param_t param;
    h264_stream_info info;
    h264_picture_t pic;
    h264_packet_t packet;
    void *dec;
    long frames = 0;
    int cap = 16, nb = 0, i;
    int *pos = (int *)malloc((size_t)cap * sizeof(int));
    int *scs = (int *)malloc((size_t)cap * sizeof(int));
    int *starts = (int *)malloc((size_t)(cap + 1) * sizeof(int));

    if (!pos || !scs || !starts) {
        free(pos); free(scs); free(starts);
        return -1;
    }

    for (i = 0; i + 3 <= size; i++) {
        int sc = start_code_len(data, i, (int)size);
        if (!sc || i + sc >= size)
            continue;
        if (nb && pos[nb - 1] == i + sc)
            continue;
        if (nb == cap) {
            int *t;
            cap *= 2;
            t = (int *)realloc(pos, (size_t)cap * sizeof(int));
            if (!t) break;
            pos = t;
            t = (int *)realloc(scs, (size_t)cap * sizeof(int));
            if (!t) break;
            scs = t;
            t = (int *)realloc(starts, (size_t)(cap + 1) * sizeof(int));
            if (!t) break;
            starts = t;
        }
        pos[nb] = i + sc;
        scs[nb] = i;
        nb++;
    }

    memset(&param, 0, sizeof(param));
    param.threads = threads;
    param.output_order = 0;
    param.disable_lf = disable_lf;
    dec = h264_dec_open(&param);
    if (!dec) {
        free(pos); free(scs); free(starts);
        return -1;
    }

    /* feed the NALs one access unit at a time and drain between sends, so the
     * decoder's own queue stays as the working set rather than the input */
    {
        int n = 0, k;
        for (i = 0; i < nb; i++) {
            int end = (i + 1 < nb) ? pos[i + 1] : (int)size;
            int j = i;
            if (!is_au_first_slice(data, pos[i], end))
                continue;
            while (j > 0 && !is_slice_nal(data, pos[j - 1]))
                j--;
            if (n == 0 || starts[n - 1] != scs[j])
                starts[n++] = scs[j];
        }
        for (k = 0; k < n; k++) {
            int start = starts[k];
            int end = (k + 1 < n) ? starts[k + 1] : (int)size;

            memset(&packet, 0, sizeof(packet));
            packet.data = data + start;
            packet.len = end - start;
            if (h264_dec_send_packet(dec, &packet) < 0) {
                fprintf(stderr, "soak: access unit at byte %d rejected\n", start);
                h264_dec_close(dec);
                free(pos); free(scs); free(starts);
                return -1;
            }
            while (h264_dec_recv_frame(dec, &info, &pic) == H264_DEC_GOT_FRAME) {
                frames++;
                h264_dec_frame_unref(dec, &pic);
            }
        }
    }

    while (h264_dec_flush(dec, &info, &pic) == H264_DEC_GOT_FRAME) {
        frames++;
        h264_dec_frame_unref(dec, &pic);
    }

    h264_dec_close(dec);
    free(pos); free(scs); free(starts);
    *frames_out = frames;
    return 0;
}

int main(int argc, char **argv)
{
    const char *path = argc > 1 ? argv[1] : NULL;
    const int threads = argc > 2 ? atoi(argv[2]) : 4;
    const int rounds = argc > 3 ? atoi(argv[3]) : 20;
    const double limit = argc > 4 ? atof(argv[4]) : 8.0;
    const int disable_lf = argc > 5 ? atoi(argv[5]) : 0;
    long size = 0, frames = 0, base_frames = 0;
    double base_ws = 0.0, growth = 0.0;
    uint8_t *data;
    int r;

    if (!path) {
        fprintf(stderr,
                "usage: %s <file.h264> [threads] [rounds] [growth-MB] [disable_lf]\n",
                argv[0]);
        return 2;
    }
    data = read_file(path, &size);
    if (!data) {
        fprintf(stderr, "soak: cannot read %s\n", path);
        return 2;
    }

    for (r = 0; r < rounds; r++) {
        if (one_round(data, size, threads, disable_lf, &frames) < 0) {
            fprintf(stderr, "soak: round %d failed\n", r);
            free(data);
            return 1;
        }
        if (r == 1) {
            base_ws = work_set_mb();
            base_frames = frames;
        } else if (r > 1) {
            growth = work_set_mb() - base_ws;
            if (frames != base_frames) {
                printf("soak: frame count moved between rounds (%ld -> %ld)\n",
                       base_frames, frames);
                free(data);
                return 1;
            }
        }
        printf("round %2d  frames=%-7ld working set=%6.1f MB\n", r, frames,
               work_set_mb());
        fflush(stdout);
    }

    printf("soak: %+.2f MB growth between round 2 and round %d (limit %.1f MB)\n",
           growth, rounds - 1, limit);
    free(data);
    return growth > limit ? 1 : 0;
}
