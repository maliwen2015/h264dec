#include "h264dec.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef _WIN32
#include <windows.h>
static double wall_time_ms(void)
{
    static LARGE_INTEGER freq;
    static int init = 0;
    if (!init) { QueryPerformanceFrequency(&freq); init = 1; }
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    return (double)now.QuadPart * 1000.0 / freq.QuadPart;
}
#else
#include <sys/time.h>
static double wall_time_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000.0 + tv.tv_usec / 1000.0;
}
#endif

typedef struct demo_context {
    void* decoder;
    FILE* yuv_fp;
    int frame_count;
    int au_errors;
    int error_frames;
    double start_time;
} demo_context_t;

static int save_yuv_frame(FILE* fp, const h264_picture_t* picture)
{
    int i;
    if (!fp)
        return 0;
    for (i = 0; i < picture->lines[0]; i++)
        fwrite(picture->planes[0] + i * picture->strides[0], picture->bytes_per_sample,
               picture->widths[0], fp);
    if (picture->num_planes < 3)
        return 0;
    for (i = 0; i < picture->lines[1]; i++)
        fwrite(picture->planes[1] + i * picture->strides[1], picture->bytes_per_sample,
               picture->widths[1], fp);
    for (i = 0; i < picture->lines[2]; i++)
        fwrite(picture->planes[2] + i * picture->strides[2], picture->bytes_per_sample,
               picture->widths[2], fp);
    return 0;
}

static void handle_frame(demo_context_t* demo, h264_picture_t* pic)
{
    demo->frame_count++;
    if (pic->b_decode_error)
        demo->error_frames++;
    save_yuv_frame(demo->yuv_fp, pic);
    h264_dec_frame_unref(demo->decoder, pic);
}

static void drain(demo_context_t* demo)
{
    h264_stream_info info;
    h264_picture_t pic;
    while (h264_dec_recv_frame(demo->decoder, &info, &pic) == H264_DEC_GOT_FRAME)
        handle_frame(demo, &pic);
}

/* Length of the annex-B start code at d[i], or 0 when there is none. */
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

/* A slice starts an access unit when first_mb_in_slice is 0, the first ue(v)
 * of the slice header. An exp-Golomb code word is a run of zero bits, then a
 * one, then that many information bits, so the value is 0 exactly when the
 * very first bit is one. Nothing has to be unescaped to see that bit: an
 * emulation prevention byte only follows two zero bytes. */
static int is_au_first_slice(const uint8_t *d, int off, int end)
{
    int type = d[off] & 0x1f;
    return (type == 1 || type == 5) && off + 1 < end && (d[off + 1] & 0x80);
}

/* Offsets at which the access units of the stream begin. */
static int find_access_units(const uint8_t *data, int size, int **out)
{
    int cap = 16, nb = 0, n = 0, i;
    int *pos = (int *)malloc(cap * sizeof(int));   /* NAL header byte */
    int *scs = (int *)malloc(cap * sizeof(int));   /* start code offset */
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
            continue;                       /* 4-byte code seen again as 3-byte */
        if (nb == cap) {
            int *t;
            cap *= 2;
            t = (int *)realloc(pos, cap * sizeof(int));
            if (!t)
                break;
            pos = t;
            t = (int *)realloc(scs, cap * sizeof(int));
            if (!t)
                break;
            scs = t;
        }
        pos[nb] = i + sc;
        scs[nb] = i;
        nb++;
    }
    starts = (int *)malloc((nb + 1) * sizeof(int));
    if (!starts) {
        free(pos);
        free(scs);
        *out = NULL;
        return 0;
    }
    for (i = 0; i < nb; i++) {
        int end = (i + 1 < nb) ? pos[i + 1] : size;
        int j = i;
        if (!is_au_first_slice(data, pos[i], end))
            continue;
        /* pull in the parameter sets and SEI that precede the slice */
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

int main(int argc, char* argv[])
{
    const char* input_file;
    const char* output_file;
    int threads = 1, output_order = 0, disable_lf = 0, skip_non_ref = 0;
    FILE *fp;
    long file_size;
    uint8_t *buffer;
    h264_param_t param;
    h264_packet_t packet;
    h264_stream_info info;
    h264_picture_t pic;
    demo_context_t demo;
    int ret = 0;
    int got_header = 0;

    if (argc < 3) {
        printf("Usage: %s <input.h264> <output.yuv|NUL|-> [threads] [output_order] [disable_lf] [skip_non_ref]\n",
               argv[0]);
        printf("  '-' decodes without writing the pictures, for throughput runs\n");
        return 1;
    }
    input_file = argv[1];
    output_file = argv[2];
    if (argc >= 4) threads = atoi(argv[3]);
    if (argc >= 5) output_order = atoi(argv[4]);
    if (argc >= 6) disable_lf = atoi(argv[5]);
    if (argc >= 7) skip_non_ref = atoi(argv[6]);

    fp = fopen(input_file, "rb");
    if (!fp) {
        printf("Failed to open input: %s\n", input_file);
        return 1;
    }
    fseek(fp, 0, SEEK_END);
    file_size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    buffer = (uint8_t *)malloc(file_size);
    if (!buffer || fread(buffer, 1, file_size, fp) != (size_t)file_size) {
        printf("Failed to read input\n");
        fclose(fp);
        free(buffer);
        return 1;
    }
    fclose(fp);

    memset(&param, 0, sizeof(param));
    param.threads = threads;
    param.output_order = output_order;
    param.disable_lf = disable_lf;
    param.skip_non_ref = skip_non_ref;
    memset(&demo, 0, sizeof(demo));
    demo.decoder = h264_dec_open(&param);
    if (!demo.decoder) {
        printf("Failed to create decoder\n");
        free(buffer);
        return 1;
    }
    /* "-" asks for no sink at all: the pictures are still delivered and
     * released, only the copy out of them is skipped. A throughput comparison
     * needs that, because a 720p clip is 2 GB of YUV and the copy is serial on
     * the calling thread - 8% of the wall at ten slots. */
    demo.yuv_fp = strcmp(output_file, "-") ? fopen(output_file, "wb") : NULL;
    /* A 720p frame is 1.4 MB written as 1440 fwrite() calls; the default stdio
     * buffer is far smaller, so every few rows turn into a write syscall. A
     * one-megabyte buffer was worth 2% at one slot and 4-5% at eight and
     * twelve in the file-to-YUV benchmark, where the output path is the
     * difference between this decoder and FFmpeg. */
    if (demo.yuv_fp)
        setvbuf(demo.yuv_fp, NULL, _IOFBF, 1 << 20);
    if (!demo.yuv_fp && strcmp(output_file, "-")) {
        printf("Failed to open output: %s\n", output_file);
        h264_dec_close(demo.decoder);
        free(buffer);
        return 1;
    }

    demo.start_time = wall_time_ms();

    /* feed one access unit per call and drain the output in between, so that
     * the decoder never has to hold more than a couple of pictures */
    {
        int *au, nb_au, k;

        nb_au = find_access_units(buffer, (int)file_size, &au);
        for (k = 0; k < nb_au; k++) {
            int start = au[k];
            int end = (k + 1 < nb_au) ? au[k + 1] : (int)file_size;

            memset(&packet, 0, sizeof(packet));
            packet.data = buffer + start;
            packet.len = end - start;
            if (h264_dec_send_packet(demo.decoder, &packet) < 0) {
                /* a broken access unit must not stop the stream: keep feeding
                 * the rest and let b_decode_error mark what was damaged */
                printf("access unit at byte %d failed to decode\n", start);
                demo.au_errors++;
            }
            drain(&demo);
        }
        free(au);
        if (!nb_au)
            printf("No access unit found\n");
    }

    while (1) {
        ret = h264_dec_flush(demo.decoder, &info, &pic);
        if (ret == H264_DEC_GOT_FRAME) {
            handle_frame(&demo, &pic);
        } else {
            break;
        }
    }

    if (!got_header && info.width) {
        printf("Video: %dx%d profile=%d level=%d chroma=%d depth=%d\n",
               info.width, info.height, info.profile, info.level,
               info.chroma_format, info.bit_depth);
    }

    {
        double total_ms = wall_time_ms() - demo.start_time;
        double fps = total_ms > 0 ? demo.frame_count * 1000.0 / total_ms : 0;
        printf("frames=%d elapsed=%.1f ms fps=%.2f threads=%d "
               "au_errors=%d corrupt_frames=%d\n",
               demo.frame_count, total_ms, fps, threads,
               demo.au_errors, demo.error_frames);
    }

    free(buffer);
    if (demo.yuv_fp)
        fclose(demo.yuv_fp);
    h264_dec_close(demo.decoder);
    return demo.au_errors ? 1 : 0;
}
