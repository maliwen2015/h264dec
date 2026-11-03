/* dump NAL types and first_mb_in_slice for an annex-B stream (debug helper) */
#include <stdio.h>
#include <stdlib.h>

static int sc_len(const unsigned char *d, int i, int size)
{
    if (i + 3 > size || d[i] || d[i + 1])
        return 0;
    if (d[i + 2] == 1)
        return 3;
    if (i + 4 <= size && d[i + 2] == 0 && d[i + 3] == 1)
        return 4;
    return 0;
}

int main(int argc, char **argv)
{
    FILE *f;
    long sz;
    unsigned char *b;
    int i, prev = -1;

    if (argc < 2)
        return 1;
    f = fopen(argv[1], "rb");
    fseek(f, 0, SEEK_END);
    sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    b = malloc(sz);
    if (fread(b, 1, sz, f) != (size_t)sz)
        return 1;
    fclose(f);
    for (i = 0; i + 3 <= (int)sz; i++) {
        int sc = sc_len(b, i, (int)sz);
        int type, n = 0, j = i + sc, zeros;
        unsigned char bits[8];
        long first_mb = -1;
        if (!sc || j >= (int)sz)
            continue;
        if (prev == j)
            continue;
        prev = j;
        type = b[j] & 0x1f;
        if (type != 1 && type != 5) {
            printf("%7d type %2d ref %d\n", j, type, (b[j] >> 5) & 3);
            continue;
        }
        j++;
        while (j < (int)sz && n < 8) {
            if (j + 2 < (int)sz && b[j] == 0 && b[j + 1] == 0 && b[j + 2] == 3) {
                bits[n++] = 0;
                bits[n++] = 0;
                j += 3;
            } else {
                bits[n++] = b[j++];
            }
        }
        for (zeros = 0; zeros < n * 8; zeros++)
            if ((bits[zeros >> 3] >> (7 - (zeros & 7))) & 1)
                break;
        if (zeros <= 15 && 2 * zeros < n * 8) {
            unsigned val = 1;
            int k;
            for (k = 0; k < zeros; k++) {
                int p = zeros + 1 + k;
                val = (val << 1) | ((bits[p >> 3] >> (7 - (p & 7))) & 1);
            }
            first_mb = (long)val - 1;
        }
        printf("%7d type %2d ref %d zeros %d first_mb %ld\n",
               i, type, (b[i + sc] >> 5) & 3, zeros, first_mb);
    }
    free(b);
    return 0;
}
