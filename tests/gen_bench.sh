#!/bin/bash
# Larger streams for performance measurement (not part of the bit-exact suite).
set -e
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
# kept out of testdata/*.h264 so the bit-exact sweep is not slowed down
OUT="$ROOT/testdata/bench"
mkdir -p "$OUT"
X264_COMMON="keyint=60:min-keyint=1:scene_cut=0:scenecut=0"

bench() {
    local name="$1" src="$2"; shift 2
    ffmpeg -y -hide_banner -loglevel error -f lavfi -i "$src" -an -c:v libx264 "$@" \
        -pix_fmt yuv420p "$OUT/$name.h264"
    ffmpeg -y -hide_banner -loglevel error -i "$OUT/$name.h264" -f rawvideo \
        -pix_fmt yuv420p "$OUT/$name.yuv"
    printf '%-28s %10s bytes  %s\n' "$name" "$(stat -c%s "$OUT/$name.h264")" \
        "$(ffprobe -v error -select_streams v -count_frames -show_entries stream=profile,width,height,nb_read_frames -of csv=p=0 "$OUT/$name.h264")"
}

SRC720="testsrc2=size=1280x720:rate=25:duration=12"
SRC1080="testsrc2=size=1920x1080:rate=25:duration=8"
# build-to-build A/B needs windows long enough to see a few percent: a 12s clip
# decodes in about a second here and the desktop swings that much by itself
SRC720_LONG="testsrc2=size=1280x720:rate=25:duration=60"
SRC1080_LONG="testsrc2=size=1920x1080:rate=25:duration=40"

bench bench_720p_high "$SRC720" -profile:v high -level 4.1 -preset medium -crf 27 \
    -x264-params "${X264_COMMON}:bframes=3:ref=4:cabac=1:8x8dct=1:weightp=2:b-pyramid=normal"

bench bench_720p_main "$SRC720" -profile:v main -level 4.1 -preset medium -crf 27 \
    -x264-params "${X264_COMMON}:bframes=3:ref=4:cabac=1"

bench bench_1080p_high "$SRC1080" -profile:v high -level 4.1 -preset fast -crf 27 \
    -x264-params "${X264_COMMON}:bframes=3:ref=4:cabac=1:8x8dct=1:weightp=2:b-pyramid=normal"

# Interlaced broadcast content at a size worth timing. The rate is what decides
# the shape: with +ildct+ilme and tff=1, a 50 fields/s source gives x264 one
# field per input frame and it codes separate field pictures (PAFF, 1000 coded
# pictures for 20 s), while a 25 fps source gives it frame pairs and it codes
# MBAFF (500 coded pictures of the same content). The small bit-exact versions
# are in tests/gen_streams.sh. Note that this x264 build rejects "mb-aff=0", and
# that in a real PAFF stream the field pictures do not start at the beginning:
# of long_paff_720p's 1000 pictures 415 are fields and 585 are frames.
bench long_paff_720p "testsrc2=size=1280x720:rate=50:duration=20" \
    -profile:v high -level 4.1 -flags +ildct+ilme -preset medium -crf 27 \
    -x264-params "${X264_COMMON}:tff=1:bframes=2:ref=4:cabac=1:8x8dct=1:weightp=2"

bench long_mbff_720p "testsrc2=size=1280x720:rate=25:duration=20" \
    -profile:v high -level 4.1 -flags +ildct+ilme -preset medium -crf 27 \
    -x264-params "${X264_COMMON}:tff=1:bframes=2:ref=4:cabac=1:8x8dct=1:weightp=2"

# Three shapes at 1080p, for where the per-macroblock cost sits rather than what
# the whole picture costs: residuals with motion, no motion compensation at all,
# and almost nothing but P-skip. The loop-filter QP skip is measured on these, and
# the flat one is what makes it visible - on inter content with residuals it is
# nearly invisible because the macroblocks are above the threshold. No reference
# YUV is written for these; they exist to be timed.
SRC1080_YUV="$OUT/bench_1080p_high.yuv"

shape() {
    local name="$1"; shift
    ffmpeg -y -hide_banner -loglevel error "$@" -pix_fmt yuv420p "$OUT/$name.h264"
    printf '%-28s %10s bytes  %s\n' "$name" "$(stat -c%s "$OUT/$name.h264")" \
        "$(ffprobe -v error -select_streams v -count_frames \
            -show_entries stream=profile,width,height,nb_read_frames -of csv=p=0 \
            "$OUT/$name.h264")"
}

shape prof_inter_1080p -f rawvideo -pix_fmt yuv420p -s 1920x1080 -r 25 -i "$SRC1080_YUV" \
    -c:v libx264 -preset medium -crf 20 -g 250 -x264-params keyint=250:min-keyint=250
shape prof_intra_1080p -f rawvideo -pix_fmt yuv420p -s 1920x1080 -r 25 -i "$SRC1080_YUV" \
    -c:v libx264 -preset medium -crf 20 -g 1 -x264-params keyint=1:scenecut=0
shape prof_static_1080p -f lavfi -i "color=black:s=1920x1080:r=25:d=8" \
    -c:v libx264 -preset medium -crf 20 -g 250 -x264-params keyint=250:min-keyint=250
