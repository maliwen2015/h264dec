#!/bin/bash
# Generate H.264 test streams with x264 and reference YUV with ffmpeg.
set -e
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT="$ROOT/testdata"
mkdir -p "$OUT"

SRC="testsrc2=size=176x144:rate=25:duration=1.2"
SRC2="testsrc2=size=320x240:rate=25:duration=1.2"

enc() {
    local name="$1" src="$2"; shift 2
    ffmpeg -y -hide_banner -loglevel error -f lavfi -i "$src" -an -c:v libx264 "$@" \
        -pix_fmt yuv420p "$OUT/$name.h264"
    ffmpeg -y -hide_banner -loglevel error -i "$OUT/$name.h264" -f rawvideo \
        -pix_fmt yuv420p "$OUT/$name.yuv"
    printf '%-28s %8s bytes  %s\n' "$name" "$(stat -c%s "$OUT/$name.h264")" \
        "$(ffprobe -v error -select_streams v -show_entries stream=profile,level,width,height -of csv=p=0 "$OUT/$name.h264")"
}

X264_COMMON="keyint=15:min-keyint=1:scenecut=0"

enc baseline_cavlc "$SRC" -profile:v baseline -level 3.0 \
    -x264-params "${X264_COMMON}:bframes=0:ref=1:slices=1:cabac=0"

enc main_cavlc "$SRC" -profile:v main -level 3.0 \
    -x264-params "${X264_COMMON}:bframes=2:ref=3:slices=4:cabac=0:weightp=2"

enc main_cabac "$SRC" -profile:v main -level 3.0 \
    -x264-params "${X264_COMMON}:bframes=3:ref=4:slices=4:cabac=1:weightp=2:weightb=1:b-pyramid=normal"

enc high_8x8 "$SRC2" -profile:v high -level 3.1 \
    -x264-params "${X264_COMMON}:bframes=3:ref=4:slices=4:cabac=1:8x8dct=1:weightp=2:weightb=1:b-pyramid=normal:aq-mode=3"

enc high_cqm "$SRC" -profile:v high -level 3.0 -x264-params "cqm=jvt" \
    -x264-params "${X264_COMMON}:bframes=2:ref=2:slices=2:cabac=1:8x8dct=1"

enc high_odd_crop "testsrc2=size=318x174:rate=25:duration=1.0" -profile:v high -level 3.0 \
    -x264-params "${X264_COMMON}:bframes=2:ref=2:slices=2:cabac=1:8x8dct=1"


enc high_nodeblock "$SRC" -profile:v high -level 3.0 -x264-params "no-deblock=1" \
    -x264-params "${X264_COMMON}:bframes=2:ref=2:slices=1:cabac=1:8x8dct=1"

enc high_constrained "$SRC" -profile:v high -level 3.0 -x264-params "constrained-intra=1" \
    -x264-params "${X264_COMMON}:bframes=2:ref=2:slices=2:cabac=1:8x8dct=1"

# Lossless: x264 only accepts qp=0 through the High 4:4:4 Predictive profile,
# which is what sets qpprime_y_zero_transform_bypass_flag.
enc high_lossless "$SRC" -profile:v high444 -preset ultrafast \
    -x264-params "${X264_COMMON}:qp=0:bframes=0:ref=1:slices=1:cabac=1"

enc mbaff "$SRC2" -profile:v high -level 4.0 -flags +ildct+ilme -x264-params "tff=1" \
    -x264-params "${X264_COMMON}:bframes=3:ref=3:slices=4:cabac=1:8x8dct=1:weightp=2:weightb=1:b-pyramid=normal"

enc mbaff_cavlc "$SRC" -profile:v main -level 3.1 -flags +ildct+ilme -x264-params "bff=1" \
    -x264-params "${X264_COMMON}:bframes=2:ref=2:slices=2:cabac=0"

enc open_gop "$SRC" -profile:v high -level 3.0 -x264-params \
    "${X264_COMMON}:bframes=3:ref=3:slices=2:cabac=1:8x8dct=1:open-gop=1:b-pyramid=normal"

enc slice_max_size "$SRC2" -profile:v high -level 3.1 \
    -x264-params "${X264_COMMON}:bframes=2:ref=2:cabac=1:8x8dct=1:slice-max-size=600"

# Field coding needs interlaced content: build a 50p source and weave each pair
# of frames into one 25i frame, so the two fields really differ. This x264
# build uses MBAFF (frame pictures with field macroblock pairs) for those.
ISRC="testsrc2=size=176x144:rate=50:duration=1.2"

encf() {
    local name="$1" src="$2"; shift 2
    ffmpeg -y -hide_banner -loglevel error -f lavfi -i "$src" -pix_fmt yuv420p \
        -f rawvideo "$OUT/$name.50p.yuv"
    perl "$ROOT/h264dec/tests/weave_fields.pl" "$OUT/$name.50p.yuv" \
        "$OUT/$name.25i.yuv" 176 144
    ffmpeg -y -hide_banner -loglevel error -f rawvideo -pix_fmt yuv420p \
        -s 176x144 -r 25 -i "$OUT/$name.25i.yuv" -an -c:v libx264 "$@" \
        -pix_fmt yuv420p "$OUT/$name.h264"
    ffmpeg -y -hide_banner -loglevel error -i "$OUT/$name.h264" -f rawvideo \
        -pix_fmt yuv420p "$OUT/$name.yuv"
    rm -f "$OUT/$name.50p.yuv" "$OUT/$name.25i.yuv"
    printf '%-28s %8s bytes  %s\n' "$name" "$(stat -c%s "$OUT/$name.h264")" \
        "$(ffprobe -v error -select_streams v -show_entries stream=profile,level,width,height -of csv=p=0 "$OUT/$name.h264")"
}

encf field_cavlc "$ISRC" -profile:v main -level 3.0 -flags +ildct+ilme \
    -x264-params "${X264_COMMON}:bframes=0:ref=1:slices=1:cabac=0"

encf field_cabac "$ISRC" -profile:v main -level 3.0 -flags +ildct+ilme \
    -x264-params "${X264_COMMON}:bframes=3:ref=3:slices=2:cabac=1:weightp=1:b-pyramid=normal"

encf field_mbs "testsrc2=size=176x144:rate=50:duration=1.2,scroll=horizontal=0.04" \
    -profile:v main -level 3.0 -flags +ildct+ilme \
    -x264-params "${X264_COMMON}:bframes=0:ref=1:slices=1:cabac=0"

# Separate field pictures (PAFF). Feeding a 50p source makes every coded
# picture one field; x264 only uses field coding when tff is set with MB
# level adaptive framing disabled, otherwise it stays in MBAFF.
PSRC="testsrc2=size=176x144:rate=50:duration=1.2,scroll=horizontal=0.04"

paff() {
    local name="$1"; shift
    ffmpeg -y -hide_banner -loglevel error -f lavfi -i "$PSRC" -an -c:v libx264 \
        -flags +ildct+ilme "$@" -pix_fmt yuv420p "$OUT/$name.h264"
    ffmpeg -y -hide_banner -loglevel error -i "$OUT/$name.h264" -f rawvideo \
        -pix_fmt yuv420p "$OUT/$name.yuv"
    printf '%-28s %8s bytes  %s\n' "$name" "$(stat -c%s "$OUT/$name.h264")" \
        "$(ffprobe -v error -select_streams v -count_frames -show_entries stream=profile,width,height,nb_read_frames -of csv=p=0 "$OUT/$name.h264")"
}

paff paff_cavlc -profile:v main -level 3.0 \
    -x264-params "${X264_COMMON}:tff=1:mb-aff=0:bframes=0:ref=1:slices=1:cabac=0"

paff paff_cabac -profile:v main -level 3.0 \
    -x264-params "${X264_COMMON}:tff=1:mb-aff=0:bframes=3:ref=3:slices=2:cabac=1:weightp=1:b-pyramid=normal"

paff paff_high -profile:v high -level 3.1 \
    -x264-params "${X264_COMMON}:tff=1:mb-aff=0:bframes=2:ref=4:slices=1:cabac=1:8x8dct=1:weightp=2"

paff paff_odd -profile:v main -level 3.0 \
    -x264-params "${X264_COMMON}:tff=1:mb-aff=0:bframes=0:ref=2:slices=3:cabac=0"

# Field order that only a picture timing SEI can express: the frame is coded
# with MBAFF and the encoder says which field comes first.
enc pic_timing "$SRC" -profile:v main -level 3.0 \
    -x264-params "${X264_COMMON}:tff=1:bframes=0:ref=2:slices=1:cabac=1"

# An HRD in the VUI: its field widths give the picture timing delays their
# meaning, and the rest of the VUI shifts by one bit if low_delay_hrd_flag is
# not read between the HRD parameters and pic_struct_present_flag.
enc hrd_cbr "$SRC" -profile:v high -level 3.1 -b:v 384k \
    -x264-params "${X264_COMMON}:nal-hrd=cbr:vbv-maxrate=384:vbv-bufsize=192:bframes=3:ref=3:slices=2:cabac=1:8x8dct=1"

echo "done: $OUT"
