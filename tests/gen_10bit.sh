#!/bin/sh
# gen_10bit.sh - write a 10-bit corpus covering the syntax the 10-bit paths
# have to get right: CABAC and CAVLC, B-frames with weighted prediction,
# MBAFF, PAFF, several slices, transform bypass (lossless), cropping and 720p,
# across 4:0:0, 4:2:0, 4:2:2 and 4:4:4.
#
# The streams land in ../testdata/10bit, which is ignored by git. compare.sh
# detects the pixel format from ffprobe (and the "mono" name for 4:0:0
# streams, whose decoder default pix_fmt hides the chroma format), so the same
# script that gates the 8-bit corpus gates this one:
#
#   sh tests/gen_10bit.sh
#   DATA=../testdata/10bit THREADS=8 sh tests/compare.sh
#
# FORCE=1 re-encodes every stream; otherwise an existing file is reused.
set -u
ROOT=$(cd "$(dirname "$0")/.." && pwd)
OUT=${OUT:-"$ROOT/../testdata/10bit"}
FORCE=${FORCE:-0}
FRAMES=${FRAMES:-30}

command -v ffmpeg >/dev/null || { echo "ffmpeg is not on PATH"; exit 1; }
mkdir -p "$OUT"

# name | source | x264 arguments
gen() {
    name=$1; src=$2; shift 2
    stream="$OUT/$name.h264"
    if [ -s "$stream" ] && [ "$FORCE" != 1 ]; then
        echo "== $name: keep $(basename "$stream")"
        return
    fi
    echo "== $name: encode"
    ffmpeg -y -hide_banner -loglevel error -f lavfi -i "$src" \
        -frames:v "$FRAMES" -c:v libx264 -pix_fmt yuv420p10le \
        -profile:v high10 -preset medium "$@" "$stream" || exit 1
}

gen cabac_main10  "testsrc2=size=128x96:rate=25"  -crf 23
gen cavlc_main10  "testsrc2=size=320x240:rate=25" -crf 23 -coder 0
gen bframes10     "testsrc2=size=176x144:rate=25" -crf 23 -bf 4 -refs 5 -weightp 2 -weightb 1
gen mbaff10       "testsrc2=size=176x144:rate=25" -crf 23 -x264-params "interlaced=1:mbaff=1"
gen paff10        "testsrc2=size=176x144:rate=25" -crf 23 -x264-params "interlaced=1:no-mbaff=1"
gen slices10      "testsrc2=size=176x144:rate=25" -crf 23 -x264-params "slices=8"
gen hd720_10      "testsrc2=size=1280x720:rate=25" -crf 23

# genpf is gen with an explicit pixel format and profile
genpf() {
    name=$1; src=$2; pixfmt=$3; profile=$4; shift 4
    stream="$OUT/$name.h264"
    if [ -s "$stream" ] && [ "$FORCE" != 1 ]; then
        echo "== $name: keep $(basename "$stream")"
        return
    fi
    echo "== $name: encode"
    ffmpeg -y -hide_banner -loglevel error -f lavfi -i "$src" \
        -frames:v "$FRAMES" -c:v libx264 -pix_fmt "$pixfmt" \
        -profile:v "$profile" -preset medium "$@" "$stream" || exit 1
}

# The remaining 10-bit profiles: 4:4:4, 4:2:2 and monochrome.  The monochrome
# name is what compare.sh keys on when asking FFmpeg for gray10le, because
# ffprobe reports the decoder's default yuv420 output for a gray stream.
genpf 444_cabac10 "testsrc2=size=176x144:rate=25" yuv444p10le high444 -crf 23
genpf 444_cavlc10 "testsrc2=size=176x144:rate=25" yuv444p10le high444 -crf 23 -coder 0
genpf 444_mbaff10 "testsrc2=size=176x144:rate=25" yuv444p10le high444 -crf 23 -bf 2 \
      -x264-params "interlaced=1:mbaff=1"
genpf 444_crop10  "testsrc2=size=318x190:rate=25" yuv444p10le high444 -crf 23 -bf 2
genpf 422_cabac10 "testsrc2=size=176x144:rate=25" yuv422p10le high422 -crf 23
genpf 422_cavlc10 "testsrc2=size=176x144:rate=25" yuv422p10le high422 -crf 23 -coder 0
genpf 422_mbaff10 "testsrc2=size=176x144:rate=25" yuv422p10le high422 -crf 23 -bf 2 \
      -x264-params "interlaced=1:mbaff=1"
genpf 422_crop10  "testsrc2=size=318x190:rate=25" yuv422p10le high422 -crf 23 -bf 2
genpf mono_cavlc10 "testsrc2=size=176x144:rate=25" gray10le high10 -crf 23 -coder 0 \
      -x264-params "output-csp=i400"
genpf mono_bf10   "testsrc2=size=176x144:rate=25" gray10le high10 -crf 23 -bf 2 \
      -x264-params "output-csp=i400"

# transform bypass needs the 4:4:4 Predictive profile; chroma stays 4:2:0.
if [ ! -s "$OUT/lossless10.h264" ] || [ "$FORCE" = 1 ]; then
    echo "== lossless10: encode"
    ffmpeg -y -hide_banner -loglevel error -f lavfi \
        -i "testsrc2=size=176x144:rate=25" -frames:v "$FRAMES" \
        -c:v libx264 -pix_fmt yuv420p10le -profile:v high444 \
        -preset medium -qp 0 -coder 0 "$OUT/lossless10.h264" || exit 1
fi

# 4:4:4 transform bypass, with the chroma planes really present.
if [ ! -s "$OUT/444_lossless10.h264" ] || [ "$FORCE" = 1 ]; then
    echo "== 444_lossless10: encode"
    ffmpeg -y -hide_banner -loglevel error -f lavfi \
        -i "testsrc2=size=176x144:rate=25" -frames:v "$FRAMES" \
        -c:v libx264 -pix_fmt yuv444p10le -profile:v high444 \
        -preset medium -qp 0 -coder 0 "$OUT/444_lossless10.h264" || exit 1
fi

echo "10-bit corpus in $OUT"
