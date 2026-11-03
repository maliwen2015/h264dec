#!/bin/sh
# feature_matrix.sh - which H.264 profiles and chroma formats this decoder
# accepts, with FFmpeg as the reference for the label and the pixels.
#
# Every case is a ten-frame testsrc2 encoded by libx264 at one profile and pixel
# format. The script then decodes it here and with FFmpeg and compares the YUV
# byte for byte when this decoder produced pictures. It is a report, not a gate:
# a format the decoder does not implement is printed as "unsupported" and the
# script still exits 0, so it can be run over a tree where high-bit-depth
# support is the thing being added.
#
#   sh tests/feature_matrix.sh
#   OUT=/path BIN=/path/to/h264dec_example sh tests/feature_matrix.sh
#
# Cases that libx264 refuses to encode (its build may lack a bit depth) are
# SKIP, not PASS, so the number of formats really exercised stays visible.
set -u
ROOT=$(cd "$(dirname "$0")/.." && pwd)
OUT=${OUT:-"$ROOT/../testdata/features"}
BIN=${BIN:-}
if [ -z "$BIN" ]; then
    BIN="$ROOT/h264dec_example"
    [ -x "$BIN" ] || BIN="$ROOT/h264dec_example.exe"
fi
FRAMES=${FRAMES:-10}
TIMEOUT=${TIMEOUT:-60}

command -v ffmpeg  >/dev/null || { echo "ffmpeg is not on PATH"; exit 1; }
command -v ffprobe >/dev/null || { echo "ffprobe is not on PATH"; exit 1; }
[ -x "$BIN" ] || { echo "build $BIN first"; exit 1; }
mkdir -p "$OUT"

pass=0 unsupported=0 partial=0 skip=0

# name | pix_fmt | profile
run_case() {
    name=$1; pix=$2; prof=$3
    stream="$OUT/$name.h264"
    ref="$OUT/$name.ref.yuv"
    got="$OUT/$name.our.yuv"

    if ! ffmpeg -y -hide_banner -loglevel error -f lavfi \
              -i "testsrc2=size=128x96:rate=25" -frames:v "$FRAMES" \
              -c:v libx264 -pix_fmt "$pix" \
              -profile:v "$prof" "$stream" 2>"$OUT/$name.enc.log"; then
        printf '%-22s %-24s %-14s SKIP   (libx264 refused: %s)\n' \
            "$name" "-" "$pix" "$(head -c 80 "$OUT/$name.enc.log" | tr -d '\n')"
        skip=$((skip + 1)); return
    fi

    label=$(ffprobe -v error -select_streams v -show_entries \
            stream=profile,pix_fmt -of csv=p=0 "$stream" 2>/dev/null)
    [ -n "$label" ] || label="?,$pix"

    # the reference decode is the criterion for "this stream is well formed";
    # if FFmpeg cannot decode it there is nothing to compare against
    if ! ffmpeg -v error -y -i "$stream" -pix_fmt "$pix" -f rawvideo "$ref" 2>/dev/null; then
        printf '%-22s %-24s %-14s SKIP   (FFmpeg could not decode it)\n' \
            "$name" "$label" "$pix"
        skip=$((skip + 1)); return
    fi

    : > "$OUT/$name.log"
    timeout "$TIMEOUT" "$BIN" "$stream" "$got" 1 >"$OUT/$name.log" 2>&1
    code=$?
    frames=$(sed -n 's/^frames=\([0-9]*\).*/\1/p' "$OUT/$name.log" | tail -1)
    errs=$(sed -n 's/.*au_errors=\([0-9]*\).*/\1/p' "$OUT/$name.log" | tail -1)
    [ -n "$frames" ] || frames=0
    [ -n "$errs" ] || errs=0

    if [ "$code" -gt 1 ] || [ "$frames" -eq 0 ]; then
        printf '%-22s %-24s %-14s UNSUPPORTED\n' "$name" "$label" "$pix"
        unsupported=$((unsupported + 1))
        return
    fi
    if [ "$frames" -ne "$FRAMES" ] || [ "$errs" -ne 0 ]; then
        printf '%-22s %-24s %-14s PARTIAL  frames=%s/%s au_errors=%s\n' \
            "$name" "$label" "$pix" "$frames" "$FRAMES" "$errs"
        partial=$((partial + 1))
        return
    fi
    if cmp -s "$ref" "$got"; then
        printf '%-22s %-24s %-14s PASS     bit-exact\n' "$name" "$label" "$pix"
        pass=$((pass + 1))
    else
        bytes=$(cmp -l "$ref" "$got" 2>/dev/null | wc -l)
        printf '%-22s %-24s %-14s PASS*    frames=%s, %s bytes differ\n' \
            "$name" "$label" "$pix" "$frames" "$bytes"
        partial=$((partial + 1))
    fi
}

echo "decoder: $BIN"
echo "FFmpeg:  $(ffmpeg -version 2>/dev/null | head -1)"
echo
printf '%-22s %-24s %-14s %s\n' "case" "ffprobe profile,pix_fmt" "pix_fmt" "result"
printf '%-22s %-24s %-14s %s\n' "----" "---------------------" "-------" "------"

run_case baseline_420p8   yuv420p     baseline
run_case main_420p8       yuv420p     main
run_case high_420p8       yuv420p     high
run_case high10_420p10    yuv420p10le high10
run_case high422_422p10   yuv422p10le high422
run_case high444_444p10   yuv444p10le high444
run_case high422_422p8    yuv422p     high422
run_case high444_444p8    yuv444p     high444

echo
printf 'summary: %d bit-exact, %d partial, %d unsupported, %d skipped\n' \
    "$pass" "$partial" "$unsupported" "$skip"
echo "streams and YUVs are in $OUT"
