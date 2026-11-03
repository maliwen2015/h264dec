#!/bin/sh
# Training runs for the profile-guided build: everything the bit-exact corpus
# has, at one and several slots, plus the long streams so the hot paths are
# weighted the way a real decode session weights them. Run from the project
# root by `make pgo`; the .gcda files land next to the objects.
set -u
ROOT=$(cd "$(dirname "$0")/.." && pwd)
DATA=${DATA:-"$ROOT/../testdata"}
BIN="$ROOT/h264dec_example"
[ -x "$BIN" ] || BIN="$ROOT/h264dec_example.exe"
OUT=${OUT:-/tmp/h264pgo}

[ -x "$BIN" ] || { echo "build $BIN first"; exit 1; }
mkdir -p "$OUT"

n=0
for f in "$DATA"/*.h264; do
    for t in 1 6; do
        "$BIN" "$f" "$OUT/train.yuv" "$t" > /dev/null 2>&1 || \
            { echo "training run failed: $f threads=$t"; exit 1; }
        n=$((n + 1))
    done
done

# the long clips carry the interlaced and MBAFF paths and the mix of slice types
# a short conformance stream does not
for name in long_720p long_1080p long_paff_720p long_mbff_720p; do
    f="$DATA/bench/$name.h264"
    [ -f "$f" ] || continue
    for t in 1 6; do
        "$BIN" "$f" "$OUT/train.yuv" "$t" > /dev/null 2>&1 || \
            { echo "training run failed: $f threads=$t"; exit 1; }
        n=$((n + 1))
    done
done

# the scalar path has its own branch profile, so train it too
H264_DISABLE_SIMD=1 "$BIN" "$DATA/main_cabac.h264" "$OUT/train.yuv" 1 \
    > /dev/null 2>&1
n=$((n + 1))

rm -rf "$OUT"
echo "pgo training: $n runs"
