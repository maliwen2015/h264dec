#!/bin/sh
# Compare this build's output against a reference directory of YUV files, one per
# stream: <REF>/<name>.yuv for testdata/<name>.h264.
#
# With REF missing or empty the script fills it from the current build, so the
# second run measures the thing that actually matters for threading and for
# portability - the same sources, another slot count, or another operating
# system, producing the same bytes:
#
#   THREADS=1 sh tests/selfcheck.sh /tmp/ref     # build the reference
#   THREADS=6 sh tests/selfcheck.sh /tmp/ref     # compare against it
#
# On Linux, point REF at the Windows run's output (a mounted temp dir) to check
# the POSIX worker pool against the Windows one.
set -u
ROOT=$(cd "$(dirname "$0")/.." && pwd)
DATA=${DATA:-"$ROOT/../testdata"}
# BIN can be pointed at another build's binary, which is how the same sources on
# two systems, or two builds of one system, are compared
BIN=${BIN:-}
if [ -z "$BIN" ]; then
    BIN="$ROOT/h264dec_example"
    [ -x "$BIN" ] || BIN="$ROOT/h264dec_example.exe"
fi
THREADS=${THREADS:-1}
OUT=${OUT:-/tmp/selfcheck}
REF=${1:-}

[ -x "$BIN" ] || { echo "build $ROOT/h264dec_example[.exe] first"; exit 1; }
[ -n "$REF" ] || { echo "usage: sh tests/selfcheck.sh <ref-dir>"; exit 1; }
[ -d "$DATA" ] || { echo "no testdata in $DATA"; exit 1; }

mkdir -p "$REF" "$OUT"
same=0; made=0; diff=0; miss=0
for f in "$DATA"/*.h264; do
    name=$(basename "$f" .h264)
    "$BIN" "$f" "$OUT/$name.yuv" "$THREADS" >/dev/null 2>&1
    if [ ! -s "$REF/$name.yuv" ]; then
        cp "$OUT/$name.yuv" "$REF/$name.yuv"
        made=$((made + 1))
        continue
    fi
    if cmp -s "$OUT/$name.yuv" "$REF/$name.yuv"; then
        same=$((same + 1))
    else
        bytes=$(cmp -l "$OUT/$name.yuv" "$REF/$name.yuv" 2>/dev/null | wc -l)
        a=$(stat -c%s "$OUT/$name.yuv" 2>/dev/null || echo 0)
        b=$(stat -c%s "$REF/$name.yuv" 2>/dev/null || echo 0)
        echo "== $name: DIFFERS ($bytes bytes, $a vs $b)"
        diff=$((diff + 1))
    fi
    rm -f "$OUT/$name.yuv"
done

echo "threads=$THREADS ref=$REF"
echo "selfcheck identical=$same differing=$diff missing-ref=$miss created=$made"
rmdir "$OUT" 2>/dev/null
[ "$diff" -eq 0 ] && [ "$miss" -eq 0 ] || exit 1
exit 0
