#!/bin/sh
# Feed every stream as length-prefixed (AVCC) NALs instead of Annex-B start
# codes. The decode must be byte identical to the start-code decode, which
# tests/compare.sh already verifies against ffmpeg.
set -u

ROOT=$(cd "$(dirname "$0")/.." && pwd)      # h264dec/
DATA=${DATA:-"$ROOT/../testdata"}
BIN="$ROOT/h264dec_example"
AVCC="$ROOT/build/avcc_check"
TMP=${TMP:-/tmp/h264avcc}
mkdir -p "$TMP"

[ -x "$AVCC" ] || { echo "build $AVCC first"; exit 1; }
[ -d "$DATA" ] || { echo "no testdata in $DATA"; exit 1; }

ok=0; bad=0; skip=0
for f in "$DATA"/*.h264; do
    name=$(basename "$f" .h264)
    "$BIN" "$f" "$TMP/$name.b.yuv" 1 >/dev/null 2>&1 || \
        { echo "== $name: SKIP (annex-b decode failed)"; skip=$((skip + 1)); continue; }
    "$AVCC" "$f" "$TMP/$name.avcc.yuv" >/dev/null 2>&1
    code=$?
    if [ "$code" -gt 1 ]; then
        echo "== $name: FAIL (avcc decode returned error)"
        bad=$((bad + 1))
        continue
    fi
    if cmp -s "$TMP/$name.b.yuv" "$TMP/$name.avcc.yuv"; then
        echo "== $name: PASS ($(stat -c%s "$TMP/$name.avcc.yuv") bytes)"
        ok=$((ok + 1))
    else
        d=$(cmp -l "$TMP/$name.b.yuv" "$TMP/$name.avcc.yuv" 2>/dev/null | wc -l)
        echo "== $name: FAIL diff_bytes=$d annexb=$(stat -c%s "$TMP/$name.b.yuv") \
avcc=$(stat -c%s "$TMP/$name.avcc.yuv")"
        bad=$((bad + 1))
    fi
done

echo "----"
echo "avcc ok=$ok bad=$bad skip=$skip"
[ "$bad" -eq 0 ]
