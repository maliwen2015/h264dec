#!/bin/sh
# Decode every stream in testdata with the reference decoder (ffmpeg) and with
# h264dec_example, then compare the YUV output byte by byte.
set -u

ROOT=$(cd "$(dirname "$0")/.." && pwd)      # h264dec/
DATA=${DATA:-"$ROOT/../testdata"}
# BIN can be pointed at another build's binary, as selfcheck.sh and fuzz.sh can -
# which is how the whole-program binary (make lto) is gated against the reference
BIN=${BIN:-}
if [ -z "$BIN" ]; then
    BIN="$ROOT/h264dec_example"
    [ -x "$BIN" ] || BIN="$ROOT/h264dec_example.exe"
fi
THREADS=${THREADS:-1}
TMP=${TMP:-/tmp/h264cmp}
# a reference tool that wedges must not hang the suite: ffprobe has been seen
# to block indefinitely while its decode children were still running
RTIMEOUT=${RTIMEOUT:-180}
mkdir -p "$TMP"

[ -x "$BIN" ] || { echo "build $BIN first"; exit 1; }
[ -d "$DATA" ] || { echo "no testdata in $DATA"; exit 1; }

pass=0; fail=0; skip=0
echo "threads=$THREADS"
for f in "$DATA"/*.h264; do
    name=$(basename "$f" .h264)
    if ! timeout "$RTIMEOUT" ffprobe -v error -select_streams v:0 -show_entries stream=codec_name \
        -of csv=p=0 "$f" 2>/dev/null | grep -q h264; then
        echo "== $name: SKIP (not decodable by ffmpeg)"
        skip=$((skip + 1))
        continue
    fi
    srcpix=$(ffprobe -v error -select_streams v:0 -show_entries stream=pix_fmt \
             -of csv=p=0 "$f" 2>/dev/null)
    case "$name" in
        *mono*) refpix=gray10le ;;
        *)
            case "$srcpix" in
                yuv444p10le) refpix=yuv444p10le ;;
                yuv422p10le) refpix=yuv422p10le ;;
                *10le)       refpix=yuv420p10le ;;
                yuv444p)     refpix=yuv444p ;;
                yuv422p)     refpix=yuv422p ;;
                *)           refpix=yuv420p ;;
            esac
            ;;
    esac
    timeout "$RTIMEOUT" ffmpeg -v error -y -i "$f" -pix_fmt "$refpix" -f rawvideo "$TMP/$name.ref.yuv" || \
        { echo "== $name: SKIP (reference decode failed)"; skip=$((skip + 1)); continue; }
    # a child that never got to run (spawn pressure with many worker threads)
    # leaves an empty log and a signal exit; retry before failing the stream
    try=0
    while : ; do
        try=$((try + 1))
        "$BIN" "$f" "$TMP/$name.our.yuv" "$THREADS" >"$TMP/$name.log" 2>&1
        code=$?
        { [ "$code" -le 1 ] || [ "$try" -ge 3 ] || [ -s "$TMP/$name.log" ]; } && break
        sleep 1
    done
    [ "$code" -le 1 ] || \
        { echo "== $name: FAIL (decoder returned error, see $TMP/$name.log)"; fail=$((fail + 1)); continue; }

    cmp -s "$TMP/$name.ref.yuv" "$TMP/$name.our.yuv" && {
        echo "== $name: PASS ($(stat -c%s "$TMP/$name.ref.yuv") bytes)"
        pass=$((pass + 1))
    } || {
        bytes=$(cmp -l "$TMP/$name.ref.yuv" "$TMP/$name.our.yuv" 2>/dev/null | wc -l)
        refsz=$(stat -c%s "$TMP/$name.ref.yuv")
        oursz=$(stat -c%s "$TMP/$name.our.yuv")
        # same comparison with in-loop deblocking off on both sides: separates
        # reconstruction errors from deblocking errors
        timeout "$RTIMEOUT" ffmpeg -v error -y -skip_loop_filter all -i "$f" -pix_fmt "$refpix" \
            -f rawvideo "$TMP/$name.nd.ref.yuv" 2>/dev/null && \
        "$BIN" "$f" "$TMP/$name.nd.our.yuv" "$THREADS" 0 1 >/dev/null 2>&1;
        nd=$(cmp -l "$TMP/$name.nd.ref.yuv" "$TMP/$name.nd.our.yuv" 2>/dev/null | wc -l)
        echo "== $name: FAIL diff_bytes=$bytes nodeblock_diff=$nd ref=$refsz our=$oursz"
        fail=$((fail + 1))
    }
done

echo "----"
echo "pass=$pass fail=$fail skip=$skip"
[ "$fail" -eq 0 ]
