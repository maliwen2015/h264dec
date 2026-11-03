#!/bin/sh
# Stability, judged against the reference decoder rather than in the abstract.
#
#   1 determinism - the same bytes out of any thread count and any repeat. This is
#                   the property frame threading can break, and both decoders are
#                   asked the same question, because a threaded H.264 decoder is
#                   supposed to be scheduling-invariant.
#   2 damaged input - the fuzz corpus through both decoders. A signal, a hang or a
#                   decoder that dies is the failure; reporting an error is not.
#                   Also how many pictures each side still produces.
#   3 resources   - peak resident set and user CPU per picture for the two, same
#                   machine. Needs /usr/bin/time -v, so it reports itself skipped
#                   on Windows.
#
#   BIN= THREADS="2 6 12" REPS=2 TIMEO=90 SEEDS="1 2 3" LONG_MD5=1 RSS=0
#   FF=ffmpeg CLIPS="long_720p long_paff_720p long_1080p"
set -u
ROOT=$(cd "$(dirname "$0")/.." && pwd)
DATA=${DATA:-"$ROOT/../testdata"}
BENCH="$DATA/bench"
BIN=${BIN:-}
if [ -z "$BIN" ]; then
    BIN="$ROOT/h264dec_example"
    [ -x "$BIN" ] || BIN="$ROOT/h264dec_example.exe"
fi
FF=${FF:-ffmpeg}
THREADS=${THREADS:-"2 6 12"}
REPS=${REPS:-2}
TIMEO=${TIMEO:-90}
SEEDS=${SEEDS:-"1 2 3"}
CLIPS=${CLIPS:-"long_720p long_paff_720p long_1080p"}
WORK=${WORK:-/tmp/h264fuzz}
D=${WORK}_stab
LONG_MD5=${LONG_MD5:-0}
RSS=${RSS:-1}
FRAMES_SUBSET=${FRAMES_SUBSET:-24}

[ -x "$BIN" ] || { echo "build $BIN first"; exit 1; }
command -v "$FF" >/dev/null 2>&1 || { echo "$FF is not on PATH"; exit 1; }

md5of() {
    if command -v md5sum >/dev/null 2>&1; then md5sum "$1" | cut -c1-12
    else md5 -q "$1" 2>/dev/null | cut -c1-12; fi
}

bad=0
echo "== 1. determinism: one output for every thread count and every repeat =="
rm -rf "$D"; mkdir -p "$D"
for f in "$DATA"/*.h264; do
    name=$(basename "$f" .h264)
    base=""
    for r in $(seq "$REPS"); do
        timeout "$TIMEO" "$BIN" "$f" "$D/o.yuv" 1 >"$D/o.log" 2>&1 || { echo "  $name: run failed"; bad=$((bad+1)); }
        m=$(md5of "$D/o.yuv")
        [ -z "$base" ] && base=$m
        if [ "$m" != "$base" ]; then
            echo "  $name: repeat $r differs ($m vs $base)"; bad=$((bad+1))
        fi
    done
    for t in $THREADS; do
        timeout "$TIMEO" "$BIN" "$f" "$D/o.yuv" "$t" >"$D/o.log" 2>&1 || { echo "  $name: $t-slot run failed"; bad=$((bad+1)); }
        m=$(md5of "$D/o.yuv")
        if [ "$m" != "$base" ]; then
            echo "  $name: threads=$t differs ($m vs $base)"; bad=$((bad+1))
        fi
    done
done
echo "  ours: corpus of $(ls "$DATA"/*.h264 | wc -l | tr -d ' ') streams x ($REPS repeats + $(echo $THREADS | wc -w) thread counts), mismatches=$bad"

fbad=0
for c in $CLIPS; do
    s="$BENCH/$c.h264"
    [ -f "$s" ] || continue
    ffbase=""
    for t in 1 $THREADS; do
        timeout $((TIMEO * 4)) "$FF" -hide_banner -loglevel error -threads "$t" -i "$s" \
            -f md5 "$D/$c.$t.md5" >/dev/null 2>&1
        m=$(sed -n 's/.*MD5=\([0-9a-f]*\).*/\1/p' "$D/$c.$t.md5" 2>/dev/null | cut -c1-12)
        [ -z "$ffbase" ] && ffbase=$m
        if [ "$m" != "$ffbase" ] || [ -z "$m" ]; then
            echo "  $c: ffmpeg threads=$t md5=$m vs $ffbase"; fbad=$((fbad+1))
        fi
    done
    echo "  $c: ffmpeg threads 1 and $THREADS md5 = $ffbase (mismatches=$fbad)"
done

if [ "$LONG_MD5" = 1 ]; then
    for c in $CLIPS; do
        s="$BENCH/$c.h264"
        [ -f "$s" ] || continue
        "$BIN" "$s" "$D/$c.1.yuv" 1 >"$D/o.log" 2>&1
        for t in $THREADS; do
            "$BIN" "$s" "$D/$c.$t.yuv" "$t" >>"$D/o.log" 2>&1
            cmp -s "$D/$c.1.yuv" "$D/$c.$t.yuv" || { echo "  $c: ours $t slots differs from 1 slot"; bad=$((bad+1)); }
            rm -f "$D/$c.$t.yuv"
        done
        rm -f "$D/$c.1.yuv"
        echo "  $c: ours byte-compared against its own one-slot output at $THREADS"
    done
fi

echo "== 2. damaged input: both decoders over the fuzz corpus =="
SEEDS="$SEEDS" TIMEOUT="$TIMEO" sh "$ROOT/tests/fuzz.sh" >/tmp/stab_ours.log 2>&1 || true
tail -2 /tmp/stab_ours.log
# A crash is a signal (or the Windows access-violiation exit codes). A decoder
# that refuses to open a damaged file is not crashing - FFmpeg exits 183 on the
# two variants ours still decodes, which is a resilience difference, not a
# stability one, and the harness has to tell them apart.
is_signal() {
    case "$1" in
        134|139|133|136|3221225477|3221225620|3221225595|3221226505) return 0 ;;
        *) return 1 ;;
    esac
}
n=0; fhang=0; fsig=0; fref=0
for v in "$WORK"/*.h264; do
    [ -f "$v" ] || continue
    n=$((n + 1))
    timeout "$TIMEO" "$FF" -hide_banner -loglevel error -i "$v" -f null - >/tmp/stab_ff.log 2>&1
    code=$?
    if [ "$code" -eq 124 ]; then
        echo "  HANG    ffmpeg $(basename "$v")"; fhang=$((fhang+1))
    elif is_signal "$code"; then
        echo "  SIGNAL  ffmpeg $(basename "$v") exit=$code"; fsig=$((fsig+1))
    elif [ "$code" -ne 0 ]; then
        fref=$((fref+1))
    fi
done
echo "----"
echo "  ffmpeg: variants=$n hang=$fhang signal=$fsig refused_or_error=$fref"

# how much each side still decodes out of the same damaged files, and whether the
# two agree on that number
i=0; oframes=0; fframes=0; agree=0; ours_only=0
for v in "$WORK"/*.h264; do
    [ -f "$v" ] || continue
    i=$((i + 1))
    [ "$i" -gt "$FRAMES_SUBSET" ] && break
    o=$(timeout "$TIMEO" "$BIN" "$v" /dev/null 1 2>&1 | sed -n 's/^frames=\([0-9]*\).*/\1/p')
    f=$(timeout "$TIMEO" "$FF" -hide_banner -loglevel error -stats -i "$v" -f null - 2>&1 |
            tr '\r' '\n' | sed -n 's/.*frame=\s*\([0-9]*\).*/\1/p' | tail -1)
    o=${o:-0}; f=${f:-0}
    oframes=$((oframes + o)); fframes=$((fframes + f))
    [ "$o" = "$f" ] && agree=$((agree + 1))
    [ "$o" -gt 0 ] && [ "$f" -eq 0 ] && ours_only=$((ours_only + 1))
done
echo "  over $i damaged variants: ours=$oframes pictures, ffmpeg=$fframes, "
echo "  same count on $agree of $i, ours decoded where ffmpeg refused on $ours_only"

echo "== 3. resources: peak RSS and user CPU per picture =="
if [ "$RSS" = 1 ] && command -v /usr/bin/time >/dev/null 2>&1; then
    for c in $CLIPS; do
        s="$BENCH/$c.h264"
        [ -f "$s" ] || continue
        /usr/bin/time -v "$BIN" "$s" - 1 >/tmp/stab_rt.log 2>/tmp/stab_ours_rt.log
        o=$(grep -m1 "Maximum resident set size" /tmp/stab_ours_rt.log | awk '{print $NF}')
        ou=$(grep -m1 "^User time" /tmp/stab_ours_rt.log | awk '{print $NF}')
        /usr/bin/time -v "$FF" -hide_banner -loglevel error -threads 1 -i "$s" -f null - \
            >/dev/null 2>/tmp/stab_ff_rt.log
        f=$(grep -m1 "Maximum resident set size" /tmp/stab_ff_rt.log | awk '{print $NF}')
        fu=$(grep -m1 "^User time" /tmp/stab_ff_rt.log | awk '{print $NF}')
        fr=$(sed -n 's/^frames=\([0-9]*\).*/\1/p' /tmp/stab_rt.log | head -1)
        echo "  $c: ours rss=${o}KB user=${ou}s | ffmpeg rss=${f}KB user=${fu}s | frames=$fr"
    done
else
    echo "  skipped (needs /usr/bin/time -v; RSS=0 to silence)"
fi

echo "----"
echo "stability: ours_mismatches=$bad ffmpeg_mismatches=$((fhang + fsig + fbad))"
[ "$bad" -eq 0 ] && [ "$fhang" -eq 0 ] && [ "$fsig" -eq 0 ]
