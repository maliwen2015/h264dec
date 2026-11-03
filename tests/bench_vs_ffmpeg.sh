#!/bin/sh
# Matched-sink throughput against the reference decoder.
#
# Both sides have to discard the pictures. Our example copies every decoded
# frame out through stdio, which for 720p is 2 GB of memcpy on one thread, and
# FFmpeg's rawvideo sink is worse: measured on the same clip in the same window
# it goes 4043 -> 2294 frames per second between -threads 8 with -f null and the
# same run writing rawvideo, so a comparison that lets one side write and the
# other not is worth up to 1.8x on its own.
#
# REPS runs per configuration, alternating the two binaries, median kept - the
# desktop on this machine moves fps by 2-3x between minutes.
set -u
ROOT=$(cd "$(dirname "$0")/.." && pwd)
CLIP=${CLIP:-"$ROOT/../testdata/bench/long_720p.h264"}
OURS=${OURS:-"$ROOT/h264dec_example"}
[ -x "$OURS" ] || OURS="$ROOT/h264dec_example.exe"
THREADS=${THREADS:-"1 6 8 10 12 16"}
REPS=${REPS:-5}
FRAMES=${FRAMES:-1500}
SOLO=${SOLO:-}

[ -x "$OURS" ] || { echo "build $OURS first"; exit 1; }
[ -f "$CLIP" ] || { echo "no clip at $CLIP (tests/gen_bench.sh writes it)"; exit 1; }
command -v ffmpeg >/dev/null || { echo "ffmpeg is not on PATH"; exit 1; }

med() { printf '%s\n' "$@" | tr ' ' '\n' | sed '/^$/d' | sort -n \
            | awk '{a[NR]=$1} END{if (NR) print a[int((NR+1)/2)]; else print 0}'; }

# SINK=1 makes both sides write the 2 GB of YUV instead of dropping it. That is
# the job an integrator actually runs, and it changes the ranking: FFmpeg's
# rawvideo output costs it far more than our stdio copy costs us.
if [ "${SINK:-0}" = "1" ]; then
    OURS_OUT=${OURS_OUT:-NUL}
    FF_ARGS='-f rawvideo -y NUL'
    MODE="both write YUV to the null device"
else
    OURS_OUT=${OURS_OUT:--}
    FF_ARGS='-f null -'
    MODE="both discard the pictures"
fi

echo "clip=$CLIP reps=$REPS frames=$FRAMES  ($MODE)"
for t in $THREADS; do
    o=""
    f=""
    for r in $(seq 1 "$REPS"); do
        o="$o $("$OURS" "$CLIP" "$OURS_OUT" "$t" 2>/dev/null | sed -n 's/.*fps=\([0-9.]*\).*/\1/p')"
        # ffprobe-style fps is unusable here: a run that ends inside one stats
        # interval prints fps=0.0, so take the wall time from -benchmark.
        rt=$(ffmpeg -hide_banner -benchmark -loglevel info -threads "$t" -i "$CLIP" \
                   $FF_ARGS 2>&1 | tr '\r' '\n' | sed -n 's/.*rtime=\([0-9.]*\)s.*/\1/p' | tail -1)
        f="$f $(LC_ALL=C awk -v x="$rt" -v n="$FRAMES" \
                  'BEGIN{if (x+0 > 0) printf "%.2f", n/x; else print 0}')"
    done
    om=$(med $o)
    fm=$(med $f)
    if [ -z "$SOLO" ]; then SOLO=$om; fi
    LC_ALL=C awk -v t="$t" -v o="$om" -v f="$fm" \
        'BEGIN{printf "threads=%-3d ours=%7.1f  ffmpeg=%7.1f  ffmpeg/ours=%.3f\n", t, o, f, (o>0)?f/o:0}'
done

# The other reference is this decoder with no synchronisation at all: N separate
# processes, each single-threaded, each decoding the whole clip. That is what
# embarrassingly-parallel work costs on this machine, so the gap between it and
# the threaded rows above is what the pool is paying for coordination.
if [ -n "${PROCS:-}" ]; then
    echo "-- concurrent single-thread instances of this decoder --"
    for n in $PROCS; do
        d=$(mktemp -d)
        best=0
        for r in 1 2 3; do
            rm -f "$d"/*.txt
            for i in $(seq 1 "$n"); do
                "$OURS" "$CLIP" "$OURS_OUT" 1 > "$d/$i.txt" 2>/dev/null &
            done
            wait
            tot=$(sed -n 's/.*fps=\([0-9.]*\).*/\1/p' "$d"/*.txt \
                      | awk '{s+=$1} END{printf "%.1f", s}')
            best=$(LC_ALL=C awk -v a="$best" -v b="$tot" 'BEGIN{print (b+0>a+0)?b+0:a+0}')
        done
        rm -rf "$d"
        LC_ALL=C awk -v n="$n" -v tot="$best" -v solo="$SOLO" \
            'BEGIN{printf "procs=%-3d total=%7.1f  speedup=%.2fx  per_process=%6.1f\n", n, tot, (solo>0?tot/solo:0), tot/n}'
    done
fi
