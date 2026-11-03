#!/bin/sh
# Two slots is the pool shape that can strand work on the filter list: one worker
# and the decoder owner, and every thread can end up asleep - the owner waiting for
# a job slot, the worker waiting for its commit turn - while the list still holds
# rows of a picture nobody will take. That is a hang, not a slow run, so the test is
# simply "does this decode finish": each run is bounded, and a run that does not
# finish is the failure.
#
# Measured on the build before a released claim woke anybody, at two slots: 2 blocked
# runs out of 40 on the 720p clip, 1 out of 20 on the interlaced one, none out of 20 on
# 1080p, and none out of 20 at three slots - with two workers there is always a thread
# awake that can take the list. So this runs the two clips that do reach it, twenty
# times each, which fails the gate on roughly one pre-fix run in eight.
set -u
ROOT=$(cd "$(dirname "$0")/.." && pwd)
DATA=${DATA:-"$ROOT/../testdata"}
BIN=${BIN:-}
if [ -z "$BIN" ]; then
    BIN="$ROOT/h264dec_example"
    [ -x "$BIN" ] || BIN="$ROOT/h264dec_example.exe"
fi
RUNS=${RUNS:-20}
SLOTS=${SLOTS:-2}
TIMEOUT=${TIMEOUT:-120}
CLIPS=${CLIPS:-"bench/long_720p.h264 bench/long_paff_720p.h264"}

[ -x "$BIN" ] || { echo "build $BIN first"; exit 1; }

blocked=0; failed=0; runs=0
for slot in $SLOTS; do
    for clip in $CLIPS; do
        f="$DATA/$clip"
        [ -f "$f" ] || { echo "no clip at $f"; exit 1; }
        i=1
        while [ "$i" -le "$RUNS" ]; do
            runs=$((runs + 1))
            timeout "$TIMEOUT" "$BIN" "$f" - "$slot" >/dev/null 2>&1
            code=$?
            if [ "$code" = "124" ]; then
                echo "BLOCKED  $(basename "$clip") at $slot slots, run $i:" \
                     "the filter list kept a picture nobody took"
                blocked=$((blocked + 1))
            elif [ "$code" != "0" ]; then
                echo "FAILED   $(basename "$clip") at $slot slots, run $i (exit $code)"
                failed=$((failed + 1))
            fi
            i=$((i + 1))
        done
    done
done
echo "claim hammer: runs=$runs slots=[$SLOTS] blocked=$blocked failed=$failed"
[ "$blocked" = "0" ] && [ "$failed" = "0" ] || exit 1
