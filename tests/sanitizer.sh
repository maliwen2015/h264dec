#!/bin/sh
# AddressSanitizer leg, run on a POSIX build (Linux/WSL: MinGW has no ASAN
# runtime here). Several of the motion-compensation kernels read a few bytes
# past their block on purpose - the planes carry a margin and the edge-emulation
# buffer is oversized - so the thing that has to be true is not "no over-read"
# but "no read outside the allocation", which is exactly what ASAN answers.
#
# UBSAN runs along with it. The corpus is clean on both: the packed motion
# vectors used to shift a 16-bit half into the sign bit of an int (148 reports
# across 30 streams) and one weighted-prediction offset used to shift a negative
# value, and both are fixed here, so a line from either sanitizer is a
# regression rather than a known state.
#
# The tree is left sanitizer-built; run make clean afterwards.
set -u
ROOT=$(cd "$(dirname "$0")/.." && pwd)
DATA=${DATA:-"$ROOT/../testdata"}
LONG=${LONG:-}
JOBS=$(nproc 2>/dev/null || echo 4)

cd "$ROOT" || exit 1
make clean >/dev/null 2>&1
make -j"$JOBS" CFLAGS="-O1 -g -std=c99 -fsanitize=address,undefined -fno-omit-frame-pointer" \
    all >/tmp/sanitizer_build.log 2>&1 || { tail -20 /tmp/sanitizer_build.log; exit 1; }
echo "build: warnings=$(grep -c 'warning:' /tmp/sanitizer_build.log)"

[ -f "$ROOT/h264dec_example" ] || { echo "no h264dec_example after the build"; exit 1; }

STREAMS=${1:-"$DATA/*.h264"}
LONG=${LONG:-"$DATA/bench/long_720p.h264"}
as=0; ub=0; rc=0; n=0
for f in $STREAMS $LONG; do
    [ -f "$f" ] || continue
    n=$((n + 1))
    "$ROOT/h264dec_example" "$f" - 1 >/tmp/sanitizer_run.log 2>&1
    [ $? -ne 0 ] && { echo "exit!=0: $(basename "$f")"; rc=$((rc + 1)); }
    a=$(grep -c 'ERROR: AddressSanitizer\|LeakSanitizer' /tmp/sanitizer_run.log)
    u=$(grep -c 'runtime error' /tmp/sanitizer_run.log)
    as=$((as + a)); ub=$((ub + u))
    [ "$a" -gt 0 ] && grep -m1 'ERROR: AddressSanitizer\|LeakSanitizer' /tmp/sanitizer_run.log | sed 's/^/  /'
done
echo "sanitizer: streams=$n asan_reports=$as nonzero_exits=$rc ubsan_lines=$ub"

# Damaged input as well: the corpus above is valid syntax, and the UB and
# out-of-bounds paths that matter are the ones a corrupt header reaches. The
# fuzz harness now looks for sanitizer lines itself.
if [ "${FUZZ:-1}" = 1 ]; then
    echo "-- damaged input through the sanitizer build --"
    DATA="$DATA" THREADS=1 SEEDS="${SEEDS:-1 2 3}" \
        BIN="$ROOT/h264dec_example" WORK="${FUZZ_WORK:-/tmp/h264fuzz_san}" \
        sh "$ROOT/tests/fuzz.sh" || rc=$((rc + 1))
    if [ -d "$DATA/10bit" ]; then
        DATA="$DATA/10bit" THREADS=1 SEEDS="${SEEDS:-1 2 3}" \
            BIN="$ROOT/h264dec_example" WORK="${FUZZ_WORK:-/tmp/h264fuzz_san}_10" \
            sh "$ROOT/tests/fuzz.sh" || rc=$((rc + 1))
    fi
fi

[ "$as" -eq 0 ] && [ "$ub" -eq 0 ] && [ "$rc" -eq 0 ] || exit 1
