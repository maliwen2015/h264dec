#!/bin/sh
# Re-open soak: does the process stay flat when the decoder is opened, drained
# and closed over and over, as it is when a channel is switched?
#
#   LONG    rounds of the long benchmark clip at several slots (pool pressure)
#   CHURN   rounds of a short stream at one slot (open/close paths)
set -u
ROOT=$(cd "$(dirname "$0")/.." && pwd)
DATA=${DATA:-"$ROOT/../testdata"}
BIN="$ROOT/build/soak"
LONG=${LONG:-30}
CHURN=${CHURN:-300}
LONG_THREADS=${LONG_THREADS:-4}
SOAK=${SOAK:-"$DATA/bench/long_720p.h264"}
SHORT=${SHORT:-"$DATA/main_cabac.h264"}

[ -x "$BIN" ] || { echo "build $BIN first"; exit 1; }

fail=0
for f in "$SOAK" "$SHORT"; do
    [ -f "$f" ] || { echo "no stream at $f"; exit 1; }
done

echo "== long stream, $LONG rounds at $LONG_THREADS slots, 8 MB limit"
"$BIN" "$SOAK" "$LONG_THREADS" "$LONG" 8 || fail=1

echo "== short stream, $CHURN rounds at 1 slot, 4 MB limit"
"$BIN" "$SHORT" 1 "$CHURN" 4 || fail=1

# the same file with the deblocker off, so a leak that only shows on one path
# still has to surface
echo "== short stream, $CHURN rounds, deblocking off"
"$BIN" "$SHORT" 1 "$CHURN" 4 1 || fail=1

echo "soak fail=$fail"
exit $fail
