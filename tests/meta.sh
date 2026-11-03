#!/bin/sh
# Field order and picture metadata: decode every stream and compare the
# interlaced / top_field_first / repeat_pict triple the decoder exports with
# what ffprobe reports for the same stream. FFmpeg derives those flags from the
# decoding process and the picture timing SEI on its own, so agreement is an
# independent check of the pic_struct rules.
set -u

ROOT=$(cd "$(dirname "$0")/.." && pwd)      # h264dec/
DATA=${DATA:-"$ROOT/../testdata"}
META="$ROOT/build/meta_dump"
THREADS=${THREADS:-1}
TMP=${TMP:-/tmp/h264meta}
RTIMEOUT=${RTIMEOUT:-180}      # a wedged ffprobe must not hang the suite
mkdir -p "$TMP"

[ -x "$META" ] || { echo "build $META first"; exit 1; }
[ -d "$DATA" ] || { echo "no testdata in $DATA"; exit 1; }

ok=0 bad=0 skip=0
for f in "$DATA"/*.h264; do
    name=$(basename "$f" .h264)
    if ! timeout "$RTIMEOUT" ffprobe -v error -select_streams v \
        -show_entries frame=interlaced_frame,top_field_first,repeat_pict \
        -of csv=p=0 "$f" > "$TMP/$name.ref" 2>/dev/null || [ ! -s "$TMP/$name.ref" ]; then
        echo "== $name: SKIP (no reference frames)"
        skip=$((skip + 1))
        continue
    fi
    # decode order on both sides: ffprobe lists the frames as the decoder emitted
    "$META" "$f" "$THREADS" 1 > "$TMP/$name.our" 2>/dev/null
    cut -d, -f4,5,6 "$TMP/$name.our" > "$TMP/$name.our3"
    awk -F, 'NF >= 3 { print $1 "," $2 "," $3 }' "$TMP/$name.ref" > "$TMP/$name.ref3"

    no=$(wc -l < "$TMP/$name.our3"); nr=$(wc -l < "$TMP/$name.ref3")
    if [ "$no" != "$nr" ]; then
        echo "== $name: FAIL picture_count ours=$no ref=$nr"
        bad=$((bad + 1))
        continue
    fi
    d=$(paste "$TMP/$name.our3" "$TMP/$name.ref3" | \
        awk -F'\t' '{ split($1, a, ","); split($2, b, ",");
                      if (a[1] != b[1] || a[2] != b[2] || a[3] != b[3]) c++ }
                    END { print c + 0 }')
    if [ "$d" -eq 0 ]; then
        echo "== $name: PASS ($no pictures)"
        ok=$((ok + 1))
    else
        echo "== $name: FAIL mismatching_pictures=$d of $no"
        paste "$TMP/$name.our3" "$TMP/$name.ref3" | \
            awk -F'\t' '{ split($1, a, ","); split($2, b, ",");
                          if (a[1] != b[1] || a[2] != b[2] || a[3] != b[3]) {
                              print "    ours " a[1] "," a[2] "," a[3] \
                                    " ref " b[1] "," b[2] "," b[3]; n++ }
                          if (n > 3) exit }'
        bad=$((bad + 1))
    fi
done

echo "----"
echo "meta ok=$ok bad=$bad skip=$skip"
[ "$bad" -eq 0 ]
