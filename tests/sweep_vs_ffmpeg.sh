#!/bin/sh
# sweep_vs_ffmpeg.sh - a differential sweep over encoder settings the curated
# 27-stream corpus does not cover, decoded by FFmpeg and by this decoder and
# compared byte for byte at one, eight and twelve slots.
#
# Every case is one x264 configuration that moves a bitstream feature nobody
# else in the corpus reaches (High profile with CAVLC and 8x8 transforms, AU
# delimiters with repeated parameter sets, intra refresh with its recovery
# point, six references and four B frames, arbitrary slice boundaries,
# transform_bypass with CAVLC, and so on). A case whose encode x264 refuses is
# reported as SKIP, not as a pass, so the count of what really ran is honest.
#
# SWEEP=/path  where the generated streams land (default ../testdata/sweep)
# SLOTS="1 8 12"  slot counts to compare, beyond the one-slot reference
set -u
ROOT=$(cd "$(dirname "$0")/.." && pwd)
SWEEP=${SWEEP:-"$ROOT/../testdata/sweep"}
SLOTS=${SLOTS:-"1 8 12"}
FRAMES=${FRAMES:-30}
SENS_EVERY=${SENS_EVERY:-5}   # also cross-check the no-deblocking path every Nth case
# BIN can be pointed at another build's binary, as selfcheck.sh, fuzz.sh and
# compare.sh can - which is how the whole-program binary (make lto) is swept
BIN=${BIN:-}
if [ -z "$BIN" ]; then
    BIN="$ROOT/h264dec_example"
    [ -x "$BIN" ] || BIN="$ROOT/h264dec_example.exe"
fi
RTIMEOUT=${RTIMEOUT:-180}

command -v ffmpeg  >/dev/null || { echo "ffmpeg is not on PATH"; exit 1; }
command -v ffprobe >/dev/null || { echo "ffprobe is not on PATH"; exit 1; }
[ -x "$BIN" ] || { echo "build $BIN first"; exit 1; }
mkdir -p "$SWEEP"

pass=0 fail=0 skip=0
case_no=0

run_case() {
    name=$1; src=$2; shift 2
    case_no=$((case_no + 1))
    stream="$SWEEP/$name.h264"

    # -frames:v caps every case: the synthetic sources that are not testsrc2 have
    # no duration option at all, and a fixed frame count keeps the sweep cheap
    if ! ffmpeg -y -hide_banner -loglevel error -f lavfi -i "$src" -an -c:v libx264 \
              -r 30000/1001 -frames:v "$FRAMES" "$@" -pix_fmt yuv420p \
              "$stream" 2>"$SWEEP/$name.enc.log"; then
        echo "== $name: SKIP (x264 refused: $(head -c 120 "$SWEEP/$name.enc.log" | tr -d '\n'))"
        skip=$((skip + 1)); return
    fi
    if [ ! -s "$stream" ]; then
        echo "== $name: SKIP (empty elementary stream)"
        skip=$((skip + 1)); return
    fi

    if ! timeout "$RTIMEOUT" ffmpeg -v error -threads 1 -y -i "$stream" -pix_fmt yuv420p \
             -f rawvideo "$SWEEP/$name.ref.yuv" 2>/dev/null || [ ! -s "$SWEEP/$name.ref.yuv" ]; then
        echo "== $name: SKIP (reference decode failed)"
        skip=$((skip + 1)); return
    fi

    # the control under the whole comparison: FFmpeg's own reference has to be the
    # same bytes at one and at twelve threads, or a mismatch here would be ambiguous
    if ! timeout "$RTIMEOUT" ffmpeg -v error -threads 12 -y -i "$stream" -pix_fmt yuv420p \
             -f rawvideo "$SWEEP/$name.ref12.yuv" 2>/dev/null ||
       ! cmp -s "$SWEEP/$name.ref.yuv" "$SWEEP/$name.ref12.yuv"; then
        echo "== $name: FAIL (the FFmpeg reference is not thread-stable here)"
        fail=$((fail + 1)); rm -f "$SWEEP/$name.ref12.yuv"; return
    fi
    rm -f "$SWEEP/$name.ref12.yuv"

    info=$(ffprobe -v error -select_streams v -show_entries \
           stream=profile,codec_name,width,height,field_order -of csv=p=0 "$stream" 2>/dev/null)
    refsz=$(stat -c%s "$SWEEP/$name.ref.yuv")

    bad=""
    for t in $SLOTS; do
        : > "$SWEEP/$name.$t.log"
        timeout "$RTIMEOUT" "$BIN" "$stream" "$SWEEP/$name.$t.yuv" "$t" \
            >"$SWEEP/$name.$t.log" 2>&1
        code=$?
        if [ "$code" -gt 1 ]; then
            bad="$bad t$t:exit$code"
            continue
        fi
        if [ ! -s "$SWEEP/$name.$t.yuv" ]; then
            bad="$bad t$t:no-output"
            continue
        fi
        if ! cmp -s "$SWEEP/$name.ref.yuv" "$SWEEP/$name.$t.yuv"; then
            n=$(cmp -l "$SWEEP/$name.ref.yuv" "$SWEEP/$name.$t.yuv" 2>/dev/null | wc -l)
            oursz=$(stat -c%s "$SWEEP/$name.$t.yuv")
            # same comparison with the in-loop filter off on both sides, which
            # separates a reconstruction error from a deblocking one
            nd=""
            if timeout "$RTIMEOUT" ffmpeg -v error -y -skip_loop_filter all -i "$stream" \
                   -pix_fmt yuv420p -f rawvideo "$SWEEP/$name.ndref.yuv" 2>/dev/null; then
                "$BIN" "$stream" "$SWEEP/$name.nd$t.yuv" "$t" 0 1 >/dev/null 2>&1
                m=$(cmp -l "$SWEEP/$name.ndref.yuv" "$SWEEP/$name.nd$t.yuv" 2>/dev/null | wc -l)
                nd=" nodeblock=$m"
            fi
            bad="$bad t$t:diff=$n/$oursz$nd"
            rm -f "$SWEEP/$name.ndref.yuv" "$SWEEP/$name.nd$t.yuv"
        fi
        au=$(sed -n 's/.*au_errors=\([0-9]*\).*/\1/p' "$SWEEP/$name.$t.log" | tail -1)
        [ -z "$au" ] || [ "$au" = "0" ] || bad="$bad t$t:au_errors=$au"
        rm -f "$SWEEP/$name.$t.yuv"
    done

    # every SENS_EVERY-th case also runs the no-deblocking path on both sides. It
    # is a second bit-exact comparison - ours with the filter off against FFmpeg
    # with -skip_loop_filter all - and it is the check that this script is not
    # quietly comparing two empty buffers: on the cases that do filter, turning
    # the filter off moves hundreds of thousands of bytes
    if [ "$SENS_EVERY" -gt 0 ] && [ $((case_no % SENS_EVERY)) -eq 0 ] && [ -z "$bad" ]; then
        if timeout "$RTIMEOUT" ffmpeg -v error -y -skip_loop_filter all -i "$stream" \
               -pix_fmt yuv420p -f rawvideo "$SWEEP/$name.ndref.yuv" 2>/dev/null &&
           "$BIN" "$stream" "$SWEEP/$name.nd.yuv" 1 0 1 >/dev/null 2>&1; then
            if cmp -s "$SWEEP/$name.ndref.yuv" "$SWEEP/$name.nd.yuv"; then
                moved=$(cmp -l "$SWEEP/$name.ref.yuv" "$SWEEP/$name.nd.yuv" 2>/dev/null | wc -l)
                echo "== $name: PASS  ($(ffprobe -v error -select_streams v -show_entries \
                      stream=profile,width,height,field_order -of csv=p=0 "$stream"), \
ref=$refsz bytes, slots: $SLOTS; no-filter run matches, $moved bytes off the filtered one)"
                pass=$((pass + 1)); rm -f "$SWEEP/$name.ndref.yuv" "$SWEEP/$name.nd.yuv"
                return
            fi
            bad=" no-filter-mismatch"
        fi
        rm -f "$SWEEP/$name.ndref.yuv" "$SWEEP/$name.nd.yuv"
    fi

    if [ -n "$bad" ]; then
        echo "== $name: FAIL$bad  ($info, ref=$refsz bytes)"
        fail=$((fail + 1))
    else
        echo "== $name: PASS  ($info, ref=$refsz bytes, slots: $SLOTS)"
        pass=$((pass + 1))
    fi
}

S="testsrc2=size=176x144:rate=25:duration=1.0"
M="testsrc2=size=320x240:rate=25:duration=1.0"
H="testsrc2=size=640x360:rate=25:duration=0.8"
LIFE="life=size=176x144:rate=25:seed=17"
MB="mandelbrot=size=176x144:rate=25"
CELL="cellauto=size=176x144:rate=25:rule=110"
GRAY="color=c=gray:size=176x144:rate=25"

C="keyint=15:min-keyint=1:scenecut=0"

# High profile is not required to use CABAC: 8x8 transform with CAVLC is legal
# and nothing else in the corpus emits it.
run_case high_cavlc_8x8 "$M" -profile:v high -level 3.1 -x264-params \
    "$C:bframes=3:ref=4:slices=2:cabac=0:8x8dct=1:weightb=1:b-pyramid=normal"

# one access unit per picture with AU delimiters, HRD and re-sent parameter sets
run_case aud_repeat_hrd "$S" -profile:v main -level 3.1 -x264-params \
    "$C:bframes=2:ref=2:slices=1:cabac=1:aud=1:repeat-headers=1:nal-hrd=cbr:bitrate=400:vbv-maxrate=400:vbv-bufsize=400"

# arbitrary slice boundaries from the encoder's slice-threading mode
run_case sliced_threads "$M" -profile:v high -level 3.1 -x264-params \
    "$C:bframes=2:ref=2:slices=0:cabac=1:8x8dct=1:sliced_threads=1:threads=4"

# intra refresh: recovery point instead of IDR, and the refresh ramp itself
run_case intra_refresh "$M" -profile:v main -level 3.1 -x264-params \
    "$C:intra-refresh=1:bframes=0:ref=3:slices=2:cabac=1:rc-lookahead=10:keyint=60"

# six references, four B frames, high pyramid, temporal direct, no weightb
run_case ref6_b4_direct "$M" -profile:v high -level 3.1 -x264-params \
    "$C:bframes=4:b-adapt=2:ref=6:slices=1:cabac=1:8x8dct=1:b-pyramid=high:direct=temporal:weightb=0:weightp=0"

# many slices in a small picture, CAVLC, baseline: per-slice state on one row
run_case baseline_8slices "$S" -profile:v baseline -level 3.0 -x264-params \
    "$C:bframes=0:ref=1:slices=8:cabac=0"

# every picture an IDR: output order, DPB flush, and no inter prediction at all
run_case all_intra "$M" -profile:v main -level 3.1 -x264-params \
    "keyint=1:min-keyint=1:scenecut=0:bframes=0:ref=1:slices=1:cabac=1"

# chroma QP offsets both ways with 8x8 transforms
run_case chroma_qp_plus "$S" -profile:v high -level 3.1 -x264-params \
    "$C:bframes=2:ref=2:slices=2:cabac=1:8x8dct=1:chroma-qp-offset=6"
run_case chroma_qp_minus "$S" -profile:v high -level 3.1 -x264-params \
    "$C:bframes=2:ref=2:slices=2:cabac=1:8x8dct=1:chroma-qp-offset=-6"

# 1080p: coded 1920x1088 with a vertical crop, the deepest DPB shape here
run_case crop_1080p "testsrc2=size=1920x1080:rate=25:duration=0.4" -profile:v high \
    -level 4.0 -x264-params "keyint=15:min-keyint=1:scenecut=0:bframes=3:ref=4:slices=1:cabac=1:8x8dct=1"

# odd coded size on both dimensions
run_case odd_both_dims "testsrc2=size=418x234:rate=25:duration=0.8" -profile:v high \
    -level 3.1 -x264-params "$C:bframes=2:ref=2:slices=3:cabac=1:8x8dct=1"

# near-transparent and near-empty quality extremes
run_case crf5_heavy "$LIFE" -profile:v high -level 3.1 -preset medium -x264-params \
    "$C:crf=5:bframes=2:ref=4:slices=1:cabac=1:8x8dct=1:subme=7:trellis=2"
run_case qp51_sparse "$CELL" -profile:v main -level 3.1 -x264-params \
    "$C:qp=51:bframes=3:ref=3:slices=1:cabac=1"

# flat grey: nearly every macroblock is a P-skip with no residual
run_case grey_skipall "$GRAY" -profile:v main -level 3.1 -x264-params \
    "$C:bframes=3:ref=4:slices=1:cabac=1:weightp=2:b-pyramid=normal"

# constrained baseline with the deblocking filter switched off in the PPS
run_case conf_base_nodeblock "$S" -profile:v baseline -level 3.0 \
    -x264-params "$C:bframes=0:ref=2:slices=2:cabac=0:constrained-intra=1:no-deblock=1"

# the in-loop filter off together with 8x8 transforms and B frames, which the
# one nodeblock stream in the corpus (no B frames, 4x4) does not reach
run_case high_nodeblock_bframes "$M" -profile:v high -level 3.1 \
    -x264-params "$C:bframes=3:ref=4:slices=1:cabac=1:8x8dct=1:no-deblock=1:b-pyramid=normal"

# scene cuts forced to IDR as fast as the encoder allows, so the DPB is flushed
# and refilled constantly while other pictures are still in flight
run_case idr_storm "$S" -profile:v high -level 3.1 -x264-params \
    "keyint=8:min-keyint=1:scenecut=100:bframes=2:ref=4:slices=1:cabac=1:8x8dct=1:b-pyramid=normal"

# partitions restricted to 8x8 shapes at high profile
run_case parts_8x8_only "$M" -profile:v high -level 3.1 -x264-params \
    "$C:bframes=2:ref=3:slices=1:cabac=1:8x8dct=1:partitions=p8x8,i8x8"

# a long P chain with six references and no B frames anywhere
run_case long_p_chain "$H" -profile:v main -level 3.1 -x264-params \
    "keyint=250:min-keyint=250:scenecut=0:bframes=0:ref=6:slices=1:cabac=1"

# heavy adaptive quantisation on complex synthetic detail
run_case aq3_autovbr "$MB" -profile:v high -level 3.1 -x264-params \
    "$C:bframes=3:ref=4:slices=2:cabac=1:8x8dct=1:aq-mode=3:aq-strength=3"

# zero-latency tuning: no lookahead, no B frames, no weighted prediction
run_case tune_zerolatency "$S" -profile:v main -level 3.1 -tune zerolatency \
    -x264-params "keyint=15:min-keyint=1:scenecut=0:cabac=1:slices=2:ref=2"

# twelve CABAC slices in one picture, so several slices share a macroblock row
run_case cabac_12slices "$M" -profile:v main -level 3.1 -x264-params \
    "$C:bframes=2:ref=3:slices=12:cabac=1"

# open GOP with B-pyramid high: leading B pictures before the first IDR frame
run_case open_gop_bhigh "$S" -profile:v high -level 3.1 -x264-params \
    "$C:bframes=4:b-adapt=2:ref=4:slices=1:cabac=1:8x8dct=1:open-gop=1:b-pyramid=high"

# MBAFF with bottom-field-first flag, B frames and 8x8 transforms
run_case mbaff_bff_weighted "$S" -profile:v high -level 3.1 -flags +ildct+ilme \
    -x264-params "$C:tff=0:bff=1:bframes=3:ref=4:slices=2:cabac=1:8x8dct=1:weightp=2:weightb=1"

# CAVLC MBAFF with eight slices
run_case mbaff_cavlc_8slices "$S" -profile:v main -level 3.1 -flags +ildct+ilme \
    -x264-params "$C:tff=1:mb-aff=1:bframes=2:ref=2:slices=8:cabac=0"

# separate field pictures with the 8x8 transform and two references
run_case paff_8x8 "$S" -profile:v high -level 3.1 -flags +ildct+ilme \
    -x264-params "$C:tff=1:mb-aff=0:bframes=2:ref=4:slices=1:cabac=1:8x8dct=1:weightp=2"

# transform bypass with CAVLC, which no lossless stream in the corpus does
run_case lossless_cavlc "$S" -profile:v high444 -preset ultrafast -x264-params \
    "$C:qp=0:bframes=0:ref=1:slices=2:cabac=0"

# qp from the rate-distortion optimiser instead of mb-tree, per-MB QP variation
run_case qprd_mbtree_off "$CELL" -profile:v high -level 3.1 -x264-params \
    "$C:crf=23:bframes=2:ref=2:slices=1:cabac=1:8x8dct=1:mbtree=0:qprd=1:me=esa:subme=6"

# spatial denoising: residuals that are small but never exactly zero
run_case nr_dense "$LIFE" -profile:v main -level 3.1 -x264-params \
    "$C:bframes=3:ref=4:slices=1:cabac=1:nr=200"

echo "----"
echo "sweep pass=$pass fail=$fail skip=$skip  streams in $SWEEP"
[ "$fail" -eq 0 ]
