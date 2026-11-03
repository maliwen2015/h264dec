#!/bin/sh
# Damage every test stream and require the decoder to stay safe: it may report
# errors and skip pictures, but it must never crash and never hang.
#
#   trunc - cut the file short (truncated NAL, missing slices, missing tail)
#   flip  - toggle random bits (corrupt syntax, impossible field values)
#   drop  - remove a run of bytes in the middle (lost slices / whole pictures)
#   junk  - insert random bytes (false start codes, misparsed payloads)
set -u
ROOT=$(cd "$(dirname "$0")/.." && pwd)
DATA=${DATA:-"$ROOT/../testdata"}
# BIN points the harness at another build, the way selfcheck.sh does: another
# platform's binary, or an emulated one through a qemu wrapper
BIN=${BIN:-}
if [ -z "$BIN" ]; then
    BIN="$ROOT/h264dec_example"
    [ -x "$BIN" ] || BIN="$ROOT/h264dec_example.exe"
fi
THREADS=${THREADS:-1}
TIMEOUT=${TIMEOUT:-90}
SEEDS=${SEEDS:-"1 2 3"}
WORK=${WORK:-/tmp/h264fuzz}

[ -x "$BIN" ] || { echo "build $BIN first"; exit 1; }
rm -rf "$WORK"; mkdir -p "$WORK"

mutate() {
    perl -e '
        my ($in, $out, $mode, $seed) = @ARGV;
        open(my $f, "<", $in) or die "open $in: $!";
        binmode $f; local $/; my $d = <$f>; close $f;
        my $n = length $d;
        srand($seed * 7919 + $mode =~ tr/a-z//s);
        if ($mode eq "trunc") {
            substr($d, int($n * (0.05 + 0.9 * rand()))) = "";
        } elsif ($mode eq "flip") {
            my $k = 1 + int(rand(40));
            for (1 .. $k) {
                my $p = int(rand($n));
                substr($d, $p, 1) = chr(ord(substr($d, $p, 1)) ^ (1 << int(rand(8))));
            }
        } elsif ($mode eq "drop") {
            my $s = int(rand($n / 2));
            substr($d, $s, int(rand($n / 2))) = "";
        } elsif ($mode eq "junk") {
            my $s = int(rand($n));
            my $add = "";
            $add .= chr(int(rand(256))) for 1 .. (1 + int(rand(200)));
            substr($d, $s, 0, $add);
        } elsif ($mode eq "hdr") {
            # damage syntax headers rather than coefficient payloads: walk the
            # NAL start codes and flip a few bits in the first 24 bytes of each,
            # which is where SPS, PPS and slice headers live
            my $pos = 0;
            while (($pos = index($d, "\x00\x00\x01", $pos)) >= 0) {
                my $s = $pos + 3;
                my $e = index($d, "\x00\x00\x01", $s);
                $e = $n if $e < 0;
                if ($s < $e && rand() < 0.7) {
                    my $lim = $s + 24;
                    $lim = $e if $lim > $e;
                    my $k = 1 + int(rand(4));
                    for (1 .. $k) {
                        my $p = $s + int(rand($lim - $s));
                        substr($d, $p, 1) = chr(ord(substr($d, $p, 1)) ^ (1 << int(rand(8))));
                    }
                }
                $pos = $e;
            }
        }
        open(my $o, ">", $out) or die "open $out: $!";
        binmode $o; print $o $d; close $o;
    ' "$1" "$2" "$3" "$4"
}

bad=0; runs=0
for f in "$DATA"/*.h264; do
    name=$(basename "$f" .h264)
    for mode in trunc flip drop junk hdr; do
        for seed in $SEEDS; do
            v="$WORK/$name.$mode.$seed.h264"
            mutate "$f" "$v" "$mode" "$seed" || { echo "mutate failed: $v"; continue; }
            log="$WORK/$name.$mode.$seed.log"
            # a child that cannot even start (process table / memory pressure
            # while spawning hundreds of them) leaves an empty log and reports
            # 127 or 139; retry before calling that a decoder failure
            for try in 1 2 3; do
                timeout "$TIMEOUT" "$BIN" "$v" /dev/null "$THREADS" >"$log" 2>&1
                code=$?
                runs=$((runs + 1))
                [ "$code" -le 1 ] && break
                [ "$try" = 3 ] && break
                [ "$code" -ne 127 ] && [ -s "$log" ] && break
                sleep 1
            done
            if [ "$code" -gt 1 ]; then
                if [ "$code" -eq 124 ]; then
                    echo "HANG    $name $mode seed=$seed"
                elif [ "$code" -eq 127 ] || [ ! -s "$log" ]; then
                    # 127 is an exec failure, not a signal: the child never got
                    # to run, which happens under process-table pressure while
                    # hundreds of them are spawned
                    echo "NOSTART $name $mode seed=$seed exit=$code"
                else
                    echo "CRASH   $name $mode seed=$seed exit=$code"
                    tail -3 "$log" | sed 's/^/          /'
                fi
                bad=$((bad + 1))
            elif grep -q "underflow\|assert\|Debug error\|runtime error\|AddressSanitizer\|LeakSanitizer" "$log"; then
                echo "STATE   $name $mode seed=$seed"
                grep -m2 "underflow\|assert\|Debug error\|runtime error\|AddressSanitizer\|LeakSanitizer" "$log" | sed 's/^/          /'
                bad=$((bad + 1))
            fi
        done
    done
done

echo "----"
echo "threads=$THREADS variants=$runs problems=$bad"
[ "$bad" -eq 0 ]
