#!/usr/bin/perl
# weave_fields.pl in out width height
# Turns a progressive 50p raw YUV into a 25i one: every output frame takes its
# even rows from one input frame and its odd rows from the next, which is what
# an interlaced camera would produce. x264 then codes field macroblocks.
use strict;
use warnings;

my ($in, $out, $w, $h) = @ARGV;
open(my $I, '<', $in) or die "$in: $!";
open(my $O, '>', $out) or die "$out: $!";
binmode $I;
binmode $O;

my $fs   = $w * $h * 3 / 2;
my @plan = ([$w, $h, 0], [ $w / 2, $h / 2, $w * $h],
            [ $w / 2, $h / 2, $w * $h * 5 / 4]);

while (read($I, my $a, $fs) == $fs && read($I, my $b, $fs) == $fs) {
    my $frame = '';
    for my $p (@plan) {
        my ($pw, $ph, $off) = @$p;
        for my $r (0 .. $ph - 1) {
            $frame .= substr(($r & 1) ? $b : $a, $off + $r * $pw, $pw);
        }
    }
    print $O $frame;
}
