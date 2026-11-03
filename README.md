h264dec
=======

A from-scratch H.264/AVC decoder (ITU-T H.264, Baseline / Main / High /
High 10 / High 4:2:2 / High 4:4:4 Predictive profiles; 8-bit 4:2:0 and 10-bit
4:0:0, 4:2:0, 4:2:2, 4:4:4) with a frame-threaded worker pool and SSE2
kernels on x86 (plus an AVX2 one for 16-wide luma interpolation) or NEON on ARM,
for motion compensation, the inverse transforms, the in-loop deblocking filter
and weighted prediction. The public API mirrors `avsplusdec`: open a decoder,
send one access unit at a time, receive ref-counted pictures.

Algorithms and constant tables are derived from ITU-T H.264 and from FFmpeg's
H.264 decoder (LGPL-2.1+), which is used as the reference implementation
throughout: outputs are compared byte for byte against it. **If you ship this,
you carry the LGPL obligations of that derived code.**

The current feature and performance comparison against FFmpeg 9.0.2 - profiles
and chroma formats accepted, throughput at 1 to 12 slots on progressive and
interlaced clips, file-to-YUV behaviour, memory, and the remaining gap - is in
[ANALYSIS.md](ANALYSIS.md), with the reproducible feature sweep in
`tests/feature_matrix.sh`.

Build
-----

MSYS2 / MinGW-w64 (GCC 16), or gcc/clang on POSIX with the pthread worker pool
(`src/h264_thread.c`) - that path is verified too: Ubuntu 22.04 under WSL2,
GCC 11.4, builds warning-free with `-lpthread` (needed from glibc before 2.34,
so Ubuntu 20.04 and older need it as well, which is why `LIBS` carries it on
non-Windows). Decoded output is byte-identical between the two systems: 27 of
27 streams match the Windows build at one slot, match it again at six and at
twelve, and match it once more with the
scalar path forced, while `dsp_check`, `sei_check` and `soak` are green there too.
The whole-program build agrees across the two systems as well: `make lto` on Linux
strips the IL out of its archive the same way, a `-O2` consumer links that archive
without asking for `-flto` and passes `api_check`, and its binary reproduces the
Windows YUVs byte for byte at one and eight slots.

    mingw32-make -j8            # -> libh264dec.a + h264dec_example
    mingw32-make check          # bit-exact suite + kernel checks + soak
    mingw32-make check10        # 10-bit corpus (4:0:0/4:2:0/4:2:2/4:4:4), bit-exact at 1/8/12 slots
    mingw32-make fuzz           # damaged-stream safety run
    mingw32-make soak           # re-open / channel-switch leak soak, long form
    mingw32-make selfcheck      # one-slot reference, then 0/2/3/6/8/10/12 slots
    mingw32-make pgo            # two-pass profile-guided build, see below
    mingw32-make lto            # whole-program build -> h264dec_example_lto
    mingw32-make bench          # throughput against FFmpeg, same sink both sides
    mingw32-make features       # profile/chroma-format matrix against FFmpeg
    mingw32-make stability      # determinism and damaged-stream safety, both decoders
    mingw32-make clean

The library builds at the plain x86-64 baseline. The vector kernels carry their
own target attribute (`#pragma GCC target("sse2")`, and `"avx2"` for the one
AVX2 kernel) and are selected at run time from CPUID and XGETBV
(`src/h264_cpu.c`), so one binary runs on any x86-64 and uses the SSE2 or AVX2
kernels where the CPU and the OS state allow it. `H264_DISABLE_SIMD=1` forces the
scalar path, which is also what `tests/dsp_check.c` compares against. The build
is `-O3`, because 47% of single-thread time is entropy decoding and the C paths
are what the optimizer works on: +5.8% at one slot, +3.6% at ten, output
byte-identical on both operating systems. `-flto` is worth more again, and a great
deal more once a picture is shared out - 1.3% at one slot against these numbers,
19-23% at twelve - but its link takes minutes and its archive has to be kept fat to
stay linkable, so it is `make lto` rather than the default. One file is compiled
differently again:
`src/h264_cabac.c` gets `-fno-tree-vectorize`, because the coefficient loop it
spends most of its time in walks `last_coeff -> first_coeff -> run -> level`,
where each step needs the previous one, so the vectorizer can only add bookkeeping.
That flag on that file is worth 1.051x at one slot on 720p, 1.050x on 1080p and
1.040x on the interlaced clip - and applying it tree-wide would be worse, 1.042x,
because the scalar DSP fallbacks do benefit from the vectorizer. For a build tuned
to one machine, append e.g. `CFLAGS="-O3 -march=native"`.

**Which SIMD, and chosen where.** An ARM build gets NEON instead, and the choice
is made in two steps so no call site has to know which ISA it is talking to.
`src/h264_internal.h` decides at
compile time - `__i386__`/`__x86_64__` selects the SSE2 kernels,
`__ARM_NEON` selects the NEON ones, neither selects none - and defines `H264_SIMD`
(are there vector kernels at all), `H264_HAVE_SIMD` (which flag to ask) and a
`h264_*_simd` name for every kernel. `src/h264_cpu.c` then decides at load time:
CPUID and XGETBV on x86, and on ARM an aarch64 CPU has NEON by architecture so
there is nothing to ask, while an ARMv7 build would read `HWCAP_NEON` from the
kernel. Every flag starts at 0, so a probe that cannot answer is a scalar decode,
and `H264_DISABLE_SIMD=1` forces that. Each of the 14 dispatch sites is one
`if (H264_HAVE_SIMD)` in front of a `_simd` call with the scalar call after it,
and the one x86-only branch - the AVX2 quarter-pel, whose second half of the
register file has no NEON counterpart - carries its own `H264_SIMD_X86` guard.

`src/h264_dsp_neon.c` is that translation of the SSE2 file, kernel for kernel, at
the same eight int16 lanes per 128-bit register: the 6-tap interpolations, the
integer-motion copy and average, bilinear chroma, weighted and biweighted
prediction, the full average, the 4x4 and DC transforms, and the four deblocking
filters in both memory layouts with the same `xstride != 1` gate the SSE2 path
uses, because the transposed chroma layout measured slower there too. It keeps
the scalar reference's exact arithmetic - truncating int16 stages, saturating
narrows where the C clips, arithmetic shifts where the C uses `>>`.

Three mapping differences are worth naming, because two of them were bugs on the
way in. `pavgusb` rounds, and NEON's `vhadd_u8` does not: the rounding
instruction `vrhadd_u8` is an ARMv8.1 extension, so the average is the
truncating halve plus the low bit of the sum, which is the low bit of `a^b` (the
first version used `a|b`, which is wrong for 255+255 - caught by the sample
sweep, not by reasoning). And SSE2's `unpacklo`/`unpackhi` are NEON's *zip*
family, not its *trn* family: trn takes even- and odd-numbered lanes, zip
interleaves halves, and mapping one to the other silently broke the 8x8 transpose
the deblocking pass loads and stores through, and the 4x4 transpose inside the
IDCT. `_mm_andnot_si128(m, x)` also has its operands the other way round in
`vbicq_s16(x, m)`, which is a third bug that made every blended filter write
through the wrong mask.

No ARM hardware was used for any of this, so **the NEON path is verified for
correctness and not for speed**: `qemu-aarch64` runs the cross-built binary far
too slowly for a cycle count to mean anything, and the real figure needs an ARM
board. What was checked, on a `aarch64-linux-gnu-gcc` build running under qemu:

| check on the ARM build | result |
| --- | --- |
| `dsp_check` - every NEON kernel against its scalar C reference, sample by sample | 0 mismatching cases |
| the 27-stream corpus, NEON at 1, 8 and 12 slots, against the x86 build's output | 27/27 byte-identical at each |
| the same corpus with `H264_DISABLE_SIMD=1`, i.e. the fallback on ARM | 27/27 byte-identical |
| the 29 differential-sweep streams (MBAFF, PAFF, CAVLC with 8x8, lossless, intra refresh) | 29/29 byte-identical |
| `fuzz` over 324 damaged variants, NEON and forced-scalar | 0 problems either way |

The same three bugs the qemu sweep caught are the reason it is worth having: the
first ARM run reported 408,145 mismatching samples and both transposes and one
rounding rule were wrong, and the check only went quiet when all three were
fixed. The x86 side re-ran clean through `make check`, `make selfcheck` at seven
slot counts and `make fuzz` after the dispatch refactor, and the NEON object is
an empty translation unit there (`.text` of zero bytes), which is the same
arrangement that lets a build without SSE2 work at all.


The CABAC syntax decoder is the other place the compiler needs pushing: the
engine's range, low and bit count live in `sl->cabac`, so unless the twelve-odd
`decode_cabac_*()` helpers are inside the macroblock decode itself, every bin
reloads them. `h264_decode_mb_cabac()` used to carry
`__attribute__((flatten))` for that reason - 1.4% at one slot on 720p, 2.0% on
1080p, 0.5% on the interlaced clip when it was measured, for 150 KB of text.
Once the rest of the entropy path had been tuned the forced inlining stopped
paying: dropping it measured 1.005x on both clips (medians of thirteen paired
runs), and the object is 10% smaller. The residual decode is a separate function
now, which costs nothing measurable: it keeps `low` and `range` in its own local
across the coefficient stores, and that is where the locality pays. That is also
where the reference's remaining single-thread speed comes from: its hand asm
keeps that state in registers across a macroblock without needing the compiler
to inline at all.

Running out of `sl->cabac` does not fully free it, because a store through
`block[]` or through any of the slice's caches may alias that struct, so every
bin would reload `low` and `range` around them, which in the coefficient loop
puts a pair of loads and stores on top of each bin.
`decode_cabac_residual_internal()` copies `sl->cabac` into a local for the
duration of the block and writes it back after the block store - a local no
caller-visible pointer reaches, so nothing can alias it. That is
worth 2.3% at one slot on 720p, 2.2% on 1080p and 2.0% on the interlaced clip
(medians of nine paired runs), and 0.6-1.8% is still left at ten slots, where
the serial entropy decode is what a worker is doing while it waits. The file
comes out 392 instructions shorter.

The per-bin path also had two instructions that were avoidable. `!(c->low &
CABAC_MASK)` compiled to a mov-and pair rather than one 16-bit `test`, in the
engine and in both bypass decoders, and the normalization-shift table was
indexed through a signed int that gcc sign-extended before every load. Using
`!(uint16_t)c->low` and unsigned index arithmetic removes both. Paired medians
of thirteen and eleven runs: 1.009x at 720p and 1.005x at 1080p; together with
dropping flatten the two changes are 1.010x and 1.008x.

The same trick over the whole macroblock - one local, with the twelve-odd syntax
helpers reached through a pointer to it instead of through `sl->` - is what the
reference's asm gets for free, but it means re-threading some sixty call sites
for a smaller prize: on the 60s 720p clip the residual decoder already carries
71.9% of the regular bins and 89.3% of the bypass ones, so the layers this copy
does not yet cover hold under a third of the bins and removing the same
forwarding traffic there is worth under 1%. `make CFLAGS="-O3 -fipa-pta"`, which
is the flag that would let the compiler work the non-aliasing out across those
helpers instead of being told, measures 0.999x on 720p, 0.995x on 1080p and
1.010x on the interlaced clip at one slot - no effect, so it stays off and the
hand re-threading stays unpromising.

`make pgo` builds twice: once instrumented, then runs the whole corpus through it
(`tests/pgo_train.sh` - 61 runs, every bit-exact stream at one and six slots, the
long clips, and one pass with `H264_DISABLE_SIMD=1` so the scalar branch profile
is not lost), and once with what it recorded. Measured against a `-O3` build of the
same sources, after the CABAC engine copy above: 1.055x at one slot on 720p, 1.067x
on 1080p and 1.054x on the interlaced clip, and 1.032x, 1.030x and 1.026x at ten
slots - medians of seven paired runs taken in both orders. Which is less than `-O3`
gave but of the same kind, since 47% of the time is branchy C syntax decoding. The
profiled build does 631 frames per second on one slot against FFmpeg's 793 (1.26x)
and 2641 at twelve slots (1.83x). It is opt-in because it needs the corpus present
and a second compile, and because a plain `make` after it would silently keep the
profiled objects. Nothing about the output moves: this pass reproduced all 27 streams
byte-identical to the `-O3` build at one and twelve slots. Two warnings are turned
off for the profiled pass only,
both artefacts of compiling with a profile rather than of the code:
`-Wmissing-profile` for the damaged-stream paths the corpus never takes, and
`-Wpsabi`, which the optimiser's clone of a static `__m256i`-returning quarter-pel
helper provokes even though every caller of that helper is in the same file.

`make lto` was for a while the flag that paid the most once a picture is shared out, and
it paid there for a reason the single-thread numbers hide: the code that hands work between
threads is spread over five files, and only a whole-program build can inline across
them. `h264_progress_await()` lives in `h264_thread.c` and is called from
`h264_mb.c` and `h264_direct.c` - 6585 calls per 720p frame, nearly all of them
returning on the first atomic read - while `h264_progress_report()` publishes a row
from `h264_slice.c` and `h264_deblock.c`, and `h264_filter_help()`, which a waiting
worker takes before it sleeps, is in the pool too. When that was written it measured
1.007x, 1.033x and 1.014x at one slot and 1.19x, 1.24x and 1.09x at twelve against a
`-O3` build of the same sources. **It buys nothing now.** The same paired sweep, both
orders, five reps at twelve slots, after the shared trace flag went per-thread (the
section below the curves): 1.014x, 1.005x, 1.011x, 0.994x and 0.965x at 1, 4, 6, 10 and
12 slots on 720p, and a mean of 1.01x on 1080p - inside the control band, which is the
same binary against itself at 1-2%. The reading is that cross-module inlining was
recovering *waiting*, and the biggest single cause of that waiting was a cache line
bouncing between the filter threads; take the cause away and the cure has nothing left to
do. The target stays (it is byte-identical on all 27 corpus streams at one, six and twelve
slots, and it is what the Linux leg exercises), but on this machine and in this code the
default `-O3` build is the one to ship. The control runs the same
binary against itself at twelve slots, where the two medians land 1-2% apart in both
directions - that is the floor these measurements are read against.
Alignment was tested with it and without: `-falign-functions=32 -falign-loops=32
-falign-jumps=16` alone measures 1.011x, 0.998x and 1.011x at one slot, and on top of
LTO it is within the control band again, so it stays out of the recipe.

The two build-time levers overlap, which is worth knowing before reaching for both.
Against the LTO build, the profile on top measures 1.017x, 1.024x and 1.031x at one
slot and 1.014x, 1.015x and 1.025x at twelve - far less than the 1.052x, 1.053x and
1.059x it gives the plain build at one slot, because most of what the profile buys is
the inlining LTO has already done. The pair is the best this tree has measured at twelve
slots: LTO's 1.21x to 1.24x, plus another 1.4% to 2.5% from the profile. Those four
builds are from before the filter-claim fix below, which costs under 1% on either side
of every one of these comparisons, and from before the shared trace flag went per-thread,
of every one of these comparisons, and from before the shared trace flag went per-thread,
which took LTO's half of that gain with it - the profile is the part of the combination
still worth measuring against the plain build, and none of these four ratios is a claim
about the tree as it now stands. Combining them also brings
eight `-Wmaybe-uninitialized` warnings, all of them in `example_decoder.c` and none in
the library, which is what the profile's branch decisions do to the compiler's
proofs; they are artefacts of the combination in the same kind as the two the profiled
pass already silences, and they are left visible rather than added to the recipe.

The reason LTO can ship a library at all is `-ffat-lto-objects`: every member of
`libh264dec.a` then carries its compiled machine code next to the GIMPLE, so a linker
that does not know GIMPLE still finds something to link - which is the objection this
file used to raise against `-flto`, and the reason it stayed off. The IL is then taken
back out before the archive is written, for two reasons. A whole-program result cannot
be put back into per-file objects, so an integrator that links `libh264dec.a` gets the
`-O3` code either way and the archive gains nothing by carrying IL; and IL is written
by one compiler for one compiler, which is not a promise a shipped library should make.
Leaving it in also charges every consumer link for the pass: measured on this tree, one
test program took 2m15s to link against the fat archive and 0.23s against the stripped
one, and the fat link printed the diagnostic below in a build that never asked for
`-flto`.

What is left is the whole-program binary, and it is a separate artifact -
`h264dec_example_lto`, copied off the example this target links through the archive -
because the tree's own `h264dec_example` has to keep meaning "the default build". Naming
it separately is not tidiness but necessity on Windows: `mingw32-make` cannot stat
`h264dec_example`, since only the `.exe` is on the filesystem, so it counts that target
as missing and re-links it from whatever the archive holds. The first time the gates were
run over an LTO build they silently tested the `-O3` binary, re-linked mid-suite from the
stripped archive, which is exactly the thing to watch for. And the link does go through
the archive rather than straight over `build/*.o`, for a reason only measurement found:
the objects-link build is 0.891x on 720p and 0.881x on 1080p at twelve slots, because the
whole-program pass splits into jobs differently the two ways and the split that comes out
of the archive is the better one - so the target links the ordinary way and keeps a copy.

One warning is turned off for the LTO pass, `-Wstringop-overflow`, and it is an
artefact of the whole-program view rather than of the code. With the tables in the
same program the link-time pass folds `h264_scan8[0]` to its value, 12, and then reads
the destination of `h264_fill_rectangle(sl->mvd_cache[0][12], 4, 4, 8, 0, 2)` as the
two-byte element at that slot instead of the 40-slot neighbour-cache row it sits in,
which produces "writing 2 bytes into a region of size 0" at offsets 26, 28, 30, 40 and
so on up to 78 "into destination object 'mvd_cache' of size 26", thirty times over the
two lists. Those offsets are the four 2-byte writes of each of the four rows at stride
8 - slots 12 to 15, 20 to 23, 28 to 31 and 36 to 39 - and 39 is the last slot the row
has, so the fill ends exactly on the end of the array. That is what the direct-mode
fill is for: the sixteen 4x4 slots of a macroblock are the 4 x 4 grid that starts at
slot 12 of an 8-wide neighbour cache. To check it rather than argue it, a probe build
puts eight bytes of poison after each of the five neighbour caches and arms and reads
them around every macroblock decode and every filter step, on all 27 corpus streams
and all six long clips at one slot; no poison byte moved. A `#pragma GCC diagnostic`
around the two calls does not help: GCC does not carry diagnostic state into the
link-time pass, and the same thirty warnings come back with the pragma in place, which
is also why the flag is on the recipe instead of a source annotation. Laundering the
destination through `uintptr_t`, which is the other thing worth trying, does not help
either - the compiler keeps the base object across the round trip.

Those same neighbour caches carried the one piece of undefined behaviour the
project had, and the sanitizer leg found it the first time it was run: packing a
motion vector as `((a) & 0xffff) | (((b) & 0xffff) << 16)` puts the vertical
component - negative in half of any real stream - into an `int` above 32767 and
then shifts it left by sixteen, which is undefined rather than the wrap every
target here performs. Thirteen call sites across `h264_cabac.c` and
`h264_direct.c`, 148 reports over the 30-stream corpus, plus one weighted
prediction offset that shifted a negative `offset` the same way. The packing now
goes through `uint32_t` and converts to `int32_t` once at the end, so the value
each caller stores is the same 32 bits it stored before, and the offset
multiplies by a power of two instead of shifting - the same number by definition,
and the compiler still emits a shift for it. This was not measured as a
performance change and is not one; it is gated as a non-change, 27/27 byte-exact
against FFmpeg and 27/27 identical to the pre-fix reference on the plain build,
the same with `H264_DISABLE_SIMD=1`, on the whole-program build, and on the
aarch64 build running under qemu, with the sanitizer leg reading
`asan_reports=0 ubsan_lines=0`.

Using the API
-------------

    h264_param_t p; memset(&p, 0, sizeof p);
    p.threads = 8;              /* 0/1 = single threaded, capped at 12 slots */
    p.nal_length_size = 0;      /* 0 = Annex-B start codes (default) */
    void *dec = h264_dec_open(&p);

    /* one call per access unit; drain between calls */
    h264_packet_t pkt = { data, len, pts, dts };
    h264_dec_send_packet(dec, &pkt);
    h264_picture_t pic;
    while (h264_dec_recv_frame(dec, &info, &pic) == H264_DEC_GOT_FRAME) {
        /* picture: pic.planes[0..2], pic.lines/widths/strides, pic.qp,
           pic.b_decode_error; field order: pic.interlaced, pic.top_field_first,
           pic.repeat_pict; SEI: pic.pic_struct, pic.cpb_removal_delay,
           pic.dpb_output_delay, pic.recovery_frame_cnt, pic.timecode_* */
        h264_dec_frame_unref(dec, &pic);
    }

    h264_dec_flush(dec, &info, &pic);   /* until it returns H264_DEC_END */
    h264_dec_close(dec);

`h264dec_example.c` is the reference caller and shows the Annex-B access unit
splitting that `h264_dec_send_packet` expects. Feeding a whole file in one call
is legal but makes the decoder hold every output picture until you drain it.
`pic.dec_frame` is an opaque handle; `h264_dec_frame_unref` is mandatory, and
buffers stay valid until then.

Threaded output is bit-identical to single threaded output; the loop filter
publishes finished rows and pictures wait for the reference rows they read.
That is measured rather than argued: `make selfcheck` writes a one-slot
reference and asks 0, 2, 3, 6, 8, 10 and 12 slots to reproduce it - 27 of 27
streams byte-identical at every count - and `tests/selfcheck.sh` pointed at a
reference
directory written by the Windows build says the same from Linux.

With `nal_length_size` set to 1, 2 or 4 the packets are read as length-prefixed
(AVCC / MP4 inline) NALs instead of Annex-B, so a decoder configuration record
size field is all that is needed to feed samples straight out of an MP4. The
packet still has to be one access unit.

What is supported
-----------------

* CAVLC and CABAC, all slice types, arbitrary slice counts and ordering,
  `slice_mbyte_limit`-style constrained lengths.
* Baseline, Main, High, High 10, High 4:2:2 and High 4:4:4 Predictive
  profiles: 8x8 transform, scaling matrices, constrained and unconstrained,
  plus the High 4:4:4 lossless mode (`qpprime_y_zero_transform_bypass`). The
  8-bit path is 4:2:0; the 10-bit path covers 4:0:0, 4:2:0, 4:2:2 and 4:4:4.
  It is selected from the SPS and carries int32 residual coefficients,
  16-bit sample planes and the same prediction/MC/deblocking arithmetic at the
  0..1023 clip; 4:4:4 chroma is reconstructed as two additional luma-like
  planes, 4:2:2 uses the full-height chroma geometry, and monochrome pictures
  carry one plane. The whole 10-bit matrix is bit-exact against FFmpeg; it
  currently uses mostly scalar 16-bit kernels, so it is slower than the 8-bit
  path at one slot (390 vs 653 fps on the 720p High 10 bench clip); integer-
  position chroma and the copy/average row already use an SSE2 pavgw path. The
  8-bit path and its SSE2/AVX2 kernels are untouched.
* Frame and field pictures: MBAFF macroblock pair framing and PAFF (separate
  field pictures), field-weighted prediction, Pic-Aff direct mode,
  `pic_order_cnt_type` 0/1/2, weighted prediction (implicit and explicit),
  B-pyramid, long-term reference frames and full MMCO 1-6, `max_num_reorder_frames`
  reordering with decode or display order output.
* SEI: unregistered user data (encoder build, which the reference workarounds
  depend on), picture timing (`pic_struct`, CPB removal and DPB output delays,
  the first clock timestamp as SMPTE timecode) and the recovery point of an
  open GOP. Their values end up on the picture (see `h264_picture_t`). Other
  payloads are walked and skipped; a message that claims more bytes than the
  NAL carries stops the walk instead of running past the buffer.
* Input framing: Annex-B start codes, or 1/2/4-byte length-prefixed NALs
  (AVCC / MP4 inline samples) via `param.nal_length_size`.
* Damaged input is survivable: an undecodable access unit sets
  `b_decode_error` on the affected picture, macroblocks that were never written
  are painted mid gray, and decoding continues.
* Not supported: 12-bit, 4:2:2 and 4:4:4 content, slice data partitions
  (NAL types 2-4), redundant pictures (discarded, as the reference does),
  and separate-colour-plane. A picture timing message that carries several
  clock timestamps reports only the first one.

Tests
-----

| command | what it proves |
| --- | --- |
| `tests/compare.sh` (`THREADS=n`, `DATA=dir`) | decode is byte-identical to FFmpeg for every stream in `testdata`; it reads the source pixel format from ffprobe and asks FFmpeg for the matching 4:2:0 / 4:2:2 / 4:4:4 10-bit (or gray10le for monochrome) output |
| `tests/gen_10bit.sh` (`OUT=`, `FORCE=`, `FRAMES=`) | writes the 10-bit corpus used by `make check10`: High 10 4:2:0 CABAC/CAVLC, B-frames with weighted prediction, MBAFF, PAFF and eight slices, plus 10-bit 4:2:2 / 4:4:4, monochrome 4:0:0, cropped pictures, transform bypass and 720p |
| `make check10` | generates that corpus and runs `compare.sh` at 1, 8 and 12 slots - 19 streams, all bit-exact |
| `tests/feature_matrix.sh` (`OUT=`, `BIN=`, `FRAMES=`) | encodes one short stream per profile/chroma format (Baseline/Main/High 8-bit 4:2:0, High 10, High 4:2:2, High 4:4:4 in 8- and 10-bit) and reports which ones this decoder accepts and whether the ones it accepts are bit-exact against FFmpeg; informational, so missing formats print UNSUPPORTED rather than failing the run |
| `tests/avcc.sh` | the same streams fed as length-prefixed NALs decode identically to Annex-B |
| `tests/dsp_check.c` | SSE2 qpel, full-pel copy/average, chroma, inverse-transform, deblocking and weighted-prediction kernels, and the AVX2 qpel kernel, all equal to the scalar ones for every block size, fraction and put/avg mode, coefficient blocks across the whole int16 range, edge filters over both edge directions and all four filter kinds, and the two weighted-prediction dispatchers over the full parameter domain - each section prints how many cases and pixels it compared and how many reached the vector path, and fails if it reached none; it also checks that the index the loop filter's whole-macroblock skip stops at still matches the alpha and tc0 tables, failing both if the constant has grown past the tables and if it has fallen short of them, and checks the chroma dispatcher's integer-position copy against the bilinear filter it replaces at 2, 4 and 8 wide in put and avg modes |
| `tests/sei_check.c` | SEI walker and readers: encoder identity, unknown types, truncated and flooded messages, picture timing delays, timecode, recovery point |
| `tests/meta.sh` (`THREADS=n`) | the exported `interlaced` / `top_field_first` / `repeat_pict` triple matches ffprobe for every picture |
| `tests/fuzz.sh` (`THREADS=n`, `SEEDS=...`) | truncated, bit-flipped, byte-dropped and byte-injected streams, plus a header mode that flips bits only in the first 24 bytes after each start code (SPS, PPS and slice headers), never crash or hang; when the binary is a sanitizer build the harness also fails on any `runtime error` or AddressSanitizer line |
| `tests/sanitizer.sh` (`DATA=`, `LONG=`, `FUZZ=`, `SEEDS=`) | the 27-stream corpus and the long clip, then the same corpus damaged by the five fuzz modes in both 8-bit and 10-bit, on a POSIX build with `-fsanitize=address,undefined`. Several MC kernels read past their block on purpose - the planes carry a margin and the edge buffer is oversized - so the property that has to hold is not "no over-read" but "no read outside the allocation". LeakSanitizer is on. A report from either sanitizer, or a nonzero exit, fails the run |
| `tests/soak.sh` (`LONG=`, `CHURN=`, `LONG_THREADS=`) | opens, drains and closes the decoder in one process for hundreds of rounds and fails if the working set keeps growing - the channel-switch case a single decode cannot show |
| `tests/claim_hammer.sh` (`RUNS=`, `SLOTS=`, `CLIPS=`, `TIMEOUT=`, `BIN=`) | runs the long clips at two slots with every run bounded, and fails if a run does not finish: a picture left on the filter list with every thread asleep is a hang, not a wrong byte, and two slots is the only pool shape where it can happen |
| `tests/selfcheck.sh` (`THREADS=`, `BIN=`, `DATA=`) | writes a one-slot reference of YUVs and demands that another slot count, another build, or another operating system reproduce it byte for byte |
| `tests/api_check.c` (`[stream] [threads]`) | the public API called the way integrators mis-call it: NULL and empty packets, out-of-order recv/flush, garbage in, pictures held and returned late - each must answer with a documented code, hand out distinct buffers, and decode identically to the release-at-once path |
| `tests/bench_vs_ffmpeg.sh` (`THREADS=`, `REPS=`, `PROCS=`, `CLIP=`) | throughput against FFmpeg with both sides discarding the pictures, and against this decoder run as N independent processes - the two reference points a threading claim needs. Reports, does not gate |
| `tests/stability.sh` (`make stability`, `THREADS=`, `REPS=`, `SEEDS=`, `LONG_MD5=`, `FF=`, `RSS=`) | stability judged against the reference rather than in the abstract: the same bytes out of every repeat and every thread count, ours by comparing the YUV and FFmpeg by its own `-f md5`, then the whole fuzz corpus through both decoders where the failure is a signal or a hang and an error report is not - with how many pictures each side still decodes out of the same damaged file, and peak RSS and user CPU per picture where `/usr/bin/time` exists |
| `tests/sweep_vs_ffmpeg.sh` (`make sweep`, `SLOTS=`, `FRAMES=`, `SENS_EVERY=`, `SWEEP=`) | encodes 29 configurations the curated corpus does not reach and compares each against the FFmpeg reference at one, eight and twelve slots. Two controls ride along: the reference is regenerated at twelve threads and has to match its one-thread self, and every fifth stream is also compared with the in-loop filter off on both sides, which is where hundreds of thousands of bytes move if the comparison is comparing anything at all |
| `tests/gen_streams.sh`, `tests/gen_bench.sh` | regenerate the streams in `testdata` (x264) and the long benchmark clips |

The header mutation mode is not cosmetic: uniform bit flips mostly stop at the
slice payload, while the first bytes are where the dimensions, chroma format,
field flags and macroblock type live. Putting that corpus through the
`-fsanitize=address,undefined` build found the defect classes the valid streams
never reach. PCM macroblocks in MBAFF and PAFF pictures applied the doubled field
stride twice in the copy that writes them, once in the 8-bit path and once in
the 10-bit one, so a corrupt header that turned a macroblock into I_PCM wrote
past the picture plane; both now use the destination pointers the main decode
path has already computed for the macroblock. The 10-bit motion compensation and
chroma filter shifted negative pixel offsets left, and the bit reader could
shift by 32 when a truncated read had nothing left to pad with - all three are
undefined behaviour that UBSAN reports rather than crashes, and the shifts now
use multiplication or a guard. Two dequantisation multiplies, the CAVLC level
path and the IDCT's luma and chroma DC scaling, could overflow int on the large
levels a damaged stream can carry; the product is now formed in unsigned
32-bit arithmetic, which keeps the wrap defined without widening the hot path. A field
picture may carry up to 32 references per list, but the per-frame POC table was
sized for the 16 of a frame picture, so a slice that declared more wrote past
it; the table is list-sized now, and a frame picture's default PPS reference
counts are capped at 16. A slice whose picture structure disagreed with the
first slice of its access unit could walk an MBAFF pair past the bottom row,
and a second field could pair with a frame allocated for a different geometry
or chroma format; both are rejected now, an SPS change in either of those
fields flushes the DPB, and a pooled frame whose chroma format changed is
reallocated. Crop offsets are summed in 64 bits, SEI payload lengths and the
reported frame rate are bounded so a damaged VUI cannot produce a zero
denominator, and the MBAFF reference/MVD field conversions multiply instead of
shifting negative values. The API grew
the matching edges: an out-of-range `nal_length_size` is refused at open, and a
flush with no output slot no longer pops a queued picture and loses the caller's
reference. The one-time CAVLC table build and the environment block are now
published through an acquire/release flag, so two threads may open decoders at
the same time; eight concurrent instances run clean under ThreadSanitizer.
With those fixed, 1,089 header-mutated variants, the 405-variant general
corpus and the 285-variant 10-bit corpus are clean under ASAN+UBSAN, and
ThreadSanitizer reports no race across the corpus at 2, 6 and 12 slots.

`H264_DISABLE_SIMD=1 sh tests/compare.sh` reruns the whole sweep on the scalar
kernels; both paths are bit-exact, so a CPU without SSE2 decodes the same
pictures. `tests/compare.sh`, `tests/fuzz.sh` and `tests/sweep_vs_ffmpeg.sh` take
the same `BIN=` hook `tests/selfcheck.sh` has always had, which is how the ARM build
gets run against the x86 output through qemu (the NEON section above) without needing
an ARM box, and how `h264dec_example_lto` gets gated against the reference.

The 27 streams in `testdata` cover: CAVLC/CABAC P and B with direct modes,
multi-slice, B-pyramid, open GOP, weighted prediction P and B, MBAFF, PAFF
(4 streams), 8x8 transform, custom scaling matrices, `constrained_intra_pred`,
`deblocking_filter_constraint`, odd coded sizes with cropping, slice-length
limits, lossless, a picture timing SEI carrying `pic_struct`, and a CBR HRD in
the VUI (whose delays the picture timing message reads back).

Beyond that corpus, `make sweep` builds 29 further streams on the fly (in the
ignored `testdata/sweep`) and takes every one of them byte for byte at one, eight
and twelve slots - 87 decodes against FFmpeg 9.0.2, all identical. The settings are
chosen for shapes the curated corpus does not have: High profile carrying the 8x8
transform with **CAVLC**, AU delimiters with re-sent parameter sets and a CBR HRD,
arbitrary slice boundaries from `sliced_threads`, `intra_refresh` with its recovery
point instead of an IDR, six references with four B frames and temporal direct,
eight and twelve slices in a 176x144 picture, every picture an IDR, scene cuts every
eight frames, chroma QP offset at ±6, `crf=5` with `subme=7` against `qp=51` and a
flat grey source that is nearly all P-skip, transform bypass with CAVLC, MBAFF with
bottom-field-first, PAFF with the 8x8 transform, 1080p coded to 1920x1088, and odd
sizes on both dimensions. Its controls both fired: the reference matched itself at
one and twelve threads on all 29, and on the fifth case the no-filter run sat
382,672 bytes away from the filtered one, on the tenth 1,383,716 bytes, on the
twentieth 318,045 and on the twenty-fifth 256,096 - while the case encoded with
`no-deblock=1` moved 0 bytes, which is the answer the control should give when the
bitstream really does switch the filter off.

`make stability` asks those questions of both decoders rather than of ours alone. The
property frame threading can break is that the output not depend on the schedule: the
corpus answers 27 streams × (two repeats and thread counts 2, 6 and 12) with no
mismatch, the Linux build the same at 1, 6 and 12, and the reference is
scheduling-invariant on the long clips too - FFmpeg 9.0.2 at 1, 2, 6 and 12 threads gives
one MD5 each for 720p (`bc8e1de60427`), the interlaced clip (`ae15989e9930`) and 1080p
(`746e956744cb`), and FFmpeg 8.1.1 under Linux reproduces the 720p and 1080p values, so
the two sides of the comparison are each deterministic before they are compared. On
damaged input both decoders take the whole fuzz corpus - 324 truncated, bit-flipped,
byte-dropped and junk-injected files - with no signal and no hang from either. FFmpeg
refuses to *open* two of them (exit 183, "Invalid data found") where we read them, find
nothing decodable and report `frames=0`: a difference in reporting, not in resilience, and
the harness now classifies it that way, because an earlier version treated every exit
above 128 as a signal and reported those two as crashes. Over the 24 variants checked
picture by picture the two agree exactly - 544 pictures each, the same count on every one -
so our error handling is not quietly dropping frames the reference keeps.

Those MD5s are not merely invariant, they are the *same value* on both sides: this
build's YUV for the three long clips hashes to `bc8e1de60427…`, `ae15989e9930…` and
`746e956744cb…` - 2,073,600,000 + 1,382,400,000 + 3,110,400,000 = 6.57 GB over 3500
pictures, interlaced included - and that is what FFmpeg's own `-f md5` answers for the
same three files at every thread count tried. The bit-exactness claim therefore covers
6.5 GB of real 720p and 1080p output, not only the 30-frame corpus, and ours at twelve
slots is byte-identical to ours at one on the 1.38 GB interlaced clip, which is the case
threading could actually break.

Memory and CPU on the Linux leg, one thread, medians of three alternating runs: peak RSS
28.2 MB against the reference's 27.7 MB at 720p (1.02x) and 46.4 MB against 42.3 MB at
1080p (1.10x), so the picture pool is the same order as FFmpeg's DPB, as it has to be.
User CPU per picture is where the distance still sits: 1.67 ms against 1.21 ms at 720p
(1.38x) and 3.67 ms against 2.44 ms at 1080p (1.50x) on that toolchain (gcc 11.4 `-O3`
against a static FFmpeg 8.1.1), where the same quantity on the shipped Windows build is
1.23x at one thread (gcc 16.2 against FFmpeg 9.0.2). Those are quoted separately because
they are not the same compiler building the same reference.

Throughput on this machine (8 physical cores, 16 logical, GCC 16.2). Absolute
numbers move with whatever else is running, so two runs of the same binary are
quoted: the idle figures, and the same build re-measured later while a loaded
desktop held ~7 of 16 logical cores. Best of three / best of six respectively.
The table is from the v0.14 build, before the SSE2 deblocking kernels; the
paired measurement of those is below.

| stream | 1 slot, idle | 6 slots, idle | 1 slot, loaded | 6 slots, loaded |
| --- | --- | --- | --- | --- |
| 1280x720 High, 300 frames | 381 fps | 1148 fps | 239 fps | 391 fps |
| 1920x1080 High, 200 frames | 173 fps | 523 fps | 107 fps | 175 fps |

The 1-slot to 6-slot gain was 3.0x idle and 1.6x loaded: frame threading wants
idle memory bandwidth, so the loaded column understates it. What stays
comparable under load is a build-to-build measurement taken back to back, and
those are the numbers below. That table's slot counts are the ones the old
worker pool could actually use; the ceiling moved when the lock did, and the
scaling curve of the current build is further down.

The SSE2 4x4 and DC transform kernels are bit-exact over the whole int16 input
range and worth nothing here - 239 fps before and after, inside the noise. The
call counts say why: the 300-frame 720p stream runs only ~3.5k inverse
transforms per frame, because most macroblocks carry almost no residual.
SSE2 motion compensation is worth 232 fps against 207 fps scalar, 12%.

The deblocking filter was the remaining kernel with room, and it is now
vectorised. Splitting the cost first, with the pixel filters stubbed out but
the bS and filter caches still filled: 354 fps with the filter, 446 with only
its arithmetic removed, 483 with deblocking off - so the pixel filter is 20.7%
of decode time and the cache work that stays scalar is 6.0%. Measured against
the build without those kernels, nine paired runs on the long clips, one slot:
**1.067x at 720p and 1.048x at 1080p** (medians). Kernel by kernel, from
`build/dsp_bench`: 2.29x for the inter luma edge and 2.58x for the intra one
when the positions are strided, 1.5x when they are transposed, and for chroma
1.65x and 2.15x strided but 0.98x and 0.78x transposed - a chroma edge is only
eight positions wide, so the transpose costs more than it saves and those two
layouts stay scalar. The 20.7% and 6.0% were measured on the 300-frame clip with
the scalar filter; the same stub on the shipped build puts the filter at 9.4% and
the cache work at 10.4% (table below), which is the right direction but not a
ratio - different clip, and a stub also removes the cache traffic that goes with
the filter - so the honest statement of the kernels' own speed is the `dsp_bench`
numbers above.

A sampling profile of one slot (`perf record -e cpu-clock` on a Linux build of the
same sources) attributes 4.4% of the decode to `lf_group_masks`, the helper that
turns the four `tc0` group values into the eight-lane clip and enable vectors, and
the reason is visible in the object: `(pos0 + i) / inner_iters` per lane leaves
eight `idiv` in each of the two inter kernels. Replacing the division with a shift
(`inner_iters` is 1, 2 or 4) is worth what the kernel bench can see - 1.09x on the
inter luma edge and 1.07x on the inter chroma one, and, as a control, nothing on
the intra kernels, which do not call it. It is worth nothing the fps bench can
see: nine paired runs at one slot give 0.996x, 1.001x and 1.003x on the three
long clips and seven at ten slots 0.991x, 1.003x and 0.984x, and the output is
byte-identical either way. So the change was reverted rather than shipped, and the
4.4% is not the divider: it goes with building the two vectors through a stack
array - eight scalar stores and a 16-byte load that cannot be forwarded - and with
wherever the sampling interrupt happened to land, not with work an out-of-order
core was waiting on.

The same profile's next share, 2.2% in `memcpy`, went the same way. The only
copies in the decoder whose width is not a compile-time constant are the rows of
`h264_emulated_edge_mc` - a luma block with its halo is 21 rows of up to 21 bytes -
so those rows were copied with SSE2, 16/8/4-byte moves whose final store overlaps
the previous one instead of finishing byte by byte. End to end: 0.996x, 0.999x and
1.000x at one slot on the three long clips, output byte-identical. The reason is in
the counters: rdtsc around the function on the 60s clip gives 117,669 calls, 1.53M
rows and 26.2M cycles, and one slot's 2462 ms at this machine's 3.6 GHz time-stamp
counter is 8.9 Gcycles - so edge emulation is 0.30% of the decode, and no kernel
inside it can be worth a percent. Interposing `memcpy` in the Linux build finds the
library makes no `memcpy` calls at all, because the 4-, 8- and 16-byte copies in the
neighbour-cache fill are inline moves there too: the 2.2% was where the samples
landed, not work. Two of the three largest non-CABAC shares the sampler reported
have now failed to convert; the stub-out numbers in the stage table further down are
the ones that have predicted correctly, so those are what the effort follows.

Motion compensation then needed a histogram rather than a wider kernel.
Counting the luma MC calls of the same 300-frame 720p stream by width and by
whether the position is fractional gives 1.43M calls at width 16, of which
**1.36M are at an exact pixel position** - integer motion is not the rare case,
it is nearly all of it - and the interpolation kernels are gated on a fractional
position, so all of those were running the scalar byte loop. The copy/average
kernel that replaces them (`h264_hpel_mc_sse2`; pavgusb is exactly the
reference's `(a+b+1)>>1`) is worth 1.068x at 720p and 1.039x at 1080p, median of
nine paired runs.

The AVX2 16-lane interpolation kernel measures 2.0x the SSE2 one overall in
`build/dsp_bench` (2.15x at 16x16, 2.14x at 16x8, 1.60x at 16x4) and nothing
beyond noise end to end: 1.008x at 720p and 1.006x at 1080p, median of the same
nine pairs. Fractional luma MC is only 62k
of those 1.43M calls, so widening a filter that is already vectorised reaches a
slice too thin to see. It is bit-exact against the scalar path and costs one
dispatch branch, so it is kept - with the number stated rather than implied.

Chroma has the same story in the other direction. Its width histogram says
14.9% of the 16.8M chroma MC calls of the 720p clip are 4-wide (10.0% of 12.0M
on the interlaced one), and the vector kernel's eight lanes with a masked store
can serve them - `EDGE_PAD` is 32, so the over-read is covered. Routing every
one of them there measured 1.000x at 720p and 1.0045x on the interlaced clip,
medians of seven and five paired runs: a 4x4 block is bound by the load and the
store either way. The dispatcher therefore keeps `w >= 8` and the narrow widths
stay in the scalar loop, now with the measurement behind the choice instead of a
guess.

Chroma then had the copy problem, not the width problem. Counting the same calls
by *position* instead of width says 13.99M of the 15.57M chroma MC calls of the
60s clip have **neither axis moving**, which is 93.4% of the 937M chroma pixels
those calls touch; the interlaced and the 1080p clip are the same shape (92.9%
and 89.4% of calls, 95.0% and 93.1% of their pixels). A chroma block only moves
when its luma motion vector is not a whole number of chroma samples, and most
motion vectors are. At `(0, 0)` the bilinear weights are 64, 0, 0, 0, so the
kernel's sum is `(64*p + 32) >> 6 = p`: it was computing a copy with four
multiplies and eight 16-bit unpacks per eight samples, on blocks that are 92% of
the calls and 8x8 in size. The dispatcher now hands those calls to the same byte
copy/average kernel the luma path uses, for `w >= 4` - the SSE2 narrow-row path
stores four bytes at a time, so a 2-wide chroma block (a 4x4 luma partition)
would also write its right neighbour and keeps the scalar loop. Worth 1.055x,
1.074x and 1.064x at one slot on the three clips, and 1.033x, 1.026x and 1.056x
at twelve: medians of five paired runs with the base measured twice in each run
as the control, which stayed between 0.994 and 1.009.
`tests/dsp_check.c` compares the dispatcher against the filter it replaces at 2,
4 and 8 wide in both put and avg modes, and both ways of getting it wrong turn it
red in exactly the places they should - a copy that ignores `avg` fails the five
vector-width cases, an average that drops the `+1` fails the 2-wide ones. The
sanitizer leg added with this change runs all 30 streams under
`-fsanitize=address` with no report: the copy reads eight bytes per row where the
filter read nine and a row more, so it touches strictly less than it replaced.
The sampler agrees with the arithmetic - `h264_chroma_mc_sse2` was 8.3% of a
one-slot 720p decode by physical function and is no longer in the table, while
the copy kernel it now shares with luma goes from 3.3% to 5.4%.

How those are measured matters here: a 12-second clip decodes in about a second,
and one-second pairs of two builds that differ by 5% produced ratios anywhere
between 0.78 and 1.17 on this desktop. `tests/gen_bench.sh` also builds 60s and
40s clips (`long_720p.h264`, `long_1080p.h264`), which decode in 4-6 seconds, and
on those the same pairs land within a few percent of each other every time. The
three ratios above (deblocking, full-pel, AVX2) are medians of nine paired runs
on the long clips; the transform and MC numbers in the earlier paragraphs are
older best-of-three measurements on the short ones, quoted as they were taken.

The slot ceiling was wrong, and the reason is a lock. An instrumented scratch
build - the same sources with rdtsc counters around the owner's
`process_access_unit` interval, the owner's waits for a free job slot, the wall
each worker spends inside a picture, and four costs inside it: acquiring the
decoder mutex to test a reference row, the wait once that test fails, publishing
a row, and the commit chain - said this. For the 60s 720p clip at six slots the
owner's decode-order work is 0.29 Gcyc and the commit chain 0.98, against the
11.2 Gcyc of wall the same clip takes on one slot: 11% serial, which by Amdahl
allows a 4.1x speedup on six workers. It measured 2.56x, and the missing part was
not in front of the workers but inside them. `h264_progress_await` took the
decoder lock before every reference-row test - 6585 tests per frame of 720p, 9.9M
per clip - and 12.7 of the 32.9 Gcycles the workers spent inside a picture,
38.5%, went to acquiring a lock that was almost always taken for nothing. The
waiting it exists for was 1.5 Gcyc, 4.6%. Almost all of that 0.98 Gcyc commit
chain was the same mutex: a finished picture waiting its turn in decode order
could not get the lock either.

The published row count is now read with an acquire load and no lock, and a
report stores it under a mutex before broadcasting, so the re-check inside the
waiting path still cannot miss a publication that lands between the two tests.
That second mutex was the decoder's, and is now the picture's own - see the
per-picture lock below.
Of the 9.9M checks, 12k now take the lock: the ones that really wait. Worker time
in lock acquisition went from 12.7 Gcyc to 0.04 and the commit chain from 0.98 to
0.04, so the serial part of decode is now 0.33 Gcyc in 11.2; the same 1500 frames
that took 32.9 Gcyc of summed worker wall take 20.7. Back to back against the
previous build that is 1.246x at six slots and 1.233x at 1080p (medians of seven
paired runs), 1.086x at four, 1.060x at two, and nothing at one slot, where the
early return means the path is not used at all. Output is unchanged: 27 of 27
bit-exact against the reference at one slot, 27 of 27 bit-exact again at ten,
and 27 of 27 still bit-exact at twelve.

Eight slots used to be slower than six, and that was a lock rather than the
picture pool - it took a second look to find which one. A row-progress report and
the awaits that miss it both queue on the decoder's single mutex, and every
report broadcasts to the whole pool, so at ten slots the probe measured 13% of
all worker wall spent inside `h264_progress_report()` with another 15% spent in
the await waking up to re-check a row of a picture it is not decoding. FFmpeg's
`ff_thread_report_progress()` takes the mutex of the thread that owns the picture
being published instead of a global one, so each `H264Frame` now carries its own
mutex and cond for `progress[]` and the lock-free fast path - the atomic read,
9.9M times for a 1500-frame clip - is untouched. Paired medians of five runs in
both orders against the previous build: 1.004x at two slots, where there is
nothing to contend yet, 1.085x at six, 1.134x at ten, 1.194x at twelve. Gating
only the broadcast on a count of real waiters, which is the cheaper-looking
change and was tried first, measures 0.88x at ten slots: what costs is the mutex,
not the wake.

Then the filter moved off the decode thread. A row becomes filterable one MB row
behind the decoder, so with the split the decode pass only announces how far
reconstruction has got (`H264Frame.recon[]`) and the rows go on a queue that any
thread takes from instead of sleeping: the consumer waiting for a reference row,
the worker with no job, and the decode pass waiting for its own picture to be
finished. One picture is filtered by one thread at a time, because the filter
caches carry the previous macroblock from row to row - which is why those caches,
and a claim flag, now live in the picture. Measured against the build with the
lock split but the filter still inline: 1.102x at six slots, 1.072x at ten,
1.064x at twelve, 1.067x on the interlaced clip and 1.067x on 1080p, and 1.003x
at one slot, where nothing is split. The schedule a row is filtered on is
unchanged, which is what makes the output land byte for byte where it did.

That queue is also where the first real hang in this decoder came from, and the shape
of the bug is the shape of the queue. The decode pass that finishes its last row waits
for its own picture's chain; if another thread holds the claim at that moment there is
nothing for it to take, and it used to go to sleep on the picture's own condition. A
thread that puts a claim *down* without finishing the chain - because the rows it could
reach are done - re-queues the picture and broadcasts on nothing. So the owner sleeps,
the picture sits on the list, and at two slots nobody is left awake who has to take it:
the other worker is parked at the commit barrier waiting for this job to commit, and the
decoder owner is parked in `h264_job_acquire()` waiting for a free slot. The run prints
nothing and uses no CPU. On the `-O3` build it came up on 2 runs of the 1500-frame 720p
clip out of 40 at two slots, and 1 out of 15 on the whole-program build, which is how it
survived every gate in this file: `selfcheck` does run two slots, but over ~100-frame
streams, and both soak lines run at ten and twelve slots where there is always an awake
helper. The fix makes the wait mean what the queue means: sleep on the filter list's own
condition, under the list lock, and only while another thread actually holds the claim -
which is the one event the release now broadcasts, under that same lock, so it cannot be
missed. When the claim is free the owner takes the chain itself, which is what the
header comment for the list has always promised. Nothing about the output moves: 27 of
27 byte-identical at one, two, three, six and twelve slots against the pre-fix build,
and `tests/claim_hammer.sh` - the long clips at two slots, every run bounded, because
the failure is a run that never ends - is part of `make check`, since no other line in
the suite reaches that pool shape.

The same queue handed out a second bug of its own family, and this one was found by
asking for stability rather than by waiting for the crash it caused.
`h264_alloc_frame()` now reports a state that must be unreachable: a picture whose last
reference has gone while the filter queue still holds it. With the report in place and
before the fix it fired **twice in the first twenty round-decodes** of the re-open soak
run four processes wide at twelve slots, always as `queued=0 claimed=1 done=1` - one
recycled picture in ten still held a claim nobody had put down, which is why the
segfault itself, at twice in about 450 round-decodes, was never going to be caught by
running the suite more often. `filter_finish()` published `filter_done` *before* it
lowered the claim, and `h264_filter_wait_done()` returns the moment it sees `done`: the
owner can hand the picture out, drop its last reference and recycle it - resetting
`done` back to 0 - while the helper is still inside `filter_finish()`. Its next
statement is the re-queue test, `if (!f->filter_done) filter_push_locked()`, and against
a freshly reset picture that reads true, so a picture being decoded went back on the
shared list where a second helper could take it, and both ran the loop filter through
the one `f->filter_sl` each picture carries. Two threads writing one slice context is
the segfault; the picture being filtered by the wrong thread is a corruption the
bit-exactness gate cannot see, because it needs the second take to land inside a window
of a few hundred instructions.

`filter_finish()` now takes the list lock first, lowers the claim, publishes `done`
inside that same critical section, and broadcasts the picture's own condition after
releasing the lock, so no thread can ever observe a finished chain with an unreleased
claim; and the recycle resets the four filter flags under that same lock, since that is
whose they are. After the change nothing fires in 320 round-decodes at twelve slots four
processes wide, nor in 48 at two and six slots on the interlaced clip, and the paired
A/B against the pre-fix build says the ordering costs nothing: 0.985, 1.028 and 0.990 at
one slot and 0.992, 0.978 and 0.991 at twelve, against a control band of 0.977 to 1.014.
The report stays compiled in, and `tests/fuzz.sh` already fails a run that prints it.
ThreadSanitizer's list for a threaded run goes from eight races to seven, which is what
says the class is closed rather than the symptom silenced.

The seventh was the filter help reading the worker's decode clone at the same time as
`h264_job_submit()` bound the next job into it: six write/read pairs, all on the
`mb_width`/`mb_height`/`mb_stride`/`b_stride` geometry. The fix is a second decoder copy
per worker rather than a lock on the filter path. `filter_clone` is a snapshot of the
geometry, the read-only block-offset tables and the parameter block of the last committed
job, and only the filter help reads it, so binding the next job never touches the state a
running pass uses. The tables behind that snapshot are the one thing a dimension or
pixel-format change replaces, so `alloc_mb_tables()` freezes the helpers, waits for the
passes already in flight, invalidates every snapshot and only then frees the old tables.
Freezing is what makes the drain cheap: an idle worker that is left free to try an empty
filter pass wakes the pool each time, and a drain without the freeze was measured
spending 657 ms of a 1.0 s 720p decode in wakeups. With it, twelve slots measure 1.00x
against the pre-snapshot pooled build on 720p and 1.015x on 1080p, and ThreadSanitizer
reports no warnings on the corpus at 2, 6 and 12 slots (with `setarch $(uname -m) -R`;
without it this WSL build dies with `unexpected memory mapping`). The four reports that
remained after the snapshot were the CAVLC trace coordinates: two workers lazily cached
`H264_TRACE_X`/`H264_TRACE_Y` in plain statics on their first picture. That trace has
since been removed from the tree altogether.

A picture's allocations were the other half of the churn. Each `H264Slice` is 34 KiB, and
the old path gave every slice its own RBSP copy and its own MC edge and bipred scratch -
about 315 KiB at 720p and 460 KiB at 1080p - so a one-slice-per-picture stream did four
allocations and four frees per picture: 6,000 of each for the 1,500-frame 720p clip.
Slice objects now come from a per-decoder free list capped at 64, with the RBSP retained
up to 128 KiB and an oversized one dropped on return; the MC scratch belongs to each
decoder and worker copy and grows only when the resolution does; and the parameter-set,
first-mb and SEI RBSP peeks share one grow-only buffer on the decoder. The frame pool was
already a lazy 36-entry pool that reallocates only when the geometry or pixel format
changes. Threads are created once in `h264_dec_open()` and joined in `h264_dec_close()`;
no picture or slice creates a thread.

That deferred pass is also where most of the remaining distance to the reference sits,
and finding that out changed what got optimised next. Comparing one thread against one
thread on three 1080p shapes - residuals with motion, every frame an I frame, and a
black frame so that nearly every macroblock is P-skip - puts the deficit in the cheap
case: 1.24x behind on the first, 1.13x on the second, **1.77x on the third**; in the
same three windows after the skip described below, 1.24x, 1.10x and 1.42x. Turning
the loop filter off on both sides says why: that pass costs this decoder 23.6% of a
frame on the skip-dominated clip and costs the reference 15.3%, because the reference
declines to filter macroblocks it can prove will not change - its
`fill_filter_caches()` returns before filling anything once the QP is low enough -
while this decoder filled the caches, walked the edges and found nothing to do every
time.

The decline can be made on proof rather than on a guess. A luma edge clips its delta to
+-`tc0[index][bS]` and a chroma edge only runs at all when `alpha[index_c]` is nonzero,
and in these tables alpha is zero through index 15 while tc0 is zero for all four bS
through 16, so below that no sample on any edge of the macroblock can move.
`filter_mb_inert()` asks that of the macroblock and of the neighbours whose edges it
owns, ordered so the usual answer costs one load, one add and one compare - in a normal
stream one QP settles almost every macroblock - and `fill_filter_caches()` then returns
before touching the mv, ref and nnz caches, which is the part that was being paid for
nothing. `LF_FILTER_INERT_INDEX` is where that run ends, and `dsp_check` verifies it
against the tables in both directions: too large fails, and so does a constant that has
stopped being the tightest safe one, which is how a table edit cannot quietly make the
skip wrong.

Two things it is not. It is not applied to MBAFF pictures: there the vertical edges of a
frame-coded pair also take a QP from the partner field's macroblock, which is not one of
the four this looks at - measured, not assumed, because the first version did cover
MBAFF and changed 6 and 10 bytes of the corpus's two MBAFF streams and nothing else,
which is the negative control that says the check discriminates. And it is not FFmpeg's
rule: that compares the *average* of the two QPs against a hardcoded 15 with the chroma
offsets folded in, so it skips some macroblocks this one keeps - QP 0 next to QP 30
averages to 15 - and this is the stricter, provable form of the same idea.

Against the build without it, order-alternating medians of eleven runs on the
whole-program build: **1.257x** on the flat clip and 1.056x on the all-intra one at one
slot, 0.998x where there are real residuals; at twelve slots 1.056x on the flat clip
and 1.017x, 1.021x on the two long bench clips. On `-O3` those two clips are neutral at
one slot (0.999x, 0.996x), which is what the counters predict: the skip fires on 100%
of the flat clip's macroblocks, 14.8% of the mixed 1080p one, and 6.2% and 3.0% of the
two bench clips. Rate-controlled content at ordinary bitrates therefore pays one
compare per macroblock and gains little; flat and dark material gains a quarter of the
decode, and these synthetic clips - which is what `tests/gen_bench.sh` writes, and why
the third shape had to be encoded separately - contain almost none of it.
`tests/gen_bench.sh` writes all three shapes, since the shape comparison is what found
this.

The same "prove it inert" idea has a second, much more common case. A 16x16 inter
macroblock with zero luma CBP only ever filters its left and top edges: `mask_edge`
is 3, so `filter_mb_dir` sets `edges` to 1, and its `mask_par0` shortcut derives all
four lanes of an edge from the one corner 4x4 block. If that block's reference and
motion vector match the neighbour's adjacent block, and the luma nnz on the edge is
zero, both edges - and the chroma edges, which reuse the luma bS - have boundary
strength zero; the filter caches would be filled and both directions walked for
nothing. `filter_mb_uniform_mv()` proves that from the frame metadata before the
caches are built, for the neighbour partition modes the shortcut covers. It stays
out of MBAFF and field pictures, where the field pairing changes which QP and block
an edge addresses, and out of CAVLC 8x8DCT neighbours, whose nnz cache is
synthesized from the CBP high bits rather than read from `f->nnz`. Against the build
without it, medians of nine and seven paired runs with the pictures discarded:
**1.02-1.03x at 720p and 1.02-1.05x at 1080p on one slot**, 1.00-1.01x at
twelve, and 1.000x on the interlaced clip. The profile agrees about where the
saving lands: `fill_filter_caches_inter` falls from 3.25% to 1.44% of one-slot
samples. Output is
unchanged: 27 of 27 streams bit-exact at one, eight and twelve slots, 19 of 19 for
the 10-bit corpus at the same three counts, and the 324-variant fuzz corpus still
runs with no signal and no hang.

The luma deblock kernels also spent time building their per-lane `tc` and enable
vectors through an eight-entry stack array. For the common 16-position inter edge
(`inner_iters == 4`) the two groups are simply two broadcasts and one 64-bit
unpack, and a lane whose `tc0` is -1 still has to be disabled - that is the bS=0
lane. `lf_luma_masks()` now builds those vectors directly, and `lf_lanes_used()`
for the intra filters uses an all-ones or low-four-lanes constant instead of the
same stack array. A chroma equivalent was tried on its own and measured neutral
inside the noise, so it was not kept then; it returned with the horizontal chroma
kernel below and now serves both chroma directions. Against the build without the
luma change, paired one-slot runs land around 1.01-1.02x at 720p and 1.00-1.02x at
1080p; `dsp_check` still passes all of its kernel cases and output is
byte-identical on the full corpus.

The luma filter now has an AVX2 kernel for the layout where the sixteen positions
of an edge are consecutive bytes (`ystride == 1`): every tap is one 16-byte load
widened to int16 lanes, so the whole edge is filtered in one vector instead of two
passes of eight. The transposed direction stays on SSE2, where the 8x8 byte
transpose already costs more than the arithmetic it feeds and a wider vector does
not remove any of it. The gain is small - paired medians of nine runs put it at
1.003x on 720p and 1.001x on 1080p at one slot - and the profile agrees: the SSE2
and AVX2 luma kernels together fall from 4.54% to 4.40% of one-slot samples.
`dsp_check` now compares the AVX2 kernel lane by lane against the scalar filter in
the same sweep as the SSE2 one.

The horizontal chroma direction is the larger of the two remaining pixel costs, and
it used to stay scalar: its positions are a row apart, so the vector kernel needed
an 8x8 transpose that cost more than the filter. But a chroma edge only reads four
taps and rewrites two, so a 4x8 byte transpose built from byte and word unpacks is
enough, and it filters all eight positions at once; the eight rewritten `p0`/`q0`
pairs are adjacent in memory, so they go back as eight 16-bit stores rather than a
second transpose. `filter_mb_edgecv` falls from 4.34% to 0.84% of one-slot samples,
and the chroma kernel as a whole (both directions) from 6.25% to 3.79%. Measured
against the build before the AVX2 kernel, medians of nine paired runs with the
pictures discarded: **1.026x at 720p and 1.023x at 1080p on one slot**, and 1.014x
at twelve on 720p. The eight-position mask builder serves the vertical direction
too, worth another 1.007x on both clips. Output is 27 of 27 bit-exact at one, eight
and twelve slots, 19 of 19 for the 10-bit corpus (whose chroma paths are separate
16-bit functions), and the fuzz corpus is clean.

Every macroblock used to rebuild the 48 block offsets that its intra prediction and
IDCT calls index - 4x4 positions inside the macroblock scaled by the luma and chroma
linesizes. Those offsets depend only on `h264_scan8` and the picture's strides, so
`alloc_frame_buffers` builds them once for frame-coded macroblocks and once for
field-coded ones, and `h264_hl_decode_mb` picks one of the two with `MB_FIELD`.
`h264_hl_decode_mb` falls from 3.21% to 1.54% of one-slot samples, and paired medians
of nine runs put the change at **1.022x on 720p and 1.017x on 1080p** at one slot.
The full corpus, including the MBAFF and PAFF streams that take the doubled-stride
variant, stays byte-exact at one, eight and twelve slots.

Taken together, the changes since the 10-bit merge - the two deblocking skips, the
luma mask builder, the AVX2 luma kernel, the chroma transpose, the per-frame
block offsets and the two CABAC engine changes - measure **1.096x at 720p and
1.100x at 1080p on one slot**, 1.071x on the interlaced clip, and 1.027x and
1.018x at twelve slots on the two progressive clips, all medians of alternating
runs against the pre-optimization build. In the same window the reference is
1.19x and 1.23x ahead at one slot.

The curve on the default `-O3` build and on `make lto` (neither with `make pgo`),
with the pictures discarded rather than written out - `h264dec_example in.h264 - 12`,
see the sink note below - medians of five runs each, the two binaries taken alternately
in the same window:

| slots | 1 | 2 | 4 | 6 | 8 | 10 | 12 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1280x720, 1500 frames, `-O3` | 661 | 1060 | 1689 | 2465 | 2935 | 3298 | 3395 |
| 1280x720, `make lto` | 669 | 1078 | 1728 | 2469 | 2987 | 3260 | 3334 |
| 1920x1080, 1000 frames, `-O3` | 313 | 508 | 839 | 1128 | 1358 | 1508 | 1586 |
| 1920x1080, `make lto` | 306 | 517 | 838 | 1166 | 1376 | 1507 | 1561 |

That is 5.14x and 5.07x from one slot on the `-O3` build, where before the lock was
split it was 3.5x and 3.4x, after the lock alone 4.0x and 4.1x, and 4.29x and 4.07x up to
the trace-flag change below. The whole-program build no longer adds to it: paired through
the same window it gives 1.014x, 1.005x, 1.011x, 0.994x and 0.965x at 1, 4, 6, 10 and 12
slots on 720p and a mean of 1.01x on 1080p, inside the 1-2% band the same binary shows
against itself. 720p gains 3% between ten and twelve slots, so 8 to 12 is what to ask for
and the cap stays `H264_MAX_JOB_SLOTS 12`. None of it changes a byte
of output, measured again in full on this build: `make check` is 27/27 bit-exact
against FFmpeg and `tests/compare.sh` also repeats all 27 against the reference at
six and twelve slots, so it is the threaded paths that are covered and not only the
serial one; the same 27 are byte-identical with `H264_DISABLE_SIMD=1`, which is the
scalar path; `make selfcheck` reproduces the one-slot reference
at 0, 2, 3, 6, 8, 10 and 12, the 1.38 GB YUV of the interlaced clip is byte-stable
across repeat runs at six and twelve slots, a Linux build of the same sources
reproduces the Windows YUVs at 1 and 8 threads, `sh tests/sanitizer.sh` runs the
whole corpus and the three long clips through that Linux build with
`-fsanitize=address` and reports no report and no leak, `make stability` puts the same
questions to the reference as well - 6.5 GB of long-clip output identical between the two
and 324 damaged variants with no signal and no hang from either side - and `make fuzz` reports no problem
over 324 damaged variants at one and ten slots. Because the filter queue changes
what happens at close, the open/close soak runs five rounds of the 1500-frame clip at
four slots with sixty rounds of a short one at one, then five rounds at ten slots, which
is where a lost wakeup in that machinery shows up as a hang rather than as a wrong byte.
That is now also said directly: `tests/claim_hammer.sh` runs the two long clips that
reach the stranded-claim shape twenty times each at two slots, every run bounded because
the failure it looks for is a run that never ends, and no other line of the suite covers
that pool shape.

A shared debug flag was worth a fifth of the twelve-slot throughput, and it was worth it
for a reason no single-thread measurement can show. `h264_deblock.c` kept its trace gate
in a file-scope `static int lf_trace`, assigned once per macroblock of the deferred filter
- and since the filter was split off the decode thread, that filter runs on every worker in
the pool. Nothing synchronises the write, and nothing needs to: only the thread that wrote
it ever reads it back. But it is one cache line that twelve cores take exclusive
ownership of, once per macroblock, 5.6M times over the 60s clip - invisible at one slot and
a throughput tax at twelve. `lf_seq` and the MB-dump counter in `h264_mb.c` join it, so all
three are now thread-local (with a plain-static fallback where a compiler has no thread
keyword: only the trace line numbering degrades, never the decoded bits), and the
environment block is filled once at `h264_dec_open()` behind an atomic compare-exchange
instead of lazily inside an out-of-line accessor. That accessor also sat on the row-report
path, behind `if (!h->threaded) return;` - so it was a cost paid only by the threads, and
the sample profile never showed it because it is not where the time goes at one slot.
The two edits are separable and both windows agree on the split: keeping the inline
accessor but putting the trace state back in one shared word leaves 1.064x and 1.106x at
twelve slots where the pair gives 1.188x and 1.276x, so the bouncing line was the cost and
the removed call is worth somewhere between nothing and 4% - the edge of what this desktop
resolves. ThreadSanitizer's list loses its `h264_deblock.c:970` entry with the change,
which is the same fact stated the other way. Nothing about the output moves: the three long
clips are byte-identical to FFmpeg and to their own one-slot run at twelve slots
(2.07 + 1.38 + 3.11 GB), and eleven trace-gate combinations produce the same lines the
previous build does apart from the timing summary and the heap addresses the PS trace
prints.

Against the reference itself, same machine, same clip, the two binaries run
alternately in the same minute - `make bench`, medians of five paired runs. The
top pair is what the two decoders cost, the bottom pair what the two *programs*
cost, because the second half writes the 2 GB of YUV out and the first throws it
away. Each table below is one window: ours and the reference are paired inside it,
and the two windows are a few fps apart on the same build, which is what the
desktop here costs between minutes:

| slots / threads | 1 | 6 | 8 | 10 | 12 |
| --- | --- | --- | --- | --- | --- |
| ours, pictures discarded | 660 fps | 2505 | 2983 | 3237 | 3343 |
| FFmpeg `-f null` | 793 | 3254 | 4032 | 4573 | 4854 |
| ours, writing YUV | 621 | 2427 | 2804 | 2799 | 2812 |
| FFmpeg `-f rawvideo`, writing YUV | 767 | 2381 | 2336 | 2290 | 2226 |

As a decoder this port is 1.20x slower than FFmpeg on one thread, 1.41x at ten slots and
1.45x at twelve. FFmpeg still gets 6.1x out of its one-thread time where this
one gets 5.1x, and that difference in scaling is most of the multi-thread gap. As the
job an integrator actually runs - file in, YUV out - this one is 2% ahead at six
slots, 20% ahead at eight, 22% at ten and 26% at twelve, because FFmpeg's own output
path costs it 50% of its threaded throughput (4573 down to 2290 frames per second at
ten threads) while the example's stdio copy costs 14% (3237 down to 2799).
Comparing one side writing against the other discarding is therefore worth up to
2.0x on its own, and earlier revisions of this file, which did exactly that, quoted
1.44x and 2.52x. `h264dec_example` now takes `-` as its output name to discard the
pictures, which is what makes the comparison possible.

The same script on the whole-program build, with its own FFmpeg runs taken in the
same minute rather than reusing the ones above:

| slots / threads | 1 | 6 | 8 | 10 | 12 |
| --- | --- | --- | --- | --- | --- |
| ours, `make lto`, pictures discarded | 659 fps | 2523 | 2978 | 3267 | 3313 |
| FFmpeg `-f null`, same runs | 780 | 3297 | 4011 | 4601 | 4902 |
| ours, `make lto`, writing YUV | 629 | 2436 | 2771 | 2855 | 2818 |
| FFmpeg `-f rawvideo`, same runs | 763 | 2389 | 2347 | 2283 | 2236 |

As a decoder that is 1.19x behind the reference on one thread and 1.48x at twelve
slots, where the `-O3` build of the same sources in the window above was 1.20x and 1.45x -
the two builds are now inside each other's noise on both columns, which is the finding
about `make lto` rather than a rounding difference. FFmpeg gets 6.3x out of its
one-thread time here where this gets 5.0x, so what is left of the multi-thread gap is
per-thread work rather than bookkeeping. As the job an integrator runs
- file in, YUV out - this build is 18% ahead at eight slots, 25% at ten and 26% at
twelve.

The budget below is the measurement that sent the filter off the decode thread. It
is taken on the build with the lock split but the filtering still inline: at twelve
slots on the 60s clip, with the same flags as the shipped build, the process holds
**7.8 of the sixteen logical cores** busy, and the per worker wall budget says why:
83 to 92% of wall is inside a picture, but **21 to 30% of wall is a thread blocked
in `h264_progress_await()`** waiting for a reference row another thread has not
finished filtering, 6 to 16% is a worker with no job queued, and the commit chain
is under 1%. The pool-wide mutex is no longer the story: it is held for 23.8% of
the wall in total (the owner's decode-order phase is 20.7% of it) and every one of
the twelve workers waits on it for about a millisecond across the whole clip,
15.6ms in total against a 618ms run.

Three experiments went looking for the missing cores and two came back empty.
Raising `H264_MAX_JOB_SLOTS` from 12 to 20 buys 1.5% at fourteen slots and nothing
past that, so it is not the slot count. Spinning on the atomic for up to a few
microseconds before blocking in the await - the ~7us wake latency looked like the
obstacle - measures 1.004x at ten slots and 0.99x at twelve, inside the 1.5% noise
of a one-second window, so the waits are real producer lag and not wakeup cost.
And making `h264_progress_await()` return immediately, which is wrong but bounds
the whole mechanism, only reaches 1.081x: what the await *mechanism* costs is
small; what the *waiting* costs is the 26% that nothing else can use, because
CABAC makes a picture's macroblocks strictly serial and a thread released from one
row wait has nowhere else in that picture to go.

The owner thread's serial phase was the next suspect, and part of it is larger than
it looks. Timed per access unit with the time-stamp counter, on the 60s clip:
activating the parameter sets takes 0.57 Mcyc over the whole run and the slice pass -
slice headers, reference lists, weighted tables, MMCO - 8.8 Mcyc, which is 5.9
kcycles of the 6.06 Mcyc a 720p picture costs on one thread. But `h264_field_start()`
takes 180 Mcyc: 120 kcycles a picture, 2.0% of the picture, and at twelve slots the
same phase takes 446 kcycles a picture against 1.37 Mcyc of wall each picture gets -
a third of the wall. Nearly all of it is `reset_meta()` zeroing the 1.36 MB of
per-macroblock arrays, which is the 111 kcycles measured where it stands below.

So the clear was moved off the owner thread: `h264_frame_start()` marks the picture
dirty and the decode pass clears its own arrays before its first macroblock, with
`h264_picture_pixels()` doing the same so a failed slice pass never reaches the
filter unwiped (and `ref2frm` stays with the owner, because the slice pass writes it
before any job runs). Output is byte-identical on four clips at one, six and twelve
slots. At one slot it is 0.997x, 1.001x and 1.012x - the same thread does the work
either way - and at twelve slots 1.007x, 0.996x and 0.983x, the interlaced clip
losing 1.7%. That is the finding: the owner's phase is serial only for the owner,
while every other thread decodes around it, so a third of the wall costs nothing
anyone waits for; put the same bytes in front of a picture's own decode pass and they
land ahead of that picture's first published row, which is exactly what the threads
waiting on those rows are blocked on. Reverted.

So the ceiling is the shape of the work, not the bookkeeping - and the bookkeeping is now
close to free: 11.8 of the twelve threads offered are doing work. The same sources as twelve *independent* processes reach
3412 fps in the window this was re-measured in (3117 in an earlier one, which is
what the desktop here costs) against 3343 threaded: **98% of the coordination-free
number**, where it had been 80% the same day before the trace flag went per-thread. The
whole-program build reads the same way from its own endpoint - 3404 as twelve processes
and 3313 threaded, 97% of that ceiling - from a single-thread cost that is the same 660
fps at `-O3` and 659 at `make lto`. About 2% of waiting is left for the pool to sell, and
what cross-module inlining had been buying was largely the same waiting that the shared
cache line was causing. And per picture this decoder is
still cheaper in one process (3.2ms of CPU) than across twelve processes
(3.9ms), since the reference planes are shared. The loop filter was the unit that
could be lifted out of a picture and handed around - 22.9% of single-thread time,
row-local, needing nothing from the decode pass but its finished rows - and
lifting it out is the 1.06 to 1.10x two paragraphs up. What that leaves is
genuinely serial: CABAC makes a picture's macroblocks one dependent run, so a
thread released from a row wait inside a picture still has nowhere in that picture
to go, and the only unit inside a picture that could be run in parallel again is a
slice boundary as the encoder chose it. The bench clip has one slice per picture,
so slice threading buys nothing here that this filter queue did not already take.
FFmpeg, for the size of what remains, lands 1.8x *above* its own twelve-process
number (2667 fps as twelve separate processes, 4777 as one threaded process): this
pool now gets 5.1x against a process ceiling of 5.2x, so about 2% is left
between the two, and the whole distance to the reference is per-thread work.

That last distance was measured before anyone refactored for it, and it is not
metadata traffic. `reset_meta()` zeroes about 1.36 MB of per-macroblock arrays for
every picture - as much again as the frame's pixels, 978 KB of it the motion
vector table - which looks like the obvious thing to remove; timed in place it is
111k TSC ticks per picture at one slot, 31us of the 1704us a picture takes there,
and 247k summed at twelve slots, 69us of the 3.29ms a picture costs then. About
2% either way, because `memset` runs at tens of GB/s. The other reading of the
same numbers is the useful one: a picture costs 1.71ms of CPU at one slot and
3.29ms at twelve, and the 1.58ms difference is thirteen runnable threads sharing
eight physical cores, while FFmpeg's picture goes 1.44ms to 1.65ms because it
blocks enough to keep roughly eight threads live. Its advantage under threading is
therefore the work a picture carries into every slot count, not its scheduling -
which puts the remaining effort on the single-thread path, starting with the
entropy decode that is 47% of it. Moving the clear itself was tried anyway, and is
the owner-thread experiment above.

The single-thread 1.34x is a different matter and is mostly not the decoder's
algorithms: FFmpeg's hot paths are hand-written asm where this port compiles
intrinsics, its demuxer does access-unit detection with a parser instead of the
example's byte scan, and it also decodes 8-bit 4:2:2 and 4:4:4, which this one
does not (its 10-bit matrix is complete).

Interlaced content is where this pool is weakest, and the reason is not the one
the code suggests. `tests/gen_bench.sh` now also writes `long_paff_720p.h264`:
20 s at 50 fields per second, 720p High with B-frames and weighted prediction,
1000 coded pictures of which 415 are field pictures. The field path is bit-exact -
1,382,400,000 bytes match FFmpeg's decode of the same stream, and six and ten
slots reproduce the one-slot output byte for byte. Field pictures are decoded in
the owner thread after draining every job (`dispatch_access_unit()` in
`src/h264dec.c`), because the two fields of a frame share one picture whose
`picture_structure`, `poc` and `have_*_field` state the second field's header
phase rewrites (`src/h264_slice.c:589`). That serialization looked like the thing
to remove, so it was removed and measured: fields dispatched as ordinary jobs,
with a wait for the matching first field in place of the whole-pool drain, stays
bit-exact at one, six and ten slots on all 27 streams and is worth 1.003x at six
slots and 1.006x at ten - medians of seven and five paired runs, which is
nothing. The drain is not on the critical path, so the shipped code keeps the
simpler rule.
What the curve does say is that interlaced content saturates earlier: on the
default `-O3` build with the pictures discarded, 642 pictures per second at one slot,
833 at two, 1335 at four, 1657 at six, 1887 at eight, and then 1900 at ten and 1918 at
twelve, which is the top - while the progressive clip still climbs to 2616 (611 at
one slot, 2023 at six, 2493 at ten). A
PAFF stream carries twice the coded pictures for the same pixel work, and
everything done per picture - parameter-set activation, slice headers, reference
list build, MMCO, POC, output queueing - is decode-order work on one thread.
FFmpeg on the same stream, measured alternately with this build and also discarding,
does 868 pictures per second at `-threads 1` and 3623 at twelve, so here the
single-thread gap is the usual 1.33x while the threading gap is 1.9x.
Closing what is left of that means taking the per-picture phase apart - header
work that need not happen in decode order on one thread, and slice threading where
the encoder actually cut slices - rather than threading fields, which was tried and
measured 1.003x.

Where the time goes now: each stage stubbed out in turn in a scratch tree, best
of five runs of the 60s clip at one slot.

| stage removed | frame time | that stage |
| --- | --- | --- |
| nothing (the build as shipped) | 2.59 ms | - |
| all luma and chroma MC pixel work | 2.28 ms | 12.1% |
| deblocking pixel filters (SSE2, bS and caches still filled) | 2.35 ms | 9.4% |
| deblocking entirely, caches included | 2.08 ms | 19.8% (so the cache and bS work alone is 10.4%) |
| inverse transforms and residual add | 2.51 ms | 3.2% |

About 65% of decode time is therefore none of the above: entropy decoding, the
per-macroblock bookkeeping around it, and the DPB (intra prediction is under 1%,
see below). That is the
part a SIMD kernel cannot reach - CABAC is a serial dependency chain by
construction, and the reference decodes it in C too - which is why the kernel
work stops here. The bS and filter-cache fill is the largest single remaining
target at 10.4%; it walks MB neighbour caches and compares motion vectors, and
FFmpeg leaves it scalar as well.

The 65% has since been split, in the same kind of scratch tree but with rdtsc
accumulators rather than stubs - around the entropy call, the reconstruction
call and the per-row loop filter in the macroblock loop of
`h264_decode_slice_data`, with the whole slice as the denominator. At one slot
on the 60s 720p clip, 5.4M macroblocks:

| phase | share |
| --- | --- |
| entropy decode (`h264_decode_mb_cabac` / `_cavlc`) | 46.9% |
| reconstruction (`h264_hl_decode_mb`: MC, transforms, intra, caches) | 27.7% |
| in-loop filter, one row behind | 22.9% |
| slice loop bookkeeping | 2.4% |

On the interlaced clip the shape is the same (40.9 / 32.4 / 24.1 / 2.6 over
3.68M macroblocks). Two things follow. The loop filter's 22.9% agrees with the
19.8% the stub method gave for the same stage, which is the cross-check that
neither method is off by much. And entropy is the single largest block by
far - bigger than every pixel stage put together - so the 1.34x that still
separates this decoder from FFmpeg on one thread lives mostly in the C syntax
decoder, where the two implementations differ in codegen and in per-symbol
detail rather than in SIMD coverage. That is also why the build is `-O3`: it is
worth 5.8% of single-thread time on this profile, more than any kernel work
left on the table here.

Splitting the entropy stage further, by snapshotting bin counters around the
three residual wrappers (counting only, so behaviour is untouched): the 60s
720p clip decodes 117.7M regular bins and 21.9M bypass bins, of which the
residual-token decoder accounts for 71.9% and 89.3% - and on the interlaced
clip 64.9% and 87.6%. That is the bin count; timing it (rdtsc around the same
wrappers, so this one build is a few percent slower overall and the shares are
what matter) puts the residual decoder at 56.6% of the per-macroblock syntax
time - 27.4 cycles per bin - against 61.9 cycles per bin for the rest of it, the
mode, partition, reference and MVD decoding. So the per-bin cost in the syntax
layer is more than double the coefficient layer it sits around, and that layer is
21% of single-thread decode time. The neighbour-cache fill everyone expects to be
the cost there is not: 44 Mcyc, 0.9% of the syntax time.

Three explanations for the interlaced shortfall were tested and rejected.
Threading the field pictures: 1.003x, above. Twice the coded pictures per frame
of pixels: an MBAFF clip of the same source, same encoder settings and same
duration codes 500 pictures in 20 s, exactly the 25 pictures per second of the
progressive clip, and it still saturates early - 3.11x for progressive at six
slots and 3.47x at ten, against 2.75x and 2.84x for MBAFF and 2.70x and 2.68x for
the PAFF clip. The loop filter and its row publication: with the fifth argument
to the example turning deblocking off, the ordering is the same (3.18x and 3.38x
progressive, 2.99x and 2.76x MBAFF, 2.84x and 2.60x PAFF). What is left standing
is the memory access pattern of field coding. On one thread a PAFF field costs
1.82x what a progressive frame of the same source costs per frame-equivalent of
picture (601 fields per second, so 300 frames' worth of lines), while MBAFF,
which does not split lines, costs 0.96x - the shortfall tracks whether the
reconstruction touches alternating lines, not how many pictures or filter rows
there are.

Two things measured and rejected on the way there. FFmpeg keeps one bin accessor
out of line for the cold branches; restoring that here cost 0.9% at 720p and
2.6% on the interlaced clip, so this port's all-inline spelling stays. And the
CABAC bounds check on refill reads costs well under a percent (a refill happens
once every ~22 bins, 6.5M of them for 145M bins), so dropping it in favour of a
padded buffer - which is what the reference does - is not worth weakening the
guarantee that a damaged stream cannot read past a slice.

Each stub number is an upper bound, since a stage removed also changes what the
following stages see (unfiltered neighbours make the loop filter fire differently
in the MC-stubbed build), so read them as shares of the whole rather than as
exact timings.

Weighted prediction was the cost that table missed, because it is not one of the
rows: stubbing it out (an env-gated early return in both entries, so one binary
measures both) gives 10.4% of the 720p clip and 14.6% of the 1080p one at one
slot, since every macroblock the unipred path copies goes through a weight pass.
Counting the arguments the decoder really passes says what to do: of 2.59M
single-list calls on the 720p clip, 98% have `weight == 1 << denom` with
`offset == 0` - the identity, multiplying by one over 598M pixels - and no call
has |weight| above 1; the biweighted path is never identity, because implicit
weighting on these clips gives |weight| <= 48 at denom 5. So both dispatchers
now return without touching pixels in the identity case, the biweight dispatchers
take a `pavgusb` path when the two weights are equal and the offset is zero, and
the rest runs an SSE2 kernel with one int16 lane per pixel. The kernel is gated
to parameters whose product cannot overflow int16 - |weight| <= 128, denom <= 7
single and <= 6 double, scaled offset inside int16 - and outside that the scalar
reference runs. Both halves of that gate were checked by breaking them: replacing
`paddsw` with `paddw` makes the sweep report pixels that wrap to 0 where the
reference clips to 255 (a 255 pixel at weight 128 with a 16320 scaled offset),
and dropping `offset == 0` from the identity test gives 528 mismatching cases -
which is how the first version of this change was caught doing nothing at all,
since its dispatch blocks were wrapped in a guard macro this file does not
define and the sweep stayed green. Worth 1.100x at 720p and 1.154x at 1080p on
one slot, medians of seven paired runs, and 1.052x at ten slots - less there,
because part of the saving is memory traffic that is not the binding constraint
once six or ten workers share it.

Intra prediction, the other thing FFmpeg ships an SSE2 kernel for and this port
does not, measures under 1% on both clips - the build with every luma and chroma
prediction stubbed out is no faster than the baseline (454 fps against 456) - so
it stays scalar here.

Memory holds flat across re-opens, which is the case a single decode cannot
show. `tests/soak.sh` opens, drains and closes the decoder inside one process:
300 rounds of a 30-frame stream at one slot stay at 5.0 MB working set (+0.32 MB
between the second and the last round), the same with deblocking off, and 60
rounds of the 1500-frame clip at four slots climb 18.3 → 19.2 MB by round 40 and
then stop moving - the allocator settling rather than a leak, which is why the
test compares round 2 to the end against a growth limit instead of printing a
number, and checks the frame count every round so a decode that drifts fails too.

Flat RSS was not the same as no leak, though. `h264_thread_close()` joined only
the workers whose flag was still set, and the worker cleared that flag as it
exited, so every thread that finished before the owner reached it was neither
joined nor closed. The pages go back either way, which is why the soak never
flagged it, but the handle and, on glibc, the thread's stack mapping do not: 1500
re-opens at twelve slots took `VmPeak` from 34.6 GB to 86.4 GB in three seconds on
Linux, where the fixed build sits flat at 0.96 GB, and 400 re-opens left 3344
handles on Windows against 78 for the fixed build. The flag is now `spawned`,
written only by the owner, so a thread that is already waiting to be reaped is
still reaped.

Known issues
------------

* Interlaced streams stop scaling at about 3.0x where progressive ones reach 5.1x, and
  they peak at eight slots (3.2x) rather than at twelve - the 60s interlaced clip measures
  698, 2214 and 2102 frames per second at 1, 8 and 12 slots on the current build. It is not
  the field pictures being decoded in the owner thread - threading
  them was tried and measured 1.003x. The per-picture decode-order phase is what
  saturates; see the interlaced paragraph above.

* Parameter set lifetime is the thing that used to break threading. The owner
  takes a reference while it parses a slice header - outside the pool lock,
  because header parsing must not wait behind a reconstruction - at the same
  time as a worker drops the references of the picture it finished, inside the
  lock. With plain `++`/`--` that lost an update and freed a set that still had
  holders: a segfault in about 2% of clean runs at two or more slots and 3-5% of
  truncated ones, always reading `refcount` in `h264_sps_unref`/
  `h264_pps_unref`. The counts are atomic now, and the active pair holds its own
  reference instead of borrowing the table's, which is what FFmpeg's
  `AVBufferRef` counts and `ps->sps`/`ps->pps` give it. A count that drops below
  zero is reported.

* Damaged pictures are reported, not hidden: `b_decode_error` is set and macroblocks
  that never decoded are painted mid gray, so `au_errors` from
  `h264_dec_send_packet` is only the synchronous half of that story. With frame
  threading an AU can fail after `h264_dec_send_packet` has already returned, so
  check `b_decode_error` on the picture as well.

Layout
------

    src/h264dec.c      public API, NAL dispatch, access unit and job handling
    src/h264_nal.c     Annex-B and length-prefixed splitting, RBSP unescaping
    src/h264_ps.c      SPS/PPS, VUI, scaling lists, dequant tables
    src/h264_sei.c     SEI walk
    src/h264_slice.c   slice headers, picture start/end, POC, reference marking
    src/h264_frame.c   picture pool, DPB, reference lists, MMCO, output reordering
    src/h264_cavlc.c   CAVLC syntax
    src/h264_cabac.c   CABAC engine and context state
    src/h264_mb.c      per-macroblock reconstruction, motion compensation, caches
    src/h264_direct.c  spatial and temporal direct prediction
    src/h264_deblock.c in-loop deblocking, one row behind the decoder, kernel dispatch
    src/h264_idct.c    4x4/8x8 transforms, Hadamard, lossless add, dispatch
    src/h264_idct16.c  the 10-bit transform path (int32 coefficients, 16-bit samples)
    src/h264_intrapred.c intra prediction, including the lossless fusion
    src/h264_intrapred16.c the 10-bit intra predictors
    src/h264_mc.c      motion compensation dispatch
    src/h264_mc16.c    the 10-bit motion compensation, weighted prediction and edge emulation
    src/h264_cpu.c     CPUID / HWCAP probe and the SIMD enable switch
    src/h264_dsp_sse2.c SSE2 quarter-pel, full-pel, chroma, transform, deblocking
                     and weighted-prediction kernels
    src/h264_dsp_neon.c the same kernels in NEON, for an ARM target
    src/h264_dsp_avx2.c AVX2 quarter-pel for 16-wide luma blocks
    src/h264_tables.c  shared tables and the debug-environment switch
    src/h264_thread.c  worker pool, row progress, the stealable filter queue, locking
