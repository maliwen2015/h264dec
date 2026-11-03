CC      ?= gcc
# x86-64 baseline on purpose: the vector kernels carry their own target
# attribute and are selected at run time (src/h264_cpu.c). For a build tuned to
# one machine, append e.g. CFLAGS="-O3 -march=native".
# An aarch64 build needs no flag either: NEON is part of the architecture, so
# src/h264_dsp_neon.c compiles as it stands and picks itself up from the wildcard
# below, and the same is true of the SSE2 file, which becomes an empty object.
# An ARMv7 build would have to add -mfpu=neon for this file only, which is why
# the NEON path is documented as an aarch64 one.
# -O3, like the reference builds: entropy decoding is 47% of single-thread time
# and the C paths gain 5.8% at one slot, 3.6% at ten, measured bit-exact.
# -flto is worth more again, and a great deal more once the picture is shared out -
# see the lto target below. It stays opt-in for the link time, and its archive still
# carries machine code only because the objects are fat.
CFLAGS  ?= -O3 -g -std=c99 -Wall -Wextra -Wno-unused-parameter
LDFLAGS ?=
LIBS    ?=

# the soak test reads the process working set, which needs psapi on Windows
UNAME_S := $(shell uname -s 2>/dev/null)
ifneq (,$(filter MINGW% MSYS% CYGWIN%,${UNAME_S}))
SOAK_LIBS := -lpsapi
endif
# the worker pool is pthreads off Windows, and glibc before 2.34 keeps it in a
# separate library - Ubuntu 22.04 links without this, Ubuntu 20.04 does not
ifneq (,$(filter Linux Darwin FreeBSD GNU,${UNAME_S}))
LIBS += -lpthread
endif

SRC := $(wildcard src/*.c)
HDRS := $(wildcard src/*.h) h264dec.h
OBJ := $(patsubst src/%.c,build/%.o,$(SRC))

all: h264dec_example

build:
	mkdir -p build

# The coefficient loop in the CABAC syntax decoder is a serial dependency chain
# - each level reads the run before it - and vectorizing it costs: compiling this
# one file without the vectorizer is worth 1.051x at one slot, against 1.042x for
# switching it off tree-wide, which also takes the vectorizer away from the
# scalar DSP fallbacks that do benefit from it.
build/h264_cabac.o: src/h264_cabac.c $(HDRS) | build
	$(CC) $(CFLAGS) -fno-tree-vectorize -I. -c $< -o $@

build/%.o: src/%.c $(HDRS) | build
	$(CC) $(CFLAGS) -I. -c $< -o $@

libh264dec.a: $(OBJ)
	ar rcs $@ $^

h264dec_example: example_decoder.c libh264dec.a
	$(CC) $(CFLAGS) -I. $< libh264dec.a $(LDFLAGS) $(LIBS) -o $@

build/dsp_check: tests/dsp_check.c libh264dec.a | build
	$(CC) $(CFLAGS) -w -Isrc -I. $< libh264dec.a $(LDFLAGS) $(LIBS) -o $@

build/sei_check: tests/sei_check.c libh264dec.a | build
	$(CC) $(CFLAGS) -w -Isrc -I. $< libh264dec.a $(LDFLAGS) $(LIBS) -o $@

build/dsp_bench: tests/dsp_bench.c libh264dec.a | build
	$(CC) $(CFLAGS) -w -Isrc -I. $< libh264dec.a $(LDFLAGS) $(LIBS) -o $@

build/avcc_check: tests/avcc_check.c libh264dec.a | build
	$(CC) $(CFLAGS) -w -I. $< libh264dec.a $(LDFLAGS) $(LIBS) -o $@

build/meta_dump: tests/meta_dump.c libh264dec.a | build
	$(CC) $(CFLAGS) -w -I. $< libh264dec.a $(LDFLAGS) $(LIBS) -o $@

build/soak: tests/soak.c libh264dec.a | build
	$(CC) $(CFLAGS) -w -I. $< libh264dec.a $(LDFLAGS) $(LIBS) $(SOAK_LIBS) -o $@

build/api_check: tests/api_check.c libh264dec.a | build
	$(CC) $(CFLAGS) -w -I. $< libh264dec.a $(LDFLAGS) $(LIBS) -o $@

# re-open / channel-switch leak soak: rounds of open, decode, drain and close in
# one process, with the working set printed per round
soak: build/soak
	sh tests/soak.sh

# same build at different slot counts: a one-slot reference is written, then
# every other count has to reproduce it byte for byte. Point SELF_REF at a
# directory written by another platform's build to check portability instead.
SELF_REF ?= /tmp/h264selfref
selfcheck: h264dec_example
	rm -rf $(SELF_REF)
	sh tests/selfcheck.sh $(SELF_REF)
	for t in 0 2 3 6 8 10 12; do THREADS=$$t sh tests/selfcheck.sh $(SELF_REF) || exit 1; done

# bit-exact against FFmpeg, kernel equivalence, SEI walker, AVCC input,
# exported field order, re-open stability
check: h264dec_example build/dsp_check build/sei_check build/avcc_check build/meta_dump build/soak build/api_check
	./build/dsp_check
	./build/sei_check
	./build/api_check ../testdata/main_cabac.h264 1
	./build/api_check ../testdata/b_cavlc.h264 6
	./build/api_check ../testdata/b_cavlc.h264 12
	sh tests/compare.sh
	sh tests/avcc.sh
	sh tests/meta.sh
	LONG=5 CHURN=60 sh tests/soak.sh
	LONG=5 CHURN=1 LONG_THREADS=10 sh tests/soak.sh
	# two slots is the pool shape that can strand a picture on the filter list,
	# and nothing else here reaches it: selfcheck runs it only over the short
	# streams, and both soak lines run wide
	sh tests/claim_hammer.sh

# High 10 corpus: generate it if needed, then compare against FFmpeg at one,
# eight and twelve slots. The streams live in ../testdata/10bit (ignored).
check10: h264dec_example
	sh tests/gen_10bit.sh
	DATA=../testdata/10bit THREADS=1 sh tests/compare.sh
	DATA=../testdata/10bit THREADS=8 sh tests/compare.sh
	DATA=../testdata/10bit THREADS=12 sh tests/compare.sh

fuzz: h264dec_example
	sh tests/fuzz.sh

# Throughput against the reference decoder, with both sides discarding the
# pictures. Needs testdata/bench and ffmpeg on PATH.
bench: h264dec_example
	sh tests/bench_vs_ffmpeg.sh

# Which profiles and chroma formats this decoder accepts, with FFmpeg as the
# reference label and pixel comparison. Informational: unsupported formats are
# printed as such, not treated as a test failure.
features: h264dec_example
	sh tests/feature_matrix.sh

# Differential sweep against the reference over encoder settings the curated
# corpus does not reach - it encodes its own streams into ../testdata/sweep,
# which is ignored, and compares at one, eight and twelve slots. Needs libx264.
sweep: h264dec_example
	sh tests/sweep_vs_ffmpeg.sh

# stability judged against the reference: the same bytes out of any thread count
# for both decoders, and the fuzz corpus through both
stability: h264dec_example
	sh tests/stability.sh

# Profile-guided build: instrument, run the corpus, then rebuild with what the
# run recorded. The second pass has to keep build/*.gcda, so only the objects are
# removed. Two warnings are silenced for this pass only, both of them artefacts
# of compiling with a profile: -Wmissing-profile for paths the corpus never
# takes (the damaged-stream handling), and -Wpsabi, which fires when the
# optimiser clones one of the static __m256i-returning quarter-pel helpers - the
# clone is diagnosed at the end of the translation unit even though every caller
# of that helper is in the same file, so there is no ABI to break.
PGO_GEN := $(CFLAGS) -fprofile-generate
PGO_USE := $(CFLAGS) -fprofile-use -fprofile-correction \
           -Wno-missing-profile -Wno-psabi
pgo:
	rm -rf build libh264dec.a h264dec_example h264dec_example.exe
	$(MAKE) CFLAGS="$(PGO_GEN)" h264dec_example
	sh tests/pgo_train.sh
	rm -f build/*.o libh264dec.a h264dec_example h264dec_example.exe
	$(MAKE) CFLAGS="$(PGO_USE)" h264dec_example
	@echo "pgo: rebuilt with the recorded profile - run make check to confirm"
	@echo "pgo: the objects now carry the profile, so a plain make keeps this build;"
	@echo "     run make clean to go back to the plain -O3 build"

# Link-time optimisation, which is where the threaded time used to be: the row-progress
# queue is one function awaited from h264_mb.c and h264_direct.c and another reported from
# h264_slice.c and h264_deblock.c, both defined in h264_thread.c, and only a
# whole-program build can inline across them. Against this same -O3 build that measured
# 1.19x to 1.24x at twelve slots. It buys nothing now: medians of five paired runs in both
# orders, discarding the pictures, give 1.014x, 1.005x, 1.011x, 0.994x and 0.965x at 1, 4,
# 6, 10 and 12 slots on 720p and a mean of 1.01x on 1080p, inside the band the same binary
# shows against itself. What the inlining had been recovering was mostly waiting, and the
# largest single cause of that waiting was a shared trace flag bouncing one cache line
# between the filter threads - see the README paragraph that starts "A shared debug flag".
# Scaling from one slot to twelve is 5.1x either way now. The target stays: it is free to
# provide, still byte-identical over the corpus at one, six and twelve slots, and it is what
# the Linux whole-program leg exercises.
# Byte-identical output over
# the 27-stream corpus at one, six and twelve slots.
#
# -ffat-lto-objects is what lets the objects be used twice: each one keeps its compiled
# machine code next to the GIMPLE. The IL is taken back out before the archive is
# written, because a whole-program result cannot be put back into per-file objects - an
# integrator gets the -O3 code from libh264dec.a either way - and because an archive
# carrying IL is a promise that the next link uses a compiler that can read it, which a
# shipped library should not make. It would also charge every consumer link for the
# pass: measured here, one test program took 2m15s to link against the fat archive and
# 0.23s against the stripped one, and the fat link printed the diagnostic below in a
# build that never asked for -flto. What is left in the members afterwards on Linux
# is .gnu.debuglto_*, the compressed debug info of a -g whole-program compile, which a
# plain link ignores; the IL is gone, and on both systems a -O2 consumer of the archive
# links it and runs api_check clean.
#
# So the whole-program binary is its own artifact, h264dec_example_lto, copied off the
# example this target links, and the plain build stays the plain build. A separate name
# is the only stable arrangement on Windows: mingw32-make cannot stat h264dec_example -
# only the .exe is on the filesystem - so it counts that target as missing and re-links
# it from whatever the archive holds, which turned an LTO binary back into an -O3 one
# quietly, in the middle of the gate run. Overwriting h264dec_example would mean every
# gate after this target tests a different binary from the one the target built.
#
# The copy is of a link through the archive, not of the objects: linking the example
# straight from build/*.o instead measures 0.891x on 720p and 0.881x on 1080p at twelve
# slots, which is most of the gain this target is for. Whole-program optimisation splits
# into link-time jobs differently the two ways, and the split that comes out of the
# archive is the better one, so the target keeps the ordinary link rule and copies its
# result while it is still whole-program.
#
# -Wno-stringop-overflow is for the link-time pass only. With the whole program in
# hand it folds h264_scan8[0] to its constant, 12, and then reads the destination of
# a direct-mode neighbour-cache fill as the two-byte element at that slot rather than
# the 40-element cache row it belongs to, so it diagnoses a 2-byte write at offsets
# 26..78 of mvd_cache "of size 26". Those are exactly the four writes per row of a
# 4x4 fill at stride 8 starting on slot 12, which end on slot 39 - the last slot
# there is. See the README for the probe that checks it. Passing the destination
# through uintptr_t first, which is the usual way of breaking that association, does
# not: GCC keeps the base object across the round trip and still reports 30.
LTO       := -flto -ffat-lto-objects -Wno-stringop-overflow
OBJCOPY   ?= objcopy
lto:
	rm -rf build libh264dec.a h264dec_example h264dec_example.exe \
		h264dec_example_lto h264dec_example_lto.exe
	$(MAKE) CFLAGS="$(CFLAGS) $(LTO)" all
	cp h264dec_example.exe h264dec_example_lto.exe 2>/dev/null || \
		cp h264dec_example h264dec_example_lto
	for o in $(OBJ); do $(OBJCOPY) --remove-section='.gnu.lto_*' $$o; done
	ar rcs libh264dec.a $(OBJ)
	@echo "lto: h264dec_example_lto is the whole-program binary, libh264dec.a is plain"
	@echo "     gate it with BIN=./h264dec_example_lto sh tests/selfcheck.sh <ref>"
	@echo "lto: for both, make pgo CFLAGS=\"-O3 -g -std=c99 -Wall -Wextra \\"
	@echo "         -Wno-unused-parameter $(LTO)\" - that path does not strip, so its"
	@echo "     archive carries IL and every later link re-runs the pass"

clean:
	rm -rf build libh264dec.a h264dec_example h264dec_example.exe \
		h264dec_example_lto h264dec_example_lto.exe *.gcda *.gcno

.PHONY: all check check10 fuzz bench sweep stability pgo lto clean features
