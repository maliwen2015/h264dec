cmake_minimum_required(VERSION 3.15)

project(h264dec C)

# The Makefile builds with -O3 -g -std=c99 -Wall -Wextra -Wno-unused-parameter.
# Mirror that: default to Release (GCC Release is -O3) when no build type is given.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# The library is written against GCC/Clang extensions: "#pragma GCC target"
# for the per-file SIMD kernels (src/h264_dsp_sse2.c, src/h264_dsp_avx2.c),
# __attribute__((constructor)) for the load-time CPU probe (src/h264_cpu.c),
# and <cpuid.h>. MSVC has none of these, so it cannot build this source tree;
# MinGW, MSYS, Cygwin and cross toolchains all use GCC or Clang and are fine.
if(MSVC)
    message(FATAL_ERROR
        "h264dec requires a GCC- or Clang-based toolchain (MinGW, MSYS2, "
        "cross-compilers); MSVC cannot compile the SIMD kernels and the "
        "CPU probe, which use '#pragma GCC target' and GNU attributes.")
endif()

option(H264DEC_BUILD_TESTS "Build the test programs" ON)
option(H264DEC_ENABLE_LTO "Enable link-time optimisation" OFF)

# x86-64 baseline on purpose: the vector kernels carry their own target
# attribute and are selected at run time (src/h264_cpu.c), so no global ISA
# flags are needed. For a build tuned to one machine, configure with e.g.
#   -DCMAKE_C_FLAGS="-march=native"
# ---------------------------------------------------------------------------
# SIMD path detection (configure-time report + the few per-file flags the
# Makefile documents):
#
#   x86-64 / i386  SSE2 and AVX2 kernels are compiled (the files carry their
#                  own '#pragma GCC target'); whether the CPU uses them is
#                  decided at load time by the probe in src/h264_cpu.c, or
#                  forced to scalar with H264_DISABLE_SIMD=1 in the
#                  environment. 32-bit x86 needs -msse2 per file: SSE2 is in
#                  the x86-64 baseline but not the i386 one.
#   aarch64        NEON is part of the architecture, the kernels compile as
#                  they stand, no flag.
#   armv7          the NEON file needs -mfpu=neon for that file only; the
#                  dispatch stays scalar unless the compiler saw NEON too
#                  (the Makefile documents the NEON path as an aarch64 one).
#   anything else  RISC-V, ARM without NEON, ... compile the vector files as
#                  empty translation units and get the scalar C path only.
set(H264DEC_SIMD_REPORT "scalar C only")
set(H264DEC_SIMD_X86 OFF)
set(H264DEC_SIMD_ARM32 OFF)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|i[3-6]86|[xX]86)$")
    set(H264DEC_SIMD_X86 ON)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(H264DEC_SIMD_REPORT "x86-64: SSE2 + AVX2 kernels, runtime CPU probe")
    else()
        set(H264DEC_SIMD_REPORT "i386: SSE2 + AVX2 kernels (with -msse2), runtime CPU probe")
    endif()
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|[Aa][Rr][Mm]64)$")
    set(H264DEC_SIMD_REPORT "aarch64: NEON kernels (part of the architecture)")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|[Aa][Rr][Mm])")
    set(H264DEC_SIMD_ARM32 ON)
    set(H264DEC_SIMD_REPORT "armv7: NEON kernels compiled with -mfpu=neon")
endif()

message(STATUS "h264dec: target   ${CMAKE_SYSTEM_NAME} / ${CMAKE_SYSTEM_PROCESSOR} / ${CMAKE_C_COMPILER_ID}")
message(STATUS "h264dec: SIMD     ${H264DEC_SIMD_REPORT}")

# --- per-file SIMD flags -----------------------------------------------------
if(H264DEC_SIMD_X86 AND CMAKE_SIZEOF_VOID_P EQUAL 4)
    # i386 baseline has no SSE2; the kernel files include <emmintrin.h>.
    set_source_files_properties(src/h264_dsp_sse2.c src/h264_dsp_avx2.c
        PROPERTIES COMPILE_OPTIONS "-msse2")
endif()

if(H264DEC_SIMD_ARM32)
    include(CheckCCompilerFlag)
    check_c_compiler_flag(-mfpu=neon H264DEC_HAVE_MFPU_NEON)
    if(H264DEC_HAVE_MFPU_NEON)
        set_source_files_properties(src/h264_dsp_neon.c
            PROPERTIES COMPILE_OPTIONS "-mfpu=neon")
    else()
        message(WARNING "h264dec: -mfpu=neon unsupported, armv7 builds the scalar path only")
    endif()
endif()

# --- sources -----------------------------------------------------------------
file(GLOB H264DEC_SOURCES ${CMAKE_CURRENT_LIST_DIR}/src/*.c)

add_library(h264dec STATIC ${H264DEC_SOURCES})
target_include_directories(h264dec PUBLIC ${CMAKE_CURRENT_LIST_DIR})

if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
    # The coefficient loop in the CABAC syntax decoder is a serial dependency
    # chain - each level reads the run before it - and vectorizing it costs:
    # see the Makefile. GCC-only: Clang has no -fno-tree-vectorize.
    set_source_files_properties(src/h264_cabac.c
        PROPERTIES COMPILE_OPTIONS "-fno-tree-vectorize")
elseif(CMAKE_C_COMPILER_ID MATCHES "Clang")
    # The Clang equivalent, if this Clang knows it.
    include(CheckCCompilerFlag)
    check_c_compiler_flag(-fno-vectorize H264DEC_HAVE_NO_VECTORIZE)
    if(H264DEC_HAVE_NO_VECTORIZE)
        set_source_files_properties(src/h264_cabac.c
            PROPERTIES COMPILE_OPTIONS "-fno-vectorize")
    endif()
endif()

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(h264dec PRIVATE -Wall -Wextra -Wno-unused-parameter)
    # The Makefile compiles with -g even at -O3.
    target_compile_options(h264dec PRIVATE $<$<CONFIG:Release>:-g>)
endif()

# The worker pool is Win32 threads off pthreads: h264_thread.c includes
# <windows.h>/<process.h> on Windows and <pthread.h> elsewhere. glibc before
# 2.34 keeps pthreads in a separate library, hence the explicit link.
find_package(Threads)
if(NOT WIN32)
    target_link_libraries(h264dec PUBLIC Threads::Threads)
endif()

add_executable(h264dec_example example_decoder.c)
target_link_libraries(h264dec_example PRIVATE h264dec)

if(H264DEC_BUILD_TESTS)
    # The tests silence warnings wholesale (-w in the Makefile): they poke at
    # internals and are not held to the library's warning standard.
    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        add_compile_options(-w)
    endif()

    add_executable(dsp_check   tests/dsp_check.c)
    add_executable(sei_check   tests/sei_check.c)
    add_executable(dsp_bench   tests/dsp_bench.c)
    add_executable(avcc_check  tests/avcc_check.c)
    add_executable(meta_dump   tests/meta_dump.c)
    add_executable(soak        tests/soak.c)
    add_executable(api_check   tests/api_check.c)

    # dsp_check / sei_check / dsp_bench include from src/ and the root;
    # avcc_check / meta_dump / soak / api_check only need the public header.
    foreach(t dsp_check sei_check dsp_bench)
        target_include_directories(${t} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)
    endforeach()

    foreach(t dsp_check sei_check dsp_bench avcc_check meta_dump soak api_check)
        target_link_libraries(${t} PRIVATE h264dec)
    endforeach()

    # The soak test reads the process working set, which needs psapi on Windows.
    if(WIN32)
        target_link_libraries(soak PRIVATE psapi)
    endif()

    enable_testing()
    add_test(NAME dsp_check COMMAND dsp_check)
    add_test(NAME sei_check COMMAND sei_check)
endif()

if(H264DEC_ENABLE_LTO)
    include(CheckIPOSupported)
    check_ipo_supported(RESULT h264dec_ipo OUTPUT h264dec_ipo_reason)
    if(h264dec_ipo)
        set_property(TARGET h264dec h264dec_example PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
        if(TARGET soak)
            set_property(TARGET soak PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
        endif()
    else()
        message(WARNING "LTO not supported by this toolchain: ${h264dec_ipo_reason}")
    endif()
endif()
