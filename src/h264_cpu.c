/*
 * CPU feature detection.
 *
 * The vector kernels are compiled for their own instruction set, so the
 * library can be built for the plain x86-64 baseline and still use SSE2 or
 * AVX2 on a CPU that has it. AVX2 also needs the operating system to keep the
 * upper halves of the registers, which is what XGETBV says.
 *
 * On ARM the decision is split the same way: whether the NEON kernels exist at
 * all is a compile-time property of the target (aarch64 always has them, an
 * ARMv7 build has them only if it was compiled with NEON enabled), and whether
 * this CPU runs them is decided here. Both flags default to 0, so a probe that
 * cannot answer is a scalar decode, and H264_DISABLE_SIMD=1 forces the same.
 */
#include "h264_internal.h"

int h264_have_sse2 = 0;
int h264_have_avx2 = 0;
int h264_have_neon = 0;

#if defined(H264_SIMD_ARM)
# if defined(__linux__) || defined(__ANDROID__)
#  include <sys/auxv.h>
#  include <asm/hwcap.h>
# endif
#endif

#if defined(__i386__) || defined(__x86_64__)

#include <cpuid.h>

static int cpu_xgetbv_ymm_enabled(void)
{
    unsigned int lo = 0, hi = 0;

#if defined(__GNUC__) && !defined(__OPTIMIZE_SIZE__)
    __asm__ __volatile__("xgetbv" : "=a"(lo), "=d"(hi) : "c"(0));
#endif
    return (lo & 0x6) == 0x6;
}

static void cpu_probe(void)
{
    unsigned int a, b, c, d;

    if (!__get_cpuid(1, &a, &b, &c, &d))
        return;
    if (!(d & (1u << 26)))                    /* SSE2 */
        return;
    h264_have_sse2 = 1;

    /* AVX2 needs OSXSAVE + AVX so that the OS keeps the upper halves of the
     * registers, which is what XGETBV reports, and then leaf 7 bit 5 */
    if ((c & (1u << 27)) && (c & (1u << 28)) && cpu_xgetbv_ymm_enabled()) {
        if (__get_cpuid_max(0, NULL) >= 7 &&
            __get_cpuid_count(7, 0, &a, &b, &c, &d) && (b & (1u << 5)))
            h264_have_avx2 = 1;
    }
}

#endif

#if defined(H264_SIMD_ARM)
static void cpu_probe_arm(void)
{
#if defined(__aarch64__)
    /* Advanced SIMD is part of the ARMv8 architecture itself: there is no
     * aarch64 CPU without NEON, so a probe could only agree with the compiler.
     * The kernels were compiled because the target has the ISA. */
    h264_have_neon = 1;
#elif (defined(__linux__) || defined(__ANDROID__)) && defined(AT_HWCAP)
    /* ARMv7 with NEON is optional, and the kernel is the one that knows */
    h264_have_neon = (getauxval(AT_HWCAP) & HWCAP_NEON) != 0;
#endif
}
#endif

/* run at load time so no entry point can forget to ask for it */
__attribute__((constructor)) static void cpu_probe_on_load(void)
{
    const char *no_simd = getenv("H264_DISABLE_SIMD");

    /* every flag stays 0, which is the scalar path, if the operator asked for
     * no SIMD or if the probe below cannot answer for this CPU */
    if (no_simd && no_simd[0] == '1')
        return;

#if defined(H264_SIMD_X86)
    cpu_probe();
#endif
#if defined(H264_SIMD_ARM)
    cpu_probe_arm();
#endif
}
