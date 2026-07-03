/*
 * jps_simd.h
 * 128-bit SIMD abstraction for horizontal jump-point scans.
 * Supported backends: SSE2 and NEON. No scalar fallback is kept.
 */

#ifndef JPS_SIMD_H
#define JPS_SIMD_H

#include <stdint.h>

#if defined(_M_X64) || defined(__x86_64__) || defined(__SSE2__) || (defined(_M_IX86_FP) && _M_IX86_FP >= 2)

#  define JPS_HAVE_SIMD 1
#  define JPS_SIMD_SSE2 1
#  include <emmintrin.h>

typedef __m128i jps_v128;

/* 使用寄存器内建避免内存中转：对 x64/x86 SSE2 平台减少 load/store 开销。 */
static inline jps_v128 jps_v_set2(uint64_t lo, uint64_t hi)
{
#if defined(_MSC_VER)
    return _mm_set_epi64x((long long)hi, (long long)lo); /* (hi, lo) */
#else
    return _mm_set_epi64x((long long)hi, (long long)lo);
#endif
}
static inline uint64_t jps_v_lane(jps_v128 v, int i)
{
    if (i == 0)
    {
        return (uint64_t)_mm_cvtsi128_si64(v);
    }
    else
    {
        jps_v128 hi = _mm_srli_si128(v, 8); /* move high 64 to low 64 */
        return (uint64_t)_mm_cvtsi128_si64(hi);
    }
}
static inline jps_v128 jps_v_zero(void) { return _mm_setzero_si128(); }
/* 对齐加载一个 128 位单元（两个连续 uint64）。要求 p 16 字节对齐（由 GridMap 保证）。 */
static inline jps_v128 jps_v_load(const uint64_t *p) { return _mm_load_si128((const __m128i *)p); }
static inline jps_v128 jps_v_and(jps_v128 a, jps_v128 b) { return _mm_and_si128(a, b); }
static inline jps_v128 jps_v_or(jps_v128 a, jps_v128 b)  { return _mm_or_si128(a, b); }
static inline jps_v128 jps_v_not(jps_v128 a) { return _mm_xor_si128(a, _mm_set1_epi32(-1)); }
/* SSE4.1 可用时用 ptest（1 条指令），否则退回 movemask+cmpeq（SSE2 基线，2 条指令）。 */
static inline int jps_v_is_zero(jps_v128 v)
{
#if defined(__SSE4_1__)
    return _mm_testz_si128(v, v);
#else
    return _mm_movemask_epi8(_mm_cmpeq_epi8(v, _mm_setzero_si128())) == 0xFFFF;
#endif
}
/* 整 128 位左移 1，最低位补 cin：lo'=lo<<1|cin, hi'=hi<<1|(lo>>63)。 */
static inline jps_v128 jps_v_shl1(jps_v128 v, uint64_t cin)
{
    jps_v128 sh = _mm_slli_epi64(v, 1);                       /* 各 64 道独立左移 1 */
    uint64_t carry = jps_v_lane(v, 0) >> 63;                  /* 低字 bit63 → 高字 bit0 */
    return jps_v_or(sh, jps_v_set2(cin & 1ULL, carry));
}
/* 整 128 位右移 1，最高位补 cin_top：hi'=hi>>1|(cin<<63), lo'=lo>>1|(hi&1<<63)。 */
static inline jps_v128 jps_v_shr1(jps_v128 v, uint64_t cin_top)
{
    jps_v128 sh = _mm_srli_epi64(v, 1);
    uint64_t carry = jps_v_lane(v, 1) & 1ULL;                 /* 高字 bit0 → 低字 bit63 */
    return jps_v_or(sh, jps_v_set2(carry << 63, (cin_top & 1ULL) << 63));
}

/* ---- 16 位车道运算（供跳点缓存 dist 平面的 SIMD 回写：一次生成/写 8 个 int16 距离） ---- */
static inline jps_v128 jps_v_set1_i16(int16_t v) { return _mm_set1_epi16(v); }
static inline jps_v128 jps_v_setr_i16(int16_t a, int16_t b, int16_t c, int16_t d,
                                      int16_t e, int16_t f, int16_t g, int16_t h)
{
    return _mm_setr_epi16(a, b, c, d, e, f, g, h);
}
static inline jps_v128 jps_v_add_i16(jps_v128 a, jps_v128 b) { return _mm_add_epi16(a, b); }
/* 非对齐存 8×int16（回填起点列任意，不能要求 16 字节对齐）。 */
static inline void jps_v_storeu_i16(int16_t *p, jps_v128 v) { _mm_storeu_si128((__m128i *)p, v); }

#elif defined(__aarch64__) || defined(__ARM_NEON) || defined(__ARM_NEON__)

#  define JPS_HAVE_SIMD 1
#  define JPS_SIMD_NEON 1
#  include <arm_neon.h>

typedef uint64x2_t jps_v128;

static inline jps_v128 jps_v_set2(uint64_t lo, uint64_t hi)
{
    jps_v128 v = vdupq_n_u64(0);
    v = vsetq_lane_u64(lo, v, 0);
    v = vsetq_lane_u64(hi, v, 1);
    return v;
}
static inline uint64_t jps_v_lane(jps_v128 v, int i)
{
    return i == 0 ? vgetq_lane_u64(v, 0) : vgetq_lane_u64(v, 1);
}
static inline jps_v128 jps_v_zero(void) { return vdupq_n_u64(0); }
/* 加载一个 128 位单元（两个连续 uint64）。NEON 加载对齐不敏感，但数据仍按 16 字节对齐分配。 */
static inline jps_v128 jps_v_load(const uint64_t *p) { return vld1q_u64(p); }
static inline jps_v128 jps_v_and(jps_v128 a, jps_v128 b) { return vandq_u64(a, b); }
static inline jps_v128 jps_v_or(jps_v128 a, jps_v128 b)  { return vorrq_u64(a, b); }
static inline jps_v128 jps_v_not(jps_v128 a)
{
    return vreinterpretq_u64_u8(vmvnq_u8(vreinterpretq_u8_u64(a)));
}
static inline int jps_v_is_zero(jps_v128 v)
{
    return (vgetq_lane_u64(v, 0) | vgetq_lane_u64(v, 1)) == 0ULL;
}
static inline jps_v128 jps_v_shl1(jps_v128 v, uint64_t cin)
{
    jps_v128 sh = vshlq_n_u64(v, 1);
    uint64_t carry = jps_v_lane(v, 0) >> 63;
    return jps_v_or(sh, jps_v_set2(cin & 1ULL, carry));
}
static inline jps_v128 jps_v_shr1(jps_v128 v, uint64_t cin_top)
{
    jps_v128 sh = vshrq_n_u64(v, 1);
    uint64_t carry = jps_v_lane(v, 1) & 1ULL;
    return jps_v_or(sh, jps_v_set2(carry << 63, (cin_top & 1ULL) << 63));
}

/* ---- 16 位车道运算（供跳点缓存 dist 平面的 SIMD 回写） ---- */
static inline jps_v128 jps_v_set1_i16(int16_t v) { return vreinterpretq_u64_s16(vdupq_n_s16(v)); }
static inline jps_v128 jps_v_setr_i16(int16_t a, int16_t b, int16_t c, int16_t d,
                                      int16_t e, int16_t f, int16_t g, int16_t h)
{
    int16_t tmp[8]; tmp[0]=a; tmp[1]=b; tmp[2]=c; tmp[3]=d; tmp[4]=e; tmp[5]=f; tmp[6]=g; tmp[7]=h;
    return vreinterpretq_u64_s16(vld1q_s16(tmp));
}
static inline jps_v128 jps_v_add_i16(jps_v128 a, jps_v128 b)
{
    return vreinterpretq_u64_s16(vaddq_s16(vreinterpretq_s16_u64(a), vreinterpretq_s16_u64(b)));
}
static inline void jps_v_storeu_i16(int16_t *p, jps_v128 v) { vst1q_s16(p, vreinterpretq_s16_u64(v)); }

#else
#  error "JPS.Native requires a 128-bit SIMD backend (SSE2 or NEON)."
#endif  /* SIMD backend */

#endif /* JPS_SIMD_H */
