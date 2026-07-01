/*
 * jps_atomic.h
 * JPS Pathfinding — 跳点缓存世代戳(uint8)的跨平台 acquire/release 读写（始终启用，无单线程模式）。
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 *
 * 与 C# 的并发缓存模型严格一致：
 *   写者：先**普通写** dist[dir]，再 **release-store** gen[dir] = valid_gen；
 *   读者：**acquire-load** gen[dir]，若 == valid_gen 再**普通读** dist[dir]。
 * release/acquire 配对保证“看到某格 clean 世代戳”即可见其 dist 写入，于是多个 jps_pathfinder
 * 可在并行寻路中只读 / 惰性补写**同一份共享缓存**（前提：并行前单线程 Sync 一次、并行期间地图不变；
 * 补写值是固定地图的纯函数，重复计算结果一致）。读热路径只需半屏障(acquire)，比全屏障更省。
 *
 * 跨平台：
 *   · GCC/Clang（x64 / iOS / Android arm64 / Linux / macOS）→ __atomic 内建，ARM 上落为 ldar/stlr；
 *   · MSVC（本工程仅 x64）→ x86-TSO 下对齐字节读写本就具 acquire/release，辅以编译器屏障禁止重排。
 */

#ifndef JPS_ATOMIC_H
#define JPS_ATOMIC_H

#include <stdint.h>

#if defined(__GNUC__) || defined(__clang__)

static inline uint8_t jps_gen_load_acquire(const uint8_t *p)
{
    return __atomic_load_n(p, __ATOMIC_ACQUIRE);
}
static inline void jps_gen_store_release(uint8_t *p, uint8_t v)
{
    __atomic_store_n(p, v, __ATOMIC_RELEASE);
}

#elif defined(_MSC_VER)

#  if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L && !defined(__STDC_NO_ATOMICS__)
/* MSVC 启用 C11 原子（/std:c11 起）：用标准、非弃用的 atomic_thread_fence。 */
#    include <stdatomic.h>
static inline uint8_t jps_gen_load_acquire(const uint8_t *p)
{
    uint8_t v = *(const volatile uint8_t *)p;   /* volatile 防编译器省略/复制该读 */
    atomic_thread_fence(memory_order_acquire);  /* x64 上编译成编译器屏障，配合 TSO 即 acquire */
    return v;
}
static inline void jps_gen_store_release(uint8_t *p, uint8_t v)
{
    atomic_thread_fence(memory_order_release);
    *(volatile uint8_t *)p = v;
}
#  else
/* 回退（旧 MSVC / 未启用 C11 原子）：编译器屏障 + volatile 内建，x64 TSO 提供硬件 acquire/release。 */
#    include <intrin.h>
static inline uint8_t jps_gen_load_acquire(const uint8_t *p)
{
    uint8_t v = (uint8_t)__iso_volatile_load8((const volatile __int8 *)p);
__pragma(warning(suppress : 4996))
    _ReadWriteBarrier();
    return v;
}
static inline void jps_gen_store_release(uint8_t *p, uint8_t v)
{
__pragma(warning(suppress : 4996))
    _ReadWriteBarrier();
    __iso_volatile_store8((volatile __int8 *)p, (char)v);
}
#  endif

#else
#  error "JPS: unsupported compiler for acquire/release atomics (need GCC/Clang or MSVC)"
#endif

#endif /* JPS_ATOMIC_H */
