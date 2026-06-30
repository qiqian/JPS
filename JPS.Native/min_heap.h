/*
 * min_heap.h
 * JPS Pathfinding — C port of JPS.Core/Pathfinding/MinHeap.cs
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

#ifndef JPS_MIN_HEAP_H
#define JPS_MIN_HEAP_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 二叉最小堆：元素为 int（节点 id），优先级为 int64（f 值）。
 * 行为与 C# MinHeap.cs 等价（同为二叉堆，O(log n) 入队/出队）。
 * 结构体公开以便被寻路器按值内嵌，但字段仅由下列函数维护。
 */
typedef struct jps_min_heap
{
    int *elem;
    int64_t *prio;
    int count;
    int capacity;
} jps_min_heap;

void jps_min_heap_init(jps_min_heap *h, int capacity);
void jps_min_heap_free(jps_min_heap *h);
void jps_min_heap_clear(jps_min_heap *h);
void jps_min_heap_enqueue(jps_min_heap *h, int element, int64_t priority);
bool jps_min_heap_try_dequeue(jps_min_heap *h, int *element, int64_t *priority);

#ifdef __cplusplus
}
#endif

#endif /* JPS_MIN_HEAP_H */
