/*
 * min_heap.c
 * JPS Pathfinding — C port of JPS.Core/Pathfinding/MinHeap.cs
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

#include <stdlib.h>
#include "min_heap.h"

void jps_min_heap_init(jps_min_heap *h, int capacity)
{
    if (capacity < 1)
        capacity = 1;
    h->elem = (int *)malloc((size_t)capacity * sizeof(int));
    h->prio = (int64_t *)malloc((size_t)capacity * sizeof(int64_t));
    h->count = 0;
    h->capacity = capacity;
}

void jps_min_heap_free(jps_min_heap *h)
{
    free(h->elem);
    free(h->prio);
    h->elem = NULL;
    h->prio = NULL;
    h->count = 0;
    h->capacity = 0;
}

void jps_min_heap_clear(jps_min_heap *h)
{
    h->count = 0;
}

static void jps__heap_grow(jps_min_heap *h)
{
    int n = h->capacity * 2;
    h->elem = (int *)realloc(h->elem, (size_t)n * sizeof(int));
    h->prio = (int64_t *)realloc(h->prio, (size_t)n * sizeof(int64_t));
    h->capacity = n;
}

void jps_min_heap_enqueue(jps_min_heap *h, int element, int64_t priority)
{
    int i;

    if (h->count == h->capacity)
        jps__heap_grow(h);

    i = h->count++;

    /* hole sift-up（4-ary）：只搬父节点，最后写入新元素，避免每层 swap 两个数组。
     * 四叉：parent(i) = (i-1)/4，树高 ≈ log4(n)，比二叉减半 → sift-up 层数减半。 */
    while (i > 0)
    {
        int parent = (i - 1) >> 2;
        if (h->prio[parent] <= priority)
            break;
        h->elem[i] = h->elem[parent];
        h->prio[i] = h->prio[parent];
        i = parent;
    }
    h->elem[i] = element;
    h->prio[i] = priority;
}

static inline void jps__heap_sift_down(jps_min_heap *h, int i, int element, int64_t priority)
{
    /* 四叉 sift-down：四个孩子 4i+1..4i+4 在数组内连续（一条 cache line 基本全装），
     * 每层顺序扫出最小孩子。层数减半，代价是每层最多 3 次孩子间比较。 */
    while (1)
    {
        int base = (i << 2) + 1;   /* 第一个孩子 = 4i+1 */
        int limit, child, c;
        int64_t best;

        if (base >= h->count)
            break;

        child = base;
        best = h->prio[base];
        limit = base + 4;
        if (limit > h->count)
            limit = h->count;
        for (c = base + 1; c < limit; c++)
        {
            if (h->prio[c] < best)
            {
                best = h->prio[c];
                child = c;
            }
        }
        if (best >= priority)
            break;

        h->elem[i] = h->elem[child];
        h->prio[i] = h->prio[child];
        i = child;
    }
    h->elem[i] = element;
    h->prio[i] = priority;
}

bool jps_min_heap_try_dequeue(jps_min_heap *h, int *element, int64_t *priority)
{
    if (h->count == 0)
    {
        *element = 0;
        *priority = 0;
        return false;
    }

    *element = h->elem[0];
    *priority = h->prio[0];
    h->count--;

    if (h->count > 0)
    {
        jps__heap_sift_down(h, 0, h->elem[h->count], h->prio[h->count]);
    }

    return true;
}
