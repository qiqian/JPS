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

    /* hole sift-up：只搬父节点，最后写入新元素，避免每层 swap 两个数组。 */
    while (i > 0)
    {
        int parent = (i - 1) >> 1;
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
    while (1)
    {
        int l = (i << 1) + 1;
        int r = l + 1;
        int child;

        if (l >= h->count)
            break;
        child = l;
        if (r < h->count && h->prio[r] < h->prio[l])
            child = r;
        if (h->prio[child] >= priority)
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
