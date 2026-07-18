/*
 * adapter.c
 * JPS Pathfinding - native C adapter for immutable static padding and id-tracked dynamic rectangles.
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

#include <limits.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "jps.h"

typedef struct jps_adapter_obstacle
{
    int32_t id;
    uint32_t xy;
    uint32_t size;
} jps_adapter_obstacle;

_Static_assert(sizeof(jps_adapter_obstacle) == 12,
               "jps_adapter_obstacle must remain a 12-byte packed-value layout");

typedef struct jps_adapter_rect
{
    int left;
    int top;
    int right;
    int bottom;
} jps_adapter_rect;

/* Sorted by (y,x). Map sides are <=32767, so the compact native layout is 6 bytes/entry. */
typedef struct jps_adapter_coverage
{
    uint16_t x;
    uint16_t y;
    uint16_t ref_count;
} jps_adapter_coverage;

typedef struct jps_adapter_cursor
{
    jps_adapter_rect rect;
    int map_width;
    int x;
    int y;
    int started;
    uint32_t current;
} jps_adapter_cursor;

struct jps_adapter
{
    int width;
    int height;
    int obstacle_padding;
    size_t cell_count;
    size_t bit_word_count;
    uint64_t *static_blocked_bits;
    uint64_t *expanded_static_bits;
    jps_adapter_coverage *coverage;
    jps_adapter_coverage *coverage_scratch;
    size_t coverage_count;
    size_t coverage_capacity;
    size_t scratch_capacity;
    jps_adapter_obstacle *obstacles;
    int obstacle_count;
    int obstacle_capacity;
    jps_system *system;
};

static int jps__adapter_get_bit(const uint64_t *bits, size_t index)
{
    return (bits[index >> 6] & ((uint64_t)1u << (index & 63))) != 0;
}

static void jps__adapter_set_bit(uint64_t *bits, size_t index)
{
    bits[index >> 6] |= (uint64_t)1u << (index & 63);
}

static int jps__adapter_obstacle_x(const jps_adapter_obstacle *o)
{
    return (int)(int16_t)(o->xy & UINT32_C(0xffff));
}

static int jps__adapter_obstacle_y(const jps_adapter_obstacle *o)
{
    return (int)(int16_t)(o->xy >> 16);
}

static int jps__adapter_obstacle_width(const jps_adapter_obstacle *o)
{
    return (int)(uint16_t)(o->size & UINT32_C(0xffff));
}

static int jps__adapter_obstacle_height(const jps_adapter_obstacle *o)
{
    return (int)(uint16_t)(o->size >> 16);
}

static jps_adapter_obstacle jps__adapter_make_obstacle(int id, int x, int y,
                                                       int width, int height)
{
    jps_adapter_obstacle o;
    o.id = (int32_t)id;
    o.xy = (uint32_t)(uint16_t)(int16_t)x |
           ((uint32_t)(uint16_t)(int16_t)y << 16);
    o.size = (uint32_t)(uint16_t)width | ((uint32_t)(uint16_t)height << 16);
    return o;
}

static jps_adapter_rect jps__adapter_empty_rect(void)
{
    jps_adapter_rect r = { 0, 0, -1, -1 };
    return r;
}

static int jps__adapter_rect_empty(jps_adapter_rect r)
{
    return r.left > r.right || r.top > r.bottom;
}

static size_t jps__adapter_rect_area(jps_adapter_rect r)
{
    if (jps__adapter_rect_empty(r))
        return 0;
    return (size_t)(r.right - r.left + 1) * (size_t)(r.bottom - r.top + 1);
}

static jps_adapter_rect jps__adapter_expanded_rect(const jps_adapter *a,
                                                   int x, int y, int width, int height)
{
    int64_t p = a->obstacle_padding;
    int64_t left = (int64_t)x - p;
    int64_t top = (int64_t)y - p;
    int64_t right = (int64_t)x + (int64_t)width - 1 + p;
    int64_t bottom = (int64_t)y + (int64_t)height - 1 + p;
    jps_adapter_rect r;

    if (left < 0) left = 0;
    if (top < 0) top = 0;
    if (right >= a->width) right = a->width - 1;
    if (bottom >= a->height) bottom = a->height - 1;
    if (left > right || top > bottom)
        return jps__adapter_empty_rect();

    r.left = (int)left;
    r.top = (int)top;
    r.right = (int)right;
    r.bottom = (int)bottom;
    return r;
}

static void jps__adapter_cursor_init(jps_adapter_cursor *cursor,
                                     jps_adapter_rect rect, int map_width)
{
    cursor->rect = rect;
    cursor->map_width = map_width;
    cursor->x = rect.left;
    cursor->y = rect.top;
    cursor->started = 0;
    cursor->current = UINT32_MAX;
}

static int jps__adapter_cursor_next(jps_adapter_cursor *cursor)
{
    if (jps__adapter_rect_empty(cursor->rect))
        return 0;
    if (!cursor->started)
    {
        cursor->started = 1;
    }
    else if (cursor->x < cursor->rect.right)
    {
        cursor->x++;
    }
    else
    {
        cursor->x = cursor->rect.left;
        cursor->y++;
        if (cursor->y > cursor->rect.bottom)
            return 0;
    }
    cursor->current = (uint32_t)((size_t)cursor->y * (size_t)cursor->map_width +
                                 (size_t)cursor->x);
    return 1;
}

static int jps__adapter_ensure_scratch(jps_adapter *a, size_t required)
{
    size_t capacity;
    jps_adapter_coverage *next;
    if (a->scratch_capacity >= required)
        return 1;

    capacity = a->scratch_capacity == 0 ? 16 : a->scratch_capacity;
    while (capacity < required)
    {
        if (capacity > SIZE_MAX / 2)
        {
            capacity = required;
            break;
        }
        capacity *= 2;
    }
    if (capacity > SIZE_MAX / sizeof(jps_adapter_coverage))
        return 0;
    next = (jps_adapter_coverage *)realloc(
        a->coverage_scratch, capacity * sizeof(jps_adapter_coverage));
    if (next == NULL)
        return 0;
    a->coverage_scratch = next;
    a->scratch_capacity = capacity;
    return 1;
}

static uint32_t jps__adapter_coverage_index(const jps_adapter *a,
                                            const jps_adapter_coverage *coverage)
{
    return (uint32_t)coverage->y * (uint32_t)a->width + (uint32_t)coverage->x;
}

/*
 * Three-way row-major merge: current coverage, old footprint(-1), new footprint(+1).
 * A cell present in both old and new receives delta 0 before the effective map is touched.
 */
static int jps__adapter_apply_dynamic_change(jps_adapter *a,
                                             const jps_adapter_obstacle *old_obstacle,
                                             const jps_adapter_obstacle *new_obstacle)
{
    jps_adapter_rect old_rect = old_obstacle != NULL
        ? jps__adapter_expanded_rect(a, jps__adapter_obstacle_x(old_obstacle),
                                     jps__adapter_obstacle_y(old_obstacle),
                                     jps__adapter_obstacle_width(old_obstacle),
                                     jps__adapter_obstacle_height(old_obstacle))
        : jps__adapter_empty_rect();
    jps_adapter_rect new_rect = new_obstacle != NULL
        ? jps__adapter_expanded_rect(a, jps__adapter_obstacle_x(new_obstacle),
                                     jps__adapter_obstacle_y(new_obstacle),
                                     jps__adapter_obstacle_width(new_obstacle),
                                     jps__adapter_obstacle_height(new_obstacle))
        : jps__adapter_empty_rect();
    jps_adapter_cursor old_cursor;
    jps_adapter_cursor new_cursor;
    int has_old;
    int has_new;
    size_t input = 0;
    size_t output = 0;
    size_t maximum_output;

    if (jps__adapter_rect_area(new_rect) > SIZE_MAX - a->coverage_count)
        return 0;
    maximum_output = a->coverage_count + jps__adapter_rect_area(new_rect);
    if (!jps__adapter_ensure_scratch(a, maximum_output))
        return 0;

    jps__adapter_cursor_init(&old_cursor, old_rect, a->width);
    jps__adapter_cursor_init(&new_cursor, new_rect, a->width);
    has_old = jps__adapter_cursor_next(&old_cursor);
    has_new = jps__adapter_cursor_next(&new_cursor);

    while (input < a->coverage_count || has_old || has_new)
    {
        uint32_t key = input < a->coverage_count
            ? jps__adapter_coverage_index(a, &a->coverage[input])
            : UINT32_MAX;
        int before = 0;
        int delta = 0;
        int after;

        if (has_old && old_cursor.current < key) key = old_cursor.current;
        if (has_new && new_cursor.current < key) key = new_cursor.current;

        if (input < a->coverage_count &&
            jps__adapter_coverage_index(a, &a->coverage[input]) == key)
        {
            before = a->coverage[input].ref_count;
            input++;
        }
        if (has_old && old_cursor.current == key)
        {
            delta--;
            has_old = jps__adapter_cursor_next(&old_cursor);
        }
        if (has_new && new_cursor.current == key)
        {
            delta++;
            has_new = jps__adapter_cursor_next(&new_cursor);
        }

        after = before + delta;
        if (after < 0 || after > UINT16_MAX)
            return 0;

        if (before == 0 && after != 0)
            jps_system_set_blocked(a->system, (int)(key % (uint32_t)a->width),
                                   (int)(key / (uint32_t)a->width), 1);
        else if (before != 0 && after == 0 &&
                 !jps__adapter_get_bit(a->expanded_static_bits, key))
            jps_system_set_blocked(a->system, (int)(key % (uint32_t)a->width),
                                   (int)(key / (uint32_t)a->width), 0);

        if (after != 0)
        {
            a->coverage_scratch[output].x = (uint16_t)(key % (uint32_t)a->width);
            a->coverage_scratch[output].y = (uint16_t)(key / (uint32_t)a->width);
            a->coverage_scratch[output].ref_count = (uint16_t)after;
            output++;
        }
    }

    {
        jps_adapter_coverage *entries = a->coverage;
        size_t capacity = a->coverage_capacity;
        a->coverage = a->coverage_scratch;
        a->coverage_capacity = a->scratch_capacity;
        a->coverage_scratch = entries;
        a->scratch_capacity = capacity;
        a->coverage_count = output;
    }
    return 1;
}

static void jps__adapter_set_expanded_static(jps_adapter *a, int x, int y)
{
    size_t index = (size_t)y * (size_t)a->width + (size_t)x;
    if (jps__adapter_get_bit(a->expanded_static_bits, index))
        return;
    jps__adapter_set_bit(a->expanded_static_bits, index);
    jps_system_set_blocked(a->system, x, y, 1);
}

static void jps__adapter_apply_expanded_static(jps_adapter *a, int x, int y)
{
    jps_adapter_rect r = jps__adapter_expanded_rect(a, x, y, 1, 1);
    int xx, yy;
    for (yy = r.top; yy <= r.bottom; yy++)
        for (xx = r.left; xx <= r.right; xx++)
            jps__adapter_set_expanded_static(a, xx, yy);
}

static void jps__adapter_apply_boundary(jps_adapter *a)
{
    int x, y;
    int p = a->obstacle_padding;
    if (p == 0)
        return;
    for (y = 0; y < a->height; y++)
        for (x = 0; x < a->width; x++)
            if (x < p || x >= a->width - p || y < p || y >= a->height - p)
                jps__adapter_set_expanded_static(a, x, y);
}

static int jps__adapter_rebuild(jps_adapter *a)
{
    size_t index;
    int i;
    memset(a->expanded_static_bits, 0, a->bit_word_count * sizeof(uint64_t));
    jps_system_clear_all(a->system);
    jps__adapter_apply_boundary(a);

    for (index = 0; index < a->cell_count; index++)
        if (jps__adapter_get_bit(a->static_blocked_bits, index))
            jps__adapter_apply_expanded_static(a, (int)(index % (size_t)a->width),
                                               (int)(index / (size_t)a->width));

    a->coverage_count = 0;
    for (i = 0; i < a->obstacle_count; i++)
        if (!jps__adapter_apply_dynamic_change(a, NULL, &a->obstacles[i]))
            return 0;
    return 1;
}

static int jps__adapter_find_obstacle(const jps_adapter *a, int id)
{
    int i;
    for (i = 0; i < a->obstacle_count; i++)
        if (a->obstacles[i].id == id)
            return i;
    return -1;
}

static int jps__adapter_reserve_obstacle(jps_adapter *a)
{
    int new_capacity;
    jps_adapter_obstacle *next;
    if (a->obstacle_count < a->obstacle_capacity)
        return 1;
    new_capacity = a->obstacle_capacity == 0 ? 8 : a->obstacle_capacity * 2;
    if (new_capacity < a->obstacle_capacity ||
        (size_t)new_capacity > SIZE_MAX / sizeof(jps_adapter_obstacle))
        return 0;
    next = (jps_adapter_obstacle *)realloc(
        a->obstacles, (size_t)new_capacity * sizeof(jps_adapter_obstacle));
    if (next == NULL)
        return 0;
    a->obstacles = next;
    a->obstacle_capacity = new_capacity;
    return 1;
}

static jps_adapter *jps__adapter_create_core(int width, int height, int obstacle_padding,
                                             const uint8_t *cells, int count)
{
    jps_adapter *a;
    size_t cell_count;
    size_t i;
    if (width <= 0 || height <= 0 || width > 32767 || height > 32767 || obstacle_padding < 0)
        return NULL;
    cell_count = (size_t)width * (size_t)height;
    if (cells != NULL && (count < 0 || (size_t)count != cell_count))
        return NULL;

    a = (jps_adapter *)calloc(1, sizeof(jps_adapter));
    if (a == NULL)
        return NULL;
    a->width = width;
    a->height = height;
    a->obstacle_padding = obstacle_padding;
    a->cell_count = cell_count;
    a->bit_word_count = (cell_count + 63) >> 6;
    a->static_blocked_bits = (uint64_t *)calloc(a->bit_word_count, sizeof(uint64_t));
    a->expanded_static_bits = (uint64_t *)calloc(a->bit_word_count, sizeof(uint64_t));
    a->system = jps_system_create(width, height);
    if (a->static_blocked_bits == NULL || a->expanded_static_bits == NULL || a->system == NULL)
    {
        jps_adapter_destroy(a);
        return NULL;
    }

    if (cells != NULL)
        for (i = 0; i < cell_count; i++)
            if (cells[i] != 0)
                jps__adapter_set_bit(a->static_blocked_bits, i);

    if (!jps__adapter_rebuild(a))
    {
        jps_adapter_destroy(a);
        return NULL;
    }
    jps_system_sync(a->system);
    return a;
}

jps_adapter *jps_adapter_create(int width, int height, int obstacle_padding)
{
    return jps__adapter_create_core(width, height, obstacle_padding, NULL, 0);
}

jps_adapter *jps_adapter_create_from_buffer(int width, int height, int obstacle_padding,
                                             const uint8_t *cells, int count)
{
    if (cells == NULL)
        return NULL;
    return jps__adapter_create_core(width, height, obstacle_padding, cells, count);
}

void jps_adapter_destroy(jps_adapter *a)
{
    if (a == NULL)
        return;
    jps_system_destroy(a->system);
    free(a->obstacles);
    free(a->coverage_scratch);
    free(a->coverage);
    free(a->expanded_static_bits);
    free(a->static_blocked_bits);
    free(a);
}

int jps_adapter_width(const jps_adapter *a) { return a != NULL ? a->width : 0; }
int jps_adapter_height(const jps_adapter *a) { return a != NULL ? a->height : 0; }
int jps_adapter_obstacle_padding(const jps_adapter *a) { return a != NULL ? a->obstacle_padding : 0; }
int jps_adapter_dynamic_obstacle_count(const jps_adapter *a) { return a != NULL ? a->obstacle_count : 0; }
int jps_adapter_dynamic_covered_cell_count(const jps_adapter *a)
{
    return a != NULL ? (int)a->coverage_count : 0;
}

uint64_t jps_adapter_memory_bytes(const jps_adapter *a)
{
    if (a == NULL)
        return 0;
    return (uint64_t)sizeof(*a)
        + (uint64_t)a->bit_word_count * sizeof(uint64_t) * 2u
        + (uint64_t)a->coverage_capacity * sizeof(jps_adapter_coverage)
        + (uint64_t)a->scratch_capacity * sizeof(jps_adapter_coverage)
        + (uint64_t)(size_t)a->obstacle_capacity * sizeof(jps_adapter_obstacle)
        + jps_system_memory_bytes(a->system);
}

int jps_adapter_set_obstacle_padding(jps_adapter *a, int obstacle_padding)
{
    int old_padding;
    if (a == NULL)
        return JPS_ERR_NULL;
    if (obstacle_padding < 0)
        return JPS_ERR_INVALID_ARGUMENT;
    if (a->obstacle_padding == obstacle_padding)
        return 0;
    old_padding = a->obstacle_padding;
    a->obstacle_padding = obstacle_padding;
    if (!jps__adapter_rebuild(a))
    {
        a->obstacle_padding = old_padding;
        jps__adapter_rebuild(a);
        return JPS_ERR_OUT_OF_MEMORY;
    }
    return 1;
}

int jps_adapter_is_static_blocked(const jps_adapter *a, int x, int y)
{
    size_t index;
    if (a == NULL || x < 0 || x >= a->width || y < 0 || y >= a->height)
        return 0;
    index = (size_t)y * (size_t)a->width + (size_t)x;
    return jps__adapter_get_bit(a->static_blocked_bits, index);
}

int jps_adapter_is_blocked(const jps_adapter *a, int x, int y)
{
    if (a == NULL)
        return 1;
    return jps_system_is_blocked(a->system, x, y);
}

int jps_adapter_update_dynamic_obstacle(jps_adapter *a, int id,
                                        int x, int y, int width, int height)
{
    int index;
    int remove;
    jps_adapter_obstacle next;
    if (a == NULL)
        return JPS_ERR_NULL;
    remove = width == 0 && height == 0;
    if (!remove && (width <= 0 || height <= 0))
        return JPS_ERR_INVALID_ARGUMENT;
    if (!remove && (x < INT16_MIN || x > INT16_MAX ||
                    y < INT16_MIN || y > INT16_MAX ||
                    width > UINT16_MAX || height > UINT16_MAX))
        return JPS_ERR_INVALID_ARGUMENT;

    index = jps__adapter_find_obstacle(a, id);
    if (remove)
    {
        jps_adapter_obstacle old;
        if (index < 0)
            return 0;
        old = a->obstacles[index];
        if (!jps__adapter_apply_dynamic_change(a, &old, NULL))
            return JPS_ERR_OUT_OF_MEMORY;
        a->obstacle_count--;
        if (index != a->obstacle_count)
            a->obstacles[index] = a->obstacles[a->obstacle_count];
        return 1;
    }

    next = jps__adapter_make_obstacle(id, x, y, width, height);
    if (index >= 0)
    {
        jps_adapter_obstacle old = a->obstacles[index];
        if (old.xy == next.xy && old.size == next.size)
            return 0;
        if (!jps__adapter_apply_dynamic_change(a, &old, &next))
            return JPS_ERR_OUT_OF_MEMORY;
        a->obstacles[index] = next;
        return 1;
    }

    if (a->obstacle_count >= UINT16_MAX)
        return JPS_ERR_INVALID_ARGUMENT;
    if (!jps__adapter_reserve_obstacle(a))
        return JPS_ERR_OUT_OF_MEMORY;
    if (!jps__adapter_apply_dynamic_change(a, NULL, &next))
        return JPS_ERR_OUT_OF_MEMORY;
    a->obstacles[a->obstacle_count++] = next;
    return 1;
}

int jps_adapter_get_dynamic_obstacle(const jps_adapter *a, int id,
                                     int *out_x, int *out_y, int *out_width, int *out_height)
{
    int index;
    const jps_adapter_obstacle *o;
    if (a == NULL)
        return JPS_ERR_NULL;
    index = jps__adapter_find_obstacle(a, id);
    if (index < 0)
        return 0;
    o = &a->obstacles[index];
    if (out_x != NULL) *out_x = jps__adapter_obstacle_x(o);
    if (out_y != NULL) *out_y = jps__adapter_obstacle_y(o);
    if (out_width != NULL) *out_width = jps__adapter_obstacle_width(o);
    if (out_height != NULL) *out_height = jps__adapter_obstacle_height(o);
    return 1;
}

int jps_adapter_clear_dynamic_obstacles(jps_adapter *a)
{
    size_t i;
    if (a == NULL)
        return JPS_ERR_NULL;
    if (a->obstacle_count == 0)
        return 0;
    for (i = 0; i < a->coverage_count; i++)
    {
        uint32_t index = jps__adapter_coverage_index(a, &a->coverage[i]);
        if (!jps__adapter_get_bit(a->expanded_static_bits, index))
            jps_system_set_blocked(a->system, (int)(index % (uint32_t)a->width),
                                   (int)(index / (uint32_t)a->width), 0);
    }
    a->coverage_count = 0;
    a->obstacle_count = 0;
    return 1;
}

jps_system *jps_adapter_system(jps_adapter *a)
{
    return a != NULL ? a->system : NULL;
}

void jps_adapter_sync(jps_adapter *a)
{
    if (a != NULL)
        jps_system_sync(a->system);
}
