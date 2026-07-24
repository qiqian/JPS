/* Native C tests for jps_adapter. Kept dependency-free so the stress target can run them in CI. */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "jps.h"

#define CHECK(expr) do { if (!(expr)) { \
    fprintf(stderr, "adapter test failed at %s:%d: %s\n", __FILE__, __LINE__, #expr); \
    return 1; \
} } while (0)

typedef struct test_obstacle
{
    int active;
    int x;
    int y;
    int width;
    int height;
} test_obstacle;

static uint32_t test_rng = 20260717u;
static uint32_t test_random(void)
{
    test_rng = test_rng * 1664525u + 1013904223u;
    return test_rng;
}

static int test_range(int min_value, int max_exclusive)
{
    return min_value + (int)(test_random() % (uint32_t)(max_exclusive - min_value));
}

static int reference_blocked(int x, int y, int width, int height, int padding,
                             const uint8_t *static_cells,
                             const test_obstacle *obstacles, int obstacle_count)
{
    int sx, sy, i;
    if (x < padding || x >= width - padding || y < padding || y >= height - padding)
        return 1;

    for (sy = 0; sy < height; sy++)
        for (sx = 0; sx < width; sx++)
            if (static_cells[sy * width + sx] != 0 &&
                x >= sx - padding && x <= sx + padding &&
                y >= sy - padding && y <= sy + padding)
                return 1;

    for (i = 0; i < obstacle_count; i++)
        if (obstacles[i].active &&
            x >= obstacles[i].x - padding &&
            x <= obstacles[i].x + obstacles[i].width - 1 + padding &&
            y >= obstacles[i].y - padding &&
            y <= obstacles[i].y + obstacles[i].height - 1 + padding)
            return 1;
    return 0;
}

static int test_static_padding_and_boundary(void)
{
    uint8_t cells[9 * 9] = {0};
    jps_adapter *a;
    int x, y;
    cells[4 * 9 + 4] = 1;
    a = jps_adapter_create_from_buffer(9, 9, 1, cells, 81);
    CHECK(a != NULL);
    cells[4 * 9 + 4] = 0; /* adapter owns an immutable bit snapshot */
    cells[6 * 9 + 6] = 1;
    CHECK(jps_adapter_width(a) == 9 && jps_adapter_height(a) == 9);
    CHECK(jps_adapter_obstacle_padding(a) == 1);

    for (y = 3; y <= 5; y++)
        for (x = 3; x <= 5; x++)
            CHECK(jps_adapter_is_blocked(a, x, y) == 1);
    for (x = 0; x < 9; x++)
    {
        CHECK(jps_adapter_is_blocked(a, x, 0) == 1);
        CHECK(jps_adapter_is_blocked(a, x, 8) == 1);
    }
    CHECK(jps_adapter_is_blocked(a, 2, 2) == 0);
    CHECK(jps_adapter_is_static_blocked(a, 4, 4) == 1);
    CHECK(jps_adapter_is_static_blocked(a, 6, 6) == 0);
    CHECK(jps_adapter_memory_bytes(a) > 0);
    jps_adapter_destroy(a);
    return 0;
}

static int test_dynamic_move_overlap_and_remove(void)
{
    uint8_t cells[12 * 10] = {0};
    jps_adapter *a;
    int x, y, w, h;
    cells[4 * 12 + 4] = 1;
    a = jps_adapter_create_from_buffer(12, 10, 1, cells, 120);
    CHECK(a != NULL);
    CHECK(jps_adapter_update_dynamic_obstacle(a, 7, 3, 3, 2, 2) == 1);
    CHECK(jps_adapter_update_dynamic_obstacle(a, 8, 5, 4, 1, 1) == 1);
    CHECK(jps_adapter_dynamic_obstacle_count(a) == 2);
    CHECK(jps_adapter_get_dynamic_obstacle(a, 7, &x, &y, &w, &h) == 1);
    CHECK(x == 3 && y == 3 && w == 2 && h == 2);

    CHECK(jps_adapter_update_dynamic_obstacle(a, 7, 4, 3, 2, 2) == 1);
    CHECK(jps_adapter_is_blocked(a, 2, 3) == 0);
    CHECK(jps_adapter_is_blocked(a, 3, 3) == 1);
    CHECK(jps_adapter_is_blocked(a, 6, 4) == 1);

    CHECK(jps_adapter_update_dynamic_obstacle(a, 7, 0, 0, 0, 0) == 1);
    CHECK(jps_adapter_is_blocked(a, 5, 4) == 1); /* id 8 still covers it */
    CHECK(jps_adapter_update_dynamic_obstacle(a, 8, 0, 0, 0, 0) == 1);
    CHECK(jps_adapter_is_blocked(a, 5, 4) == 1); /* immutable static padding remains */
    CHECK(jps_adapter_is_blocked(a, 6, 4) == 0);
    CHECK(jps_adapter_update_dynamic_obstacle(a, 8, 0, 0, 0, 0) == 0);
    CHECK(jps_adapter_update_dynamic_obstacle(a, 9, 7, 7, 1, 1) == 1);
    CHECK(jps_adapter_clear_dynamic_obstacles(a) == 1);
    CHECK(jps_adapter_dynamic_covered_cell_count(a) == 0);
    CHECK(jps_adapter_is_blocked(a, 5, 4) == 1);
    CHECK(jps_adapter_is_blocked(a, 7, 7) == 0);
    CHECK(jps_adapter_update_dynamic_obstacle(a, 1, 1, 1, 0, 2) == JPS_ERR_INVALID_ARGUMENT);
    CHECK(jps_adapter_update_dynamic_obstacle(a, 1, INT16_MIN - 1, 1, 1, 1) == JPS_ERR_INVALID_ARGUMENT);
    CHECK(jps_adapter_update_dynamic_obstacle(a, 1, 1, INT16_MAX + 1, 1, 1) == JPS_ERR_INVALID_ARGUMENT);
    CHECK(jps_adapter_update_dynamic_obstacle(a, 1, 1, 1, UINT16_MAX + 1, 1) == JPS_ERR_INVALID_ARGUMENT);
    CHECK(jps_adapter_update_dynamic_obstacle(a, 1, 1, 1, 1, UINT16_MAX + 1) == JPS_ERR_INVALID_ARGUMENT);
    CHECK(jps_adapter_update_dynamic_obstacle(
        a, 42, INT16_MIN, INT16_MAX, UINT16_MAX, UINT16_MAX) == 1);
    CHECK(jps_adapter_get_dynamic_obstacle(a, 42, &x, &y, &w, &h) == 1);
    CHECK(x == INT16_MIN && y == INT16_MAX && w == UINT16_MAX && h == UINT16_MAX);
    jps_adapter_destroy(a);
    return 0;
}

static int test_padding_rebuild_and_find(void)
{
    uint8_t cells[9 * 7] = {0};
    jps_adapter *a;
    jps_pathfinder *pf;
    int y, n, xy[32];
    for (y = 1; y <= 5; y++)
        if (y != 3)
            cells[y * 9 + 4] = 1;
    a = jps_adapter_create_from_buffer(9, 7, 0, cells, 63);
    CHECK(a != NULL);
    pf = jps_pathfinder_create();
    CHECK(pf != NULL);
    jps_adapter_sync(a);
    n = jps_pathfinder_find_path(pf, jps_adapter_system(a), 2, 3, 6, 3);
    CHECK(n > 0 && n <= 16);
    CHECK(jps_pathfinder_path_count(pf) == n);
    CHECK(jps_pathfinder_copy_path(pf, xy, 16) == n);
    CHECK(xy[0] == 2 && xy[1] == 3);
    CHECK(xy[(n - 1) * 2] == 6 && xy[(n - 1) * 2 + 1] == 3);
    CHECK(jps_adapter_set_obstacle_padding(a, 1) == 1);
    jps_adapter_sync(a);
    CHECK(jps_pathfinder_find_path(pf, jps_adapter_system(a), 2, 3, 6, 3) == JPS_ERR_NO_PATH);
    CHECK(jps_adapter_system(a) != NULL);
    jps_pathfinder_destroy(pf);
    jps_adapter_destroy(a);
    return 0;
}

static int test_nearest_refine_packed_path(void)
{
    enum { WIDTH = 9, HEIGHT = 7 };
    uint8_t cells[WIDTH * HEIGHT] = {0};
    jps_adapter *a;
    jps_pathfinder *pf;
    int y, n, sn, xy[32];
    float sxy[32];

    for (y = 0; y < HEIGHT; y++)
        cells[y * WIDTH + 4] = 1;   /* Solid wall: nearest must stop immediately before it at x=3. */

    a = jps_adapter_create_from_buffer(WIDTH, HEIGHT, 0, cells, WIDTH * HEIGHT);
    CHECK(a != NULL);
    pf = jps_pathfinder_create();
    CHECK(pf != NULL);
    jps_adapter_sync(a);

    CHECK(jps_pathfinder_find_path(pf, jps_adapter_system(a), 1, 3, 7, 3) == JPS_ERR_NO_PATH);
    n = jps_pathfinder_find_path_nearest(pf, jps_adapter_system(a), 1, 3, 7, 3);
    CHECK(n >= 2 && n <= 16);
    CHECK(jps_pathfinder_reached_goal(pf) == 0);
    CHECK(jps_pathfinder_copy_path(pf, xy, 16) == n);
    CHECK(xy[0] == 1 && xy[1] == 3);
    CHECK(xy[(n - 1) * 2] == 3 && xy[(n - 1) * 2 + 1] == 3);

    sn = jps_pathfinder_smoothed_path_count(pf);
    CHECK(sn >= 2 && sn <= 16);
    CHECK(jps_pathfinder_copy_smoothed_path(pf, sxy, 16) == sn);
    CHECK(sxy[0] == 1.5f && sxy[1] == 3.5f);
    CHECK(sxy[(sn - 1) * 2] == 3.5f && sxy[(sn - 1) * 2 + 1] == 3.5f);

    jps_pathfinder_destroy(pf);
    jps_adapter_destroy(a);
    return 0;
}

static int test_random_updates_against_reference(void)
{
    enum { WIDTH = 16, HEIGHT = 13, IDS = 6 };
    uint8_t static_cells[WIDTH * HEIGHT] = {0};
    test_obstacle obstacles[IDS];
    jps_adapter *a;
    int padding = 0;
    int step, x, y;
    memset(obstacles, 0, sizeof(obstacles));

    for (y = 0; y < HEIGHT; y++)
        for (x = 0; x < WIDTH; x++)
            if (test_range(0, 8) == 0)
                static_cells[y * WIDTH + x] = 1;

    a = jps_adapter_create_from_buffer(
        WIDTH, HEIGHT, 0, static_cells, WIDTH * HEIGHT);
    CHECK(a != NULL);

    for (step = 0; step < 300; step++)
    {
        int action = test_range(0, 3);
        if (action == 0)
        {
            int id = test_range(0, IDS);
            test_obstacle *o = &obstacles[id];
            o->active = 1;
            o->x = test_range(-3, WIDTH + 3);
            o->y = test_range(-3, HEIGHT + 3);
            o->width = test_range(1, 5);
            o->height = test_range(1, 5);
            CHECK(jps_adapter_update_dynamic_obstacle(a, id, o->x, o->y,
                                                      o->width, o->height) >= 0);
        }
        else if (action == 1)
        {
            int id = test_range(0, IDS);
            CHECK(jps_adapter_update_dynamic_obstacle(a, id, 0, 0, 0, 0) >= 0);
            obstacles[id].active = 0;
        }
        else
        {
            padding = test_range(0, 4);
            CHECK(jps_adapter_set_obstacle_padding(a, padding) >= 0);
        }

        for (y = 0; y < HEIGHT; y++)
            for (x = 0; x < WIDTH; x++)
                CHECK(jps_adapter_is_blocked(a, x, y) ==
                      reference_blocked(x, y, WIDTH, HEIGHT, padding,
                                        static_cells, obstacles, IDS));
    }

    CHECK(jps_adapter_clear_dynamic_obstacles(a) >= 0);
    jps_adapter_destroy(a);
    return 0;
}

int jps_adapter_run_tests(void)
{
    if (test_static_padding_and_boundary() != 0) return 1;
    if (test_dynamic_move_overlap_and_remove() != 0) return 1;
    if (test_padding_rebuild_and_find() != 0) return 1;
    if (test_nearest_refine_packed_path() != 0) return 1;
    if (test_random_updates_against_reference() != 0) return 1;
    printf("jps_adapter native tests: passed\n");
    return 0;
}
