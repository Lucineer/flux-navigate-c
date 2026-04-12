#include "navigate.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static int tests_run = 0, tests_passed = 0;

#define ASSERT(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; } \
    else { printf("FAIL: %s (line %d)\n", msg, __LINE__); } \
} while(0)

static uint8_t empty_grid[NAV_GRID_H][NAV_GRID_W];

int main(void) {
    memset(empty_grid, 0, sizeof(empty_grid));

    /* 1. nav_init zeroes everything */
    {
        Navigator n;
        nav_init(&n);
        ASSERT(n.position.x == 0 && n.position.y == 0, "init position");
        ASSERT(n.wp_count == 0, "init wp_count");
        ASSERT(n.path_len == 0, "init path_len");
        ASSERT(n.navigating == 0, "init navigating");
    }

    /* 2. nav_set_grid copies grid */
    {
        Navigator n;
        nav_init(&n);
        uint8_t g[NAV_GRID_H][NAV_GRID_W];
        memset(g, 0, sizeof(g));
        g[5][10] = 1;
        nav_set_grid(&n, g);
        ASSERT(n.grid[5][10] == 1, "set_grid copies");
        ASSERT(n.grid[0][0] == 0, "set_grid zeroes");
    }

    /* 3. open grid pathfinding */
    {
        Navigator n;
        nav_init(&n);
        nav_set_grid(&n, empty_grid);
        n.position.x = 0; n.position.y = 0;
        int r = nav_set_destination(&n, 5, 5);
        ASSERT(r == 0, "open grid dest ok");
        ASSERT(n.navigating == 1, "open grid navigating");
        ASSERT(n.path_len > 0, "open grid has path");
    }

    /* 4. step advances position */
    {
        Navigator n;
        nav_init(&n);
        nav_set_grid(&n, empty_grid);
        n.position.x = 0; n.position.y = 0;
        nav_set_destination(&n, 3, 0);
        int r = nav_step(&n);
        ASSERT(r == 1, "step moved");
        ASSERT(n.position.x == 1 && n.position.y == 0, "step position");
    }

    /* 5. step returns 0 when arrived */
    {
        Navigator n;
        nav_init(&n);
        nav_set_grid(&n, empty_grid);
        n.position.x = 0; n.position.y = 0;
        nav_set_destination(&n, 1, 0);
        nav_step(&n); /* move to (1,0) */
        int r = nav_step(&n);
        ASSERT(r == 0, "arrived at dest");
        ASSERT(nav_at_destination(&n), "at_destination");
    }

    /* 6. blocked destination returns -1 */
    {
        Navigator n;
        nav_init(&n);
        uint8_t g[NAV_GRID_H][NAV_GRID_W];
        memset(g, 0, sizeof(g));
        g[2][2] = 1; /* block destination */
        nav_set_grid(&n, g);
        n.position.x = 0; n.position.y = 0;
        int r = nav_set_destination(&n, 2, 2);
        ASSERT(r == -1, "blocked dest fails");
    }

    /* 7. out of bounds destination */
    {
        Navigator n;
        nav_init(&n);
        n.position.x = 0; n.position.y = 0;
        int r = nav_set_destination(&n, -1, 0);
        ASSERT(r == -1, "oob dest x");
        r = nav_set_destination(&n, 0, NAV_GRID_H);
        ASSERT(r == -1, "oob dest y");
    }

    /* 8. obstacle avoidance in path */
    {
        Navigator n;
        nav_init(&n);
        uint8_t g[NAV_GRID_H][NAV_GRID_W];
        memset(g, 0, sizeof(g));
        g[0][1] = 1; /* block direct path */
        nav_set_grid(&n, g);
        n.position.x = 0; n.position.y = 0;
        int r = nav_set_destination(&n, 2, 0);
        ASSERT(r == 0, "obstacle path found");
        ASSERT(n.path_len >= 2, "obstacle path longer (goes around)");
    }

    /* 9. nav_add_waypoint and nav_clear_waypoints */
    {
        Navigator n;
        nav_init(&n);
        ASSERT(nav_add_waypoint(&n, 3, 3) == 0, "add wp");
        ASSERT(n.wp_count == 1, "wp_count 1");
        ASSERT(nav_add_waypoint(&n, 5, 5) == 0, "add wp2");
        ASSERT(n.wp_count == 2, "wp_count 2");
        nav_clear_waypoints(&n);
        ASSERT(n.wp_count == 0, "wp cleared");
    }

    /* 10. waypoint navigation routes through intermediate */
    {
        Navigator n;
        nav_init(&n);
        nav_set_grid(&n, empty_grid);
        n.position.x = 0; n.position.y = 0;
        nav_add_waypoint(&n, 5, 0);
        nav_set_destination(&n, 5, 5);
        /* first path should go to waypoint (5,0) */
        ASSERT(n.navigating == 1, "wp navigating");
        NavPoint nw = nav_next_waypoint(&n);
        ASSERT(nw.x == 5 && nw.y == 0, "next waypoint");
    }

    /* 11. nav_current */
    {
        Navigator n;
        nav_init(&n);
        n.position.x = 7; n.position.y = 3;
        NavPoint c = nav_current(&n);
        ASSERT(c.x == 7 && c.y == 3, "nav_current");
    }

    /* 12. nav_replan */
    {
        Navigator n;
        nav_init(&n);
        nav_set_grid(&n, empty_grid);
        n.position.x = 0; n.position.y = 0;
        nav_set_destination(&n, 5, 5);
        n.position.x = 2; n.position.y = 2; /* teleport mid-path */
        int r = nav_replan(&n);
        ASSERT(r == 0, "replan ok");
        ASSERT(n.path_len > 0, "replan has path");
    }

    /* 13. nav_blocked */
    {
        Navigator n;
        nav_init(&n);
        nav_set_grid(&n, empty_grid);
        n.position.x = 0; n.position.y = 0;
        nav_set_destination(&n, 5, 0);
        ASSERT(nav_blocked(&n) == 0, "not blocked open");
        /* block next cell */
        n.grid[0][1] = 1;
        ASSERT(nav_blocked(&n) == 1, "blocked forward");
    }

    /* 14. nav_at_destination */
    {
        Navigator n;
        nav_init(&n);
        n.position.x = 4; n.position.y = 4;
        n.destination.x = 4; n.destination.y = 4;
        ASSERT(nav_at_destination(&n) == 1, "at dest same pos");
    }

    /* 15. waypoint overflow */
    {
        Navigator n;
        nav_init(&n);
        for (int i = 0; i < WAYPOINTS_MAX; i++)
            ASSERT(nav_add_waypoint(&n, i, 0) == 0, "wp within limit");
        ASSERT(nav_add_waypoint(&n, 0, 1) == -1, "wp overflow");
    }

    /* 16. fully enclosed start - no path */
    {
        Navigator n;
        nav_init(&n);
        uint8_t g[NAV_GRID_H][NAV_GRID_W];
        memset(g, 1, sizeof(g)); /* all blocked */
        g[0][0] = 0; /* only start walkable */
        nav_set_grid(&n, g);
        n.position.x = 0; n.position.y = 0;
        int r = nav_set_destination(&n, 1, 1);
        ASSERT(r == -1, "no path enclosed");
    }

    /* 17. step on blocked forward returns -1 */
    {
        Navigator n;
        nav_init(&n);
        nav_set_grid(&n, empty_grid);
        n.position.x = 0; n.position.y = 0;
        nav_set_destination(&n, 5, 0);
        n.grid[0][1] = 1; /* block next step */
        int r = nav_step(&n);
        ASSERT(r == -1, "step blocked returns -1");
    }

    printf("\n%d/%d tests passed\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
