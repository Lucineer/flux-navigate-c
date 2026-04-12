#ifndef NAVIGATE_H
#define NAVIGATE_H
#include <stdint.h>
#include <stddef.h>

#define NAV_GRID_W 32
#define NAV_GRID_H 32
#define WAYPOINTS_MAX 32

typedef struct { int x, y; } NavPoint;

typedef struct {
    uint8_t grid[NAV_GRID_H][NAV_GRID_W]; // 0=walkable, 1=blocked
    NavPoint waypoints[WAYPOINTS_MAX];
    uint8_t wp_count;
    NavPoint position;
    NavPoint destination;
    uint8_t current_wp;
    NavPoint path[NAV_GRID_W * NAV_GRID_H]; // planned path
    uint16_t path_len;
    uint8_t navigating; // actively following path
} Navigator;

void nav_init(Navigator *n);
void nav_set_grid(Navigator *n, const uint8_t grid[NAV_GRID_H][NAV_GRID_W]);
int nav_set_destination(Navigator *n, int x, int y);
int nav_step(Navigator *n);
int nav_replan(Navigator *n);
int nav_add_waypoint(Navigator *n, int x, int y);
int nav_clear_waypoints(Navigator *n);
NavPoint nav_current(const Navigator *n);
NavPoint nav_next_waypoint(const Navigator *n);
int nav_at_destination(const Navigator *n);
float nav_progress(const Navigator *n);
int nav_blocked(const Navigator *n);

#endif
