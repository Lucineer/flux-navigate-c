#include "navigate.h"
#include <string.h>

/* BFS queue: max grid cells */
#define Q_MAX (NAV_GRID_W * NAV_GRID_H)

/* --- internal BFS --- */

/* BFS returns 1 if path found (fills n->path, n->path_len), 0 if unreachable */
static int bfs(Navigator *n, NavPoint src, NavPoint dst) {
    /* visited grid */
    static uint8_t vis[NAV_GRID_H][NAV_GRID_W];
    memset(vis, 0, sizeof(vis));

    /* queue: store flat indices */
    static int qx[Q_MAX], qy[Q_MAX];
    /* parent: flat index of predecessor, -1 for start */
    static int par[NAV_GRID_H][NAV_GRID_W];
    memset(par, -1, sizeof(par));

    int head = 0, tail = 0;
    qx[tail] = src.x; qy[tail] = src.y;
    tail++;
    vis[src.y][src.x] = 1;
    par[src.y][src.x] = src.x + src.y * NAV_GRID_W; /* self-reference */

    static const int dx[] = {1, -1, 0, 0};
    static const int dy[] = {0, 0, 1, -1};

    int found = 0;
    while (head < tail) {
        int cx = qx[head], cy = qy[head];
        head++;
        if (cx == dst.x && cy == dst.y) { found = 1; break; }
        for (int d = 0; d < 4; d++) {
            int nx = cx + dx[d], ny = cy + dy[d];
            if (nx < 0 || nx >= NAV_GRID_W || ny < 0 || ny >= NAV_GRID_H) continue;
            if (vis[ny][nx] || n->grid[ny][nx]) continue;
            vis[ny][nx] = 1;
            par[ny][nx] = cx + cy * NAV_GRID_W;
            qx[tail] = nx; qy[tail] = ny;
            tail++;
        }
    }
    if (!found) { n->path_len = 0; return 0; }

    /* reconstruct path backwards */
    static NavPoint tmp[Q_MAX];
    int len = 0;
    int px = dst.x, py = dst.y;
    while (1) {
        tmp[len].x = px; tmp[len].y = py; len++;
        int p = par[py][px];
        if (p == px + py * NAV_GRID_W) break; /* reached start */
        px = p % NAV_GRID_W;
        py = p / NAV_GRID_W;
    }
    /* reverse into n->path (skip [0]=src, path starts from next step) */
    n->path_len = (uint16_t)(len - 1);
    for (int i = 0; i < (int)n->path_len; i++) {
        n->path[i] = tmp[len - 2 - i];
    }
    return 1;
}

/* --- public API --- */

void nav_init(Navigator *n) {
    memset(n, 0, sizeof(*n));
}

void nav_set_grid(Navigator *n, const uint8_t grid[NAV_GRID_H][NAV_GRID_W]) {
    memcpy(n->grid, grid, sizeof(n->grid));
}

int nav_set_destination(Navigator *n, int x, int y) {
    if (x < 0 || x >= NAV_GRID_W || y < 0 || y >= NAV_GRID_H) return -1;
    if (n->grid[y][x]) return -1;
    n->destination.x = x;
    n->destination.y = y;
    n->current_wp = 0;
    /* If waypoints exist, navigate to first waypoint; otherwise destination */
    NavPoint target;
    if (n->wp_count > 0) {
        target = n->waypoints[0];
    } else {
        target = n->destination;
    }
    int ok = bfs(n, n->position, target);
    n->navigating = ok ? 1 : 0;
    return ok ? 0 : -1;
}

int nav_step(Navigator *n) {
    if (!n->navigating || n->path_len == 0) {
        /* check if at destination */
        if (n->position.x == n->destination.x && n->position.y == n->destination.y)
            return 0;
        return -1;
    }
    NavPoint next = n->path[0];
    if (n->grid[next.y][next.x]) {
        /* forward cell blocked */
        return -1;
    }
    n->position = next;
    /* shift path */
    for (uint16_t i = 1; i < n->path_len; i++) {
        n->path[i - 1] = n->path[i];
    }
    n->path_len--;

    /* check if arrived at current waypoint / destination */
    NavPoint target = (n->current_wp < n->wp_count)
        ? n->waypoints[n->current_wp]
        : n->destination;

    if (n->position.x == target.x && n->position.y == target.y) {
        if (n->current_wp < n->wp_count) {
            n->current_wp++;
            /* route to next waypoint or destination */
            NavPoint next_target = (n->current_wp < n->wp_count)
                ? n->waypoints[n->current_wp]
                : n->destination;
            bfs(n, n->position, next_target);
            if (n->path_len == 0) n->navigating = 0;
        } else {
            /* arrived at final destination */
            n->navigating = 0;
            return 0;
        }
    }
    return 1;
}

int nav_replan(Navigator *n) {
    NavPoint target = (n->current_wp < n->wp_count)
        ? n->waypoints[n->current_wp]
        : n->destination;
    int ok = bfs(n, n->position, target);
    n->navigating = ok ? 1 : 0;
    return ok ? 0 : -1;
}

int nav_add_waypoint(Navigator *n, int x, int y) {
    if (n->wp_count >= WAYPOINTS_MAX) return -1;
    if (x < 0 || x >= NAV_GRID_W || y < 0 || y >= NAV_GRID_H) return -1;
    n->waypoints[n->wp_count].x = x;
    n->waypoints[n->wp_count].y = y;
    n->wp_count++;
    return 0;
}

int nav_clear_waypoints(Navigator *n) {
    n->wp_count = 0;
    n->current_wp = 0;
    return 0;
}

NavPoint nav_current(const Navigator *n) {
    return n->position;
}

NavPoint nav_next_waypoint(const Navigator *n) {
    if (n->current_wp < n->wp_count)
        return n->waypoints[n->current_wp];
    return n->destination;
}

int nav_at_destination(const Navigator *n) {
    return (n->position.x == n->destination.x && n->position.y == n->destination.y);
}

float nav_progress(const Navigator *n) {
    if (n->path_len == 0 && n->navigating == 0) return 1.0f;
    /* total steps = steps taken + steps remaining */
    /* We don't track original path length, so approximate from path_len */
    if (n->path_len == 0) return 1.0f;
    /* Use distance-based progress: 1 - (remaining / (remaining + taken_estimate)) */
    /* Since we don't store original length, just return 1 - 1/(path_len+1) as heuristic */
    /* Better: track total path length at planning time is not stored, so use 0.0 */
    return 0.0f;
}

int nav_blocked(const Navigator *n) {
    if (!n->navigating || n->path_len == 0) return 0;
    NavPoint next = n->path[0];
    return n->grid[next.y][next.x] ? 1 : 0;
}
