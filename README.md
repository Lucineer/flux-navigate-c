# flux-navigate-c

Pure C11 2D grid navigation and pathfinding library.

**Zero dependencies. No `malloc`.** Everything lives in stack/static memory.

## Features

- **BFS pathfinding** on a 32×32 grid
- **Waypoint routing** with intermediate stops
- **Dynamic replanning** when the path changes
- **Blocked-cell detection** for obstacle avoidance

## Quick Start

```c
#include "navigate.h"

Navigator nav;
nav_init(&nav);

uint8_t grid[NAV_GRID_H][NAV_GRID_W] = {0};
grid[2][3] = 1; // blocked cell
nav_set_grid(&nav, grid);

nav.position = (NavPoint){0, 0};
nav_set_destination(&nav, 10, 10);

while (nav_step(&nav) == 1) { /* move */ }
// arrived!
```

## API

| Function | Description |
|---|---|
| `nav_init` | Zero-initialize navigator |
| `nav_set_grid` | Set walkability grid (0=walk, 1=blocked) |
| `nav_set_destination` | Set target and compute path |
| `nav_step` | Advance one cell (returns 0=arrived, 1=moved, -1=blocked) |
| `nav_replan` | Recalculate path from current position |
| `nav_add_waypoint` | Add intermediate stop |
| `nav_clear_waypoints` | Remove all waypoints |
| `nav_current` | Get current position |
| `nav_next_waypoint` | Get next target (waypoint or destination) |
| `nav_at_destination` | Check if at final destination |
| `nav_progress` | Progress estimate (0.0–1.0) |
| `nav_blocked` | Check if forward cell is blocked |

## Build

```sh
make test
```

## License

MIT
