# MAINTENANCE.md

## Architecture

- **navigate.c**: Single-file implementation. BFS uses static buffers (no malloc).
- **Grid**: 32×32 fixed size (`NAV_GRID_W`, `NAV_GRID_H`).
- **Path storage**: `NavPoint path[1024]` inside the `Navigator` struct (~8KB).
- **BFS internals**: `vis[]`, `par[][]`, `qx[]`, `qy[]`, `tmp[]` are all `static` local — thread-unsafe, zero-alloc.

## Known Limitations

1. **Not thread-safe** — BFS uses static locals. Use one Navigator per thread or add a mutex.
2. **Grid size is fixed** at compile time. To change, update `NAV_GRID_W`/`NAV_GRID_H` and recompile.
3. **`nav_progress()`** is approximate — doesn't store original path length, so returns 0.0 mid-path and 1.0 at destination.
4. **Waypoint validation** — waypoints aren't checked for walkability when added; invalid waypoints cause routing failure at planning time.
5. **No diagonal movement** — BFS uses 4-directional (cardinal) moves only.

## Testing

```sh
make test
```

17 tests covering: init, grid, pathfinding, stepping, arrival, blocked cells, waypoints, replanning, boundary conditions.

## Adding Features

- **Diagonal movement**: Add `{1,1},{-1,1},{1,-1},{-1,-1}` to `dx[]`/`dy[]` in BFS.
- **A\* instead of BFS**: Replace BFS loop; same interface.
- **Larger grids**: Increase defines and ensure static buffer sizes (`Q_MAX`) scale.
- **Thread safety**: Move static locals into the Navigator struct or pass context.
