import heapq
import math

# 8-connected neighbors with costs
NEIGHBORS = [
    ( 1,  0, 1.0),
    (-1,  0, 1.0),
    ( 0,  1, 1.0),
    ( 0, -1, 1.0),
    ( 1,  1, 1.4142),
    ( 1, -1, 1.4142),
    (-1,  1, 1.4142),
    (-1, -1, 1.4142),
]

def heuristic(a, b):
    """Euclidean distance heuristic."""
    return math.hypot(b[0] - a[0], b[1] - a[1])

def astar(grid, start, goal):
    """
    A* on occupancy grid.
    start, goal: (col, row) tuples
    grid: 2D numpy array, 1=obstacle, 0=free
    Returns list of (col, row) cells or empty list.
    """
    rows, cols = grid.shape

    if start == goal:
        return [start]

    g_score   = {start: 0.0}
    came_from = {start: None}
    open_heap = [(heuristic(start, goal), heuristic(start, goal), start)]

    while open_heap:
        f_cur, _, current = heapq.heappop(open_heap)

        if current == goal:
            return reconstruct(came_from, goal)

        if f_cur > g_score.get(current, float("inf")) + heuristic(current, goal):
            continue

        cx, cy = current
        for dx, dy, move_cost in NEIGHBORS:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < cols and 0 <= ny < rows):
                continue
            if grid[ny, nx] == 1:
                continue
            neighbor = (nx, ny)
            tentative_g = g_score[current] + move_cost
            if tentative_g < g_score.get(neighbor, float("inf")):
                g_score[neighbor]   = tentative_g
                came_from[neighbor] = current
                h = heuristic(neighbor, goal)
                heapq.heappush(open_heap, (tentative_g + h, h, neighbor))

    return []

def reconstruct(came_from, goal):
    path, current = [], goal
    while current is not None:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path
