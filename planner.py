import sys
from collections import deque
import heapq

def parse_world(world_file):
    with open(world_file, 'r') as f:
        cols = int(f.readline())
        rows = int(f.readline())
        grid = [list(f.readline().strip()) for _ in range(rows)]

    robot_pos = None
    dirty_cells = set()
    walls = set()

    for r in range(rows):
        for c in range(cols):
            cell = grid[r][c]
            if cell == '@':
                robot_pos = (r, c)
            elif cell == '*':
                dirty_cells.add((r, c))
            elif cell == '#':
                walls.add((r, c))
    
    return {
        "rows": rows,
        "cols": cols,
        "robot_pos": robot_pos,
        "dirty_cells": dirty_cells,
        "walls": walls,
        "grid": grid
    }


def dfs(start_state):
    # Initialize variables
    rows, cols = start_state["rows"], start_state["cols"]
    walls = start_state["walls"]
    robot_pos = start_state["robot_pos"]
    dirty_cells = start_state["dirty_cells"]
    
    initial_state = (robot_pos, frozenset(dirty_cells))
    stack = deque()
    visited = set()

    stack.append((initial_state, []))
    visited.add(initial_state)

    nodes_generated = 1
    nodes_expanded = 0

    moves = {
        'N': (-1, 0),
        'S': (1, 0),
        'E': (0, 1),
        'W': (0, -1)
    }

    # Iterate until stack is empty
    while stack:
        (robot_pos, dirty_cells), path = stack.pop()
        nodes_expanded += 1

        if not dirty_cells:
            return path, {
                "generated": nodes_generated,
                "expanded": nodes_expanded
            }
        
        r, c = robot_pos
        # Try cleaning a cell if dirty
        if (r, c) in dirty_cells:
            new_dirty_cells = dirty_cells - {(r, c)}
            new_state = (robot_pos, frozenset(new_dirty_cells))
            if new_state not in visited:
                visited.add(new_state)
                stack.append((new_state, path + ['V']))
                nodes_generated += 1

        # Try moving in all directions
        for action, (dr, dc) in moves.items():
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and (nr, nc) not in walls:
                new_pos = (nr, nc)
                new_state = (new_pos, dirty_cells)
                if new_state not in visited:
                    visited.add(new_state)
                    stack.append((new_state, path + [action]))
                    nodes_generated += 1

    return None, {"generated": nodes_generated, "expanded": nodes_expanded}


def ucs(start_state):
    # Initialize variables
    rows, cols = start_state["rows"], start_state["cols"]
    walls = start_state["walls"]
    robot_pos = start_state["robot_pos"]
    dirty_cells = start_state["dirty_cells"]
    
    initial_state = (robot_pos, frozenset(dirty_cells))
    heap = []
    visited = {}

    heapq.heappush(heap, (0, initial_state, []))
    visited[initial_state] = 0

    nodes_generated = 1
    nodes_expanded = 0

    moves = {
        'N': (-1, 0),
        'S': (1, 0),
        'E': (0, 1),
        'W': (0, -1)
    }

    # Iterate until heap is empty
    while heap:
        cost, (robot_pos, dirty_cells), path = heapq.heappop(heap)
        if cost > visited[(robot_pos, dirty_cells)]:
            continue
        nodes_expanded += 1

        if not dirty_cells:
            return path, {
                "generated": nodes_generated,
                "expanded": nodes_expanded
            }
        
        r, c = robot_pos
        # Try cleaning a cell if dirty
        if (r, c) in dirty_cells:
            new_dirty_cells = dirty_cells - {(r, c)}
            new_state = (robot_pos, frozenset(new_dirty_cells))
            new_cost = cost + 1
            if new_state not in visited or new_cost < visited[new_state]:
                visited[new_state] = new_cost
                heapq.heappush(heap, (new_cost, new_state, path + ['V']))
                nodes_generated += 1

        # Try moving in all directions
        for action, (dr, dc) in moves.items():
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and (nr, nc) not in walls:
                new_pos = (nr, nc)
                new_state = (new_pos, dirty_cells)
                new_cost = cost + 1
                if new_state not in visited or new_cost < visited[new_state]:
                    visited[new_state] = new_cost
                    heapq.heappush(heap, (new_cost, new_state, path + [action]))
                    nodes_generated += 1

    return None, {"generated": nodes_generated, "expanded": nodes_expanded}




def main():
    if len(sys.argv) != 3:
        print("Usage: python3 planner.py <algorithm> <world-file>")
        sys.exit(1)
    
    # Parse arguments
    algorithm = sys.argv[1]
    world_file = sys.argv[2]

    # Parse world file
    start_state = parse_world(world_file)
    if algorithm == "uniform-cost":
        path, stats = ucs(start_state)
    elif algorithm == "depth-first":
        path, stats = dfs(start_state)
    else:
        print(f"Unknown algorithm: {algorithm}")
        sys.exit(1)

    if path is None:
        print("No path found.")
    else:
        for action in path:
            print(action)
    print(f"{stats['generated']} nodes generated")
    print(f"{stats['expanded']} nodes expanded")


if __name__ == "__main__":
    main()