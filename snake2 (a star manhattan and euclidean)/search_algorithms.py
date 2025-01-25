import heapq
from math import sqrt

def astar(start, goal, obstacles, rows, cols, heuristic_type):
    """
    A* algorithm to find the shortest path from start to goal.

    Args:
    - start: tuple (row, col), the starting position of the snake
    - goal: tuple (row, col), the target position (food)
    - obstacles: set of obstacle positions
    - rows: number of rows in the grid
    - cols: number of columns in the grid
    - heuristic_type: "manhattan" or "euclidean"

    Returns:
    - A list of moves (e.g., [(0, 1), (1, 0), ...]) that represents the path
      from the start to the goal. If no path is found, returns an empty list.
    """
    # Priority queue for A*: (cost, current_position, path_taken)
    open_list = []
    heapq.heappush(open_list, (0, start, []))

    # Set of visited nodes
    visited = set()

    # Heuristic functions
    def manhattan(pos, goal):
        return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])

    def euclidean(pos, goal):
        return sqrt((pos[0] - goal[0]) ** 2 + (pos[1] - goal[1]) ** 2)

    heuristic = manhattan if heuristic_type == "manhattan" else euclidean

    # Perform A* search
    while open_list:
        # Get the node with the lowest cost (f = g + h)
        current_cost, current, path = heapq.heappop(open_list)

        # If the goal is reached, return the path
        if current == goal:
            return path

        # Skip if this position has already been visited
        if current in visited:
            continue

        # Mark the current node as visited
        visited.add(current)

        # Explore neighbors
        for direction in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # Up, Down, Left, Right
            new_row = current[0] + direction[0]
            new_col = current[1] + direction[1]
            new_pos = (new_row, new_col)

            # Check if the new position is valid
            if (
                0 <= new_row < rows and
                0 <= new_col < cols and
                new_pos not in obstacles and
                new_pos not in visited
            ):
                # Calculate the cost (f = g + h)
                g_cost = len(path) + 1  # Path cost so far
                h_cost = heuristic(new_pos, goal)  # Heuristic cost
                f_cost = g_cost + h_cost

                # Add the new position to the priority queue
                heapq.heappush(open_list, (f_cost, new_pos, path + [direction]))

    # If no path is found, return an empty list
    return []
