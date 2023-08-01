import heapq

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.g = float('inf')
        self.h = 0
        self.parent = None

    def __lt__(self, other):
        # Custom comparison method for heapq
        return (self.g + self.h) < (other.g + other.h)

def heuristic(node, goal):
    # Calculate the Manhattan distance as the heuristic
    return abs(node.x - goal.x) + abs(node.y - goal.y)

def get_neighbors(node, grid):
    # Returns the neighbors of a node in the grid
    neighbors = []
    directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # 4-directional movement
    for dx, dy in directions:
        nx, ny = node.x + dx, node.y + dy
        if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]) and not grid[nx][ny]:
            neighbors.append(Node(nx, ny))
    return neighbors

def reconstruct_path(node):
    # Reconstruct the path from the goal node to the start node
    path = []
    current = node
    while current is not None:
        path.append((current.x, current.y))
        current = current.parent
    return path[::-1]

def a_star(grid, start, goal):
    open_set = [start]
    start.g = 0
    start.h = heuristic(start, goal)

    while open_set:
        current = heapq.heappop(open_set)

        if current.x == goal.x and current.y == goal.y:
            return reconstruct_path(current)

        for neighbor in get_neighbors(current, grid):
            tentative_g = current.g + 1  # Assuming the cost of moving between adjacent nodes is 1
            if tentative_g < neighbor.g:
                neighbor.g = tentative_g
                neighbor.h = heuristic(neighbor, goal)
                neighbor.parent = current
                heapq.heappush(open_set, neighbor)

    return None

if __name__ == "__main__":
    # Example usage
    grid = [
        [0, 0, 0, 0, 0],
        [0, 1, 1, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 1, 1, 0],
        [0, 0, 0, 0, 0]
    ]
    start_node = Node(1, 0)
    goal_node = Node(4, 4)

    path = a_star(grid, start_node, goal_node)
    if path:
        print("Path found:", path)
    else:
        print("Path not found.")
