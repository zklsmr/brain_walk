# Define a function that returns the cost of moving from one node to another.
# In this example, we use the distance between two nodes as the cost.
def get_cost(node, neighbor):
    return node.distance_to(neighbor)

# Define the heuristic function that estimates the distance between a node and the goal.
# In this example, we use the Euclidean distance between two nodes.
def heuristic(node, goal):
    return node.distance_to(goal)

# Define the A* algorithm.
def a_star(start, goal, nodes):
    frontier = []  # Use a list to store nodes to explore.
    heapq.heappush(frontier, (0, start))  # Push the start node onto the heap.

    came_from = {}  # Use a dictionary to store the parent of each node.
    cost_so_far = {}  # Use a dictionary to store the cost of reaching each node.
    came_from[start] = None
    cost_so_far[start] = 0

    while frontier:
        _, current = heapq.heappop(frontier)  # Pop the node with the lowest f-score.

        if current == goal:
            break

        for neighbor in nodes.get_neighbors(current):
            new_cost = cost_so_far[current] + get_cost(current, neighbor)

            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(neighbor, goal)
                heapq.heappush(frontier, (priority, neighbor))
                came_from[neighbor] = current

    path = []
    current = goal
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()

    return path
