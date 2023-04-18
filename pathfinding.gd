# Define a function that returns the cost of moving from one node to another.
# In this example, we use the distance between two nodes as the cost.
func get_cost(node, neighbor):
    return node.distance_to(neighbor)

# Define the heuristic function that estimates the distance between a node and the goal.
# In this example, we use the Euclidean distance between two nodes.
func heuristic(node, goal):
    return node.distance_to(goal)

# Define the A* algorithm.
func a_star(start, goal, nodes):
    var frontier = [] # Use an Array to store nodes to explore.
    frontier.push_back(start) # Push the start node onto the heap.

    var came_from = {} # Use a Dictionary to store the parent of each node.
    var cost_so_far = {} # Use a Dictionary to store the cost of reaching each node.
    came_from[start] = null
    cost_so_far[start] = 0

    while frontier.size() > 0:
        var current = get_lowest_fscore_node(frontier, goal, cost_so_far) # Pop the node with the lowest f-score.

        if current == goal:
            break

        for neighbor in nodes.get_neighbors(current):
            var new_cost = cost_so_far[current] + get_cost(current, neighbor)

            if !cost_so_far.has(neighbor) or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                var priority = new_cost + heuristic(neighbor, goal)
                frontier.push_back(neighbor)
                frontier.sort_custom(lambda n: heuristic(n, goal) + cost_so_far[n])
                came_from[neighbor] = current

    var path = []
    var current = goal
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.invert()

    return path

# Helper function to get the node with the lowest f-score from the frontier.
func get_lowest_fscore_node(frontier, goal, cost_so_far):
    var lowest_score = INFINITY
    var lowest_score_node = null

    for node in frontier:
        var score = heuristic(node, goal) + cost_so_far[node]
        if score < lowest_score:
            lowest_score = score
            lowest_score_node = node

    frontier.erase(frontier.find(lowest_score_node))
    return lowest_score_node
