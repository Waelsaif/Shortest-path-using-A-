from queue import PriorityQueue
# Define the graph as a dictionary of nodes and their neighbors
# UQU, shaqiah, azizyah, kakia, wly_alahd
graph = {
    'UQU': [('shaqiah', 1), ('azizyah', 4)],
    'shaqiah': [('UQU', 1), ('azizyah', 2), ('kakia', 5)],
    'azizyah': [('UQU', 4), ('shaqiah', 2), ('kakia', 1)],
    'kakia': [('shaqiah', 5), ('azizyah', 1), ('wly_alahd', 3)],
    'wly_alahd': [('kakia', 3)]
}

# Define the heuristic function as the Euclidean distance to the goal node
heuristics = {
    'UQU': 6,
    'shaqiah': 5,
    'azizyah': 4,
    'kakia': 3,
    'wly_alahd': 0
}


def astar(start, goal):
    # Initialize the open set with the start node and its f-score
    open_set = PriorityQueue()
    open_set.put((0, start))
    # Initialize the closed set
    closed_set = set()
    # Initialize the g-score for each node to infinity
    g_scores = {node: float('inf') for node in graph}
    g_scores[start] = 0
    # Initialize the f-score for each node to infinity
    f_scores = {node: float('inf') for node in graph}
    f_scores[start] = heuristics[start]
    # Initialize the came_from dictionary
    came_from = {}

    while not open_set.empty():
        # Get the node in the open set with the lowest f-score
        current = open_set.get()[1]

        # If the current node is the goal node, return the path
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        # Add the current node to the closed set
        closed_set.add(current)

        # Check each neighbor of the current node
        for neighbor, distance in graph[current]:
            # If the neighbor is in the closed set, skip it
            if neighbor in closed_set:
                continue

            # Calculate the tentative g-score for the neighbor
            tentative_g_score = g_scores[current] + distance

            # If the neighbor is not in the open set, add it
            if neighbor not in [node[1] for node in open_set.queue]:
                open_set.put((f_scores[neighbor], neighbor))

            # If the tentative g-score is greater than the current g-score for the neighbor, skip it
            elif tentative_g_score >= g_scores[neighbor]:
                continue

            # Record the best path so far to the neighbor
            came_from[neighbor] = current
            g_scores[neighbor] = tentative_g_score
            f_scores[neighbor] = g_scores[neighbor] + heuristics[neighbor]

    # If the goal node is not reachable from the start node, return None
    return None


# Test the algorithm with start node 'A' and goal node 'E'
path = astar('UQU', 'wly_alahd')
print(path)

# This code implements the A* algorithm to find the shortest path from a start node to a goal node in a graph.
# The graph is represented as a dictionary where each node is a key and its value is a list of tuples representing its neighbors and the distance to them. For example, the node 'UQU' has two neighbors, 'shaqiah' with distance 1 and 'azizyah' with distance 4, so it is represented as 'UQU': [('shaqiah', 1), ('azizyah', 4)].
# the code tests the astar function by finding the shortest path from the start node 'UQU' to the goal node 'wly_alahd' and prints it. In this case, the result is ['UQU', 'azizyah', 'kakia', 'wly_alahd'], which represents the path UQU -> azizyah -> kakia -> wly_alahd.
