import argparse

parser = argparse.ArgumentParser()
parser.add_argument('file')
args = parser.parse_args()


import numpy as np
import nibabel as nib
from queue import PriorityQueue
from math import sqrt

def euclidean_dist(a, b):
    return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2 + (a[2] - b[2])**2)

def a_star(start, goal, graph):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {start: 0}

    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            break

        for next_node in graph[current]:
            new_cost = cost_so_far[current] + euclidean_dist(current, next_node)
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost + euclidean_dist(goal, next_node)
                frontier.put(next_node, priority)
                came_from[next_node] = current

    path = [goal]
    while path[-1] != start:
        path.append(came_from[path[-1]])
    path.reverse()

    return path

# Load NIfTI image
img = nib.load(args.file)
data = img.get_fdata()

# Convert image to graph representation
graph = {}
for i in range(data.shape[0]):
    for j in range(data.shape[1]):
        for k in range(data.shape[2]):
            if data[i, j, k] == 0: # voxel is traversable
                node = (i, j, k)
                neighbors = []
                if i > 0 and data[i-1, j, k] == 0:
                    neighbors.append((i-1, j, k))
                if i < data.shape[0] - 1 and data[i+1, j, k] == 0:
                    neighbors.append((i+1, j, k))
                if j > 0 and data[i, j-1, k] == 0:
                    neighbors.append((i, j-1, k))
                if j < data.shape[1] - 1 and data[i, j+1, k] == 0:
                    neighbors.append((i, j+1, k))
                if k > 0 and data[i, j, k-1] == 0:
                    neighbors.append((i, j, k-1))
                if k < data.shape[2] - 1 and data[i, j, k+1] == 0:
                    neighbors.append((i, j, k+1))
                graph[node] = neighbors

# Define start and goal positions
start = (0, 0, 0)
goal = (data.shape[0]-1, data.shape[1]-1, data.shape[2]-1)

# Find optimal path using A*
path = a_star(start, goal, graph)

# Print path
print(path)
