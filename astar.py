import numpy as np
from queue import PriorityQueue
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pygame
import numpy as np


class Node:
    def __init__(self, x, y, g, h, parent):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent
        self.id = (x, y)


def heuristic(node, goal, heuristic_function):
    if heuristic_function == "e":
        value = 10 * np.sqrt((node.x - goal.x) ** 2 + (node.y - goal.y) ** 2)
        return value
    if heuristic_function == "m":
        value = 10 * abs(node.x - goal.x) + abs(node.y - goal.y)
        return value


def find_neighbours(node, grid, goal, open_list):
    neighbours = []
    if node.x - 1 >= 0:
        if grid[node.x - 1, node.y] == 1: 
            neighbour_w = Node(node.x - 1, node.y, node.g + 10, 0, node)
            neighbour_w.h = heuristic(neighbour_w, goal, heuristic_function)
            neighbour_w.f = neighbour_w.h + neighbour_w.g
            if not any(i[2].id == neighbour_w.id for i in open_list.queue):
                neighbours.append(neighbour_w)

    if node.x + 1 < grid.shape[0]:
        if grid[node.x + 1, node.y] == 1:
            neighbour_e = Node(node.x + 1, node.y, node.g + 10, 0, node)
            neighbour_e.h = heuristic(neighbour_e, goal, heuristic_function)
            neighbour_e.f = neighbour_e.h + neighbour_e.g
            if not any(i[2].id == neighbour_e.id for i in open_list.queue):
                neighbours.append(neighbour_e)

    if node.y - 1 >= 0:
        if grid[node.x, node.y - 1] == 1:
            neighbour_n = Node(node.x, node.y - 1, node.g + 10, 0, node)
            neighbour_n.h = heuristic(neighbour_n, goal, heuristic_function)
            neighbour_n.f = neighbour_n.h + neighbour_n.g
            if not any(i[2].id == neighbour_n.id for i in open_list.queue):
                neighbours.append(neighbour_n)

    if node.y + 1 < grid.shape[1]:
        if grid[node.x, node.y + 1] == 1:
            neighbour_s = Node(node.x, node.y + 1, node.g + 10, 0, node)
            neighbour_s.h = heuristic(neighbour_s, goal, heuristic_function)
            neighbour_s.f = neighbour_s.h + neighbour_s.g
            if not any(i[2].id == neighbour_s.id for i in open_list.queue):
                neighbours.append(neighbour_s)
    return neighbours

def astar_algorithm(grid, start, goal, heuristic_function):
    open_list = PriorityQueue() #We are using priority queue and arranging based on F score because we have to frequently fetch the node with least f score
    closed_set = set() #set is used to prevent duplication

    start_node = Node(start.x, start.y, 0, 0, None) #initialize the start node
    goal_node = Node(goal.x, goal.y, 0, 0, None) #initialize the goal node

    open_list.put((start_node.f, np.random.rand(), start_node)) #first we will put the start node in open list 
    explored_nodes = [start_node] #this list we are maintaining only for visualization purposes

    while not open_list.empty():
        current_node = open_list.get()[2] #get the current node from the open list
        # open_list.pop(current_node) #remove current node from openlist
        #if current node locations is same as goal node location then trace the parents to form path, this if block will run at last when we reach goal node
        if current_node.id == goal_node.id:
            path = []
            while current_node:
                path.append(current_node)
                current_node = current_node.parent
            return path[::-1], explored_nodes

        neighbours = find_neighbours(current_node, grid, goal_node, open_list) #find neighbours of the current node 
        closed_set.add(current_node) # put the current node in closed set
        
        # #add current node to explored node if it is not there already
        if not any(node.id == current_node.id for node in explored_nodes):
            explored_nodes.append(current_node)
        #loop through the neighbours
        for neighbour in neighbours:
            if neighbour.id in closed_set: #if the neighbour is in closed set, then it is already explored so donot constder that one
                continue

            if neighbour.f < current_node.f or not any(i[2].id == neighbour.id for i in open_list.queue): #if the F score of a neighbour is less than current node and it is not already a part of open list then add it to the open list
                open_list.put((neighbour.f, np.random.rand(), neighbour)) #put the neighbour in open list based on F score
    return None, None

def visualize(grid, path=None, explored_nodes=None):
    # Create a figure and axis for the visualization
    fig, ax = plt.subplots()
    ax.matshow(grid, cmap="gray", origin="upper")

    # Plot the explored nodes if provided
    if explored_nodes:
        # Extract the coordinates of explored nodes
        explored_points = np.array([(node.y, node.x) for node in explored_nodes])
        # Scatter plot for explored nodes
        explored_scatter = ax.scatter([], [], color="blue", marker="s", s=50)

    # Plot the path if provided
    if path:
        # Extract the coordinates of path nodes
        path_points = np.array([(node.y, node.x) for node in path])
        # Scatter plot for path nodes
        path_scatter = ax.scatter([], [], color="red", marker="s", s=50)

    def update(frame):
        # Update the scatter plot for explored nodes
        if frame < len(explored_nodes):
            explored_scatter.set_offsets(explored_points[:frame + 1])
        # Update the scatter plot for the path nodes
        elif frame < len(explored_nodes) + len(path):
            path_frame = frame - len(explored_nodes)
            path_scatter.set_offsets(path_points[:path_frame + 1])

    # Total frames for the animation
    frames = len(explored_nodes) + len(path)
    # Create the animation using FuncAnimation
    anim = FuncAnimation(fig, update, frames=frames, interval=0, repeat=False)

    # Set title and labels for the plot
    plt.title("A* Algorithm Visualization")
    plt.xlabel("Y")
    plt.ylabel("X")

    # Display the animation
    plt.show()

def create_grid(grid, start_node, goal_node):
    grid_size = grid.shape[:2]
    cell_size = 20 
    pygame.init()

    window_size = (grid_size[1] * cell_size, grid_size[0] * cell_size)
    screen = pygame.display.set_mode(window_size)
    pygame.display.set_caption("Click to make obstacles")

    running = True
    clock = pygame.time.Clock()

    drawing_obstacle = False

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left mouse button
                    drawing_obstacle = True
            elif event.type == pygame.MOUSEMOTION and drawing_obstacle:
                pos = pygame.mouse.get_pos()
                j, i = pos[0] // cell_size, pos[1] // cell_size
                grid[i, j] = 0  # Set obstacle
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:  # Left mouse button
                    drawing_obstacle = False

        screen.fill((255, 255, 255)) 

        for i in range(grid_size[0]):
            for j in range(grid_size[1]):
                if grid[i, j] == 0:
                    pygame.draw.rect(
                        screen,
                        (0, 0, 0),
                        (j * cell_size, i * cell_size, cell_size, cell_size),
                    )
                else:
                    pygame.draw.rect(
                        screen,
                        (255, 255, 255),
                        (j * cell_size, i * cell_size, cell_size, cell_size),
                        1,
                    )

        pygame.draw.rect(
            screen,
            (0, 255, 0),
            (start_node.y * cell_size, start_node.x * cell_size, cell_size, cell_size),
        )

        pygame.draw.rect(
            screen,
            (255, 0, 0),
            (goal_node.y * cell_size, goal_node.x * cell_size, cell_size, cell_size),
        )

        pygame.display.flip()
        clock.tick(60) 

    pygame.quit()

    return grid


grid_size = int(input("Please enter the Grid Size: "))
heuristic_function = input(
    "Heuristic function, enter e for Euclidian, m for manhattan :"
)

grid = np.ones((grid_size, grid_size), dtype=np.uint8)

start_node = Node(0, 0, 0, 0, None)
goal_node = Node(grid_size - 1, grid_size - 1, 0, 0, None)
grid = create_grid(grid, start_node, goal_node)


path, explored_nodes = astar_algorithm(grid, start_node, goal_node, heuristic_function)
for node in path:
    print(f"({node.x}, {node.y})", end=" -> ")

visualize(grid, path, explored_nodes)
