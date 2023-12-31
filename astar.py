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
        value = 500 * np.sqrt((node.x - goal.x) ** 2 + (node.y - goal.y) ** 2)
    if heuristic_function == "m":
        value = 500 * abs(node.x - goal.x) + abs(node.y - goal.y)
        return value


def find_neighbours(node, grid, goal, open_list):
    neighbours = []
    if node.x - 1 >= 0:
        if grid[node.x - 1, node.y] == 1: 
            neighbour_w = Node(node.x - 1, node.y, node.g + 10, 0, node)
            neighbour_w.h = heuristic(neighbour_w, goal, heuristic_function)
            neighbour_w.f = neighbour_w.h + neighbour_w.g
            j = True
            for i in open_list.queue:
                if i[2].id == neighbour_w.id:
                    j = False
            if j == True:
                neighbours.append(neighbour_w)

    if node.x + 1 < grid.shape[0]:
        if grid[node.x + 1, node.y] == 1:
            neighbour_e = Node(node.x + 1, node.y, node.g + 10, 0, node)
            neighbour_e.h = heuristic(neighbour_e, goal, heuristic_function)
            neighbour_e.f = neighbour_e.h + neighbour_e.g
            neighbours.append(neighbour_e)
            j = True
            for i in open_list.queue:
                if i[2].id == neighbour_e.id:
                    j = False
            if j == True:
                neighbours.append(neighbour_e)

    if node.y - 1 >= 0:
        if grid[node.x, node.y - 1] == 1:
            neighbour_n = Node(node.x, node.y - 1, node.g + 10, 0, node)
            neighbour_n.h = heuristic(neighbour_n, goal, heuristic_function)
            neighbour_n.f = neighbour_n.h + neighbour_n.g
            neighbours.append(neighbour_n)
            j = True
            for i in open_list.queue:
                if i[2].id == neighbour_n.id:
                    j = False
            if j == True:
                neighbours.append(neighbour_n)

    if node.y + 1 < grid.shape[1]:
        if grid[node.x, node.y + 1] == 1:
            neighbour_s = Node(node.x, node.y + 1, node.g + 10, 0, node)
            neighbour_s.h = heuristic(neighbour_s, goal, heuristic_function)
            neighbour_s.f = neighbour_s.h + neighbour_s.g
            neighbours.append(neighbour_s)
            j = True
            for i in open_list.queue:
                if i[2].id == neighbour_s.id:
                    j = False
            if j == True:
                neighbours.append(neighbour_s)

    return neighbours


def astar_algorithm(grid, start_node, goal_node, heuristic_function):
    count = 0
    open_list = PriorityQueue()
    open_list.put((0, count, start_node))
    came_from = {}
    g_score = {start_node: 0}
    explored_nodes = []

    while not open_list.empty():
        print("Calculating path...")
        current_node = open_list.get()[2]
        a = True
        for node in explored_nodes:
            if current_node.id == node.id:
                a = False
        if a == True:
            explored_nodes.append(current_node)
        if current_node.x == goal_node.x and current_node.y == goal_node.y:
            path = []
            while current_node in came_from:
                path.append(current_node)
                current_node = came_from[current_node]
            return path[::-1], explored_nodes  # Return reversed path

        neighbors = find_neighbours(current_node, grid, goal_node, open_list)
        for neighbor in neighbors:
            if (
                0 <= neighbor.x < grid.shape[0]
                and 0 <= neighbor.y < grid.shape[1]
                and grid[neighbor.x, neighbor.y] == 1
            ):
                g = g_score[current_node] + neighbor.g

                if neighbor not in g_score or g < g_score[neighbor]:
                    count += 1
                    neighbor.f = g + heuristic(neighbor, goal_node, heuristic_function)
                    j = True
                    for i in open_list.queue:
                        if neighbor.id == i[2].id:
                            j = False
                    if j == True:
                        open_list.put((neighbor.f, count, neighbor))
                        came_from[neighbor] = current_node
                        g_score[neighbor] = g

    return None  


def visualize(grid, path=None, explored_nodes=None):
    plt.matshow(grid, cmap="gray", origin="upper")

    if explored_nodes:
        i = 0
        for node in explored_nodes:
            plt.plot(node.y, node.x, color="blue", marker="s", markersize=10)
            plt.pause(0.003)
        for node in path:
            plt.plot(node.y, node.x, color="red", marker="s", markersize=10)
            plt.pause(0.003)

    plt.title("A* Algorithm Visualization")
    plt.xlabel("Y")
    plt.ylabel("X")
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
