import pygame
import math
from queue import PriorityQueue

# Initialize Pygame
pygame.init()

# Set up display
WIDTH, HEIGHT = 800, 800
win = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("A* Pathfinding")

# Define grid parameters
ROWS = 50
COLS = 50
GRID_SIZE = WIDTH // ROWS

# Define node class
class Node:
    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.x = row * GRID_SIZE
        self.y = col * GRID_SIZE
        self.color = (255,255,255)
        self.neighbors = []

    def draw(self, win):
        pygame.draw.rect(win, self.color, (self.x, self.y, GRID_SIZE, GRID_SIZE))

    def update_neighbors(self, grid):
        self.neighbors = []
        if self.row < ROWS - 1 and not grid[self.row + 1][self.col].color == (0,0,0):
            self.neighbors.append(grid[self.row + 1][self.col])
        if self.row > 0 and not grid[self.row - 1][self.col].color == (0,0,0):
            self.neighbors.append(grid[self.row - 1][self.col])
        if self.col < COLS - 1 and not grid[self.row][self.col + 1].color == (0,0,0):
            self.neighbors.append(grid[self.row][self.col + 1])
        if self.col > 0 and not grid[self.row][self.col - 1].color == (0,0,0):
            self.neighbors.append(grid[self.row][self.col - 1])

# Heuristic function (Euclidean distance)
def heuristic(node1, node2):
    return 1.5*math.sqrt((node1.row - node2.row) ** 2 + (node1.col - node2.col) ** 2)
    # return 1*(abs(node1.row - node2.row) + abs(node1.col - node2.col))

# A* algorithm
def astar_algorithm(grid, start, end):
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start))
    came_from = {}
    g_score = {node: float("inf") for row in grid for node in row}
    g_score[start] = 0
    f_score = {node: float("inf") for row in grid for node in row}
    f_score[start] = heuristic(start, end)

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        current = open_set.get()[2]

        if current == end:
            reconstruct_path(came_from, end)
            end.color = (255,0,0)
            return True

        for neighbor in current.neighbors:
            temp_g_score = g_score[current] + 1

            if temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score + heuristic(neighbor, end)
                if neighbor not in [item[2] for item in open_set.queue]:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))
                    neighbor.color = (255,255,0)

        draw_grid(grid)
        pygame.display.update()

        if current != start:
            current.color = (0,0,255)

    return False

# Reconstruct path
def reconstruct_path(came_from, current):
    while current in came_from:
        current = came_from[current]
        current.color = (0,255,0)

# Draw grid
def draw_grid(grid):
    win.fill((0,0,0))
    for row in grid:
        for node in row:
            node.draw(win)
    pygame.display.update()

# Create grid
def make_grid():
    grid = [[Node(row, col) for col in range(COLS)] for row in range(ROWS)]
    for row in grid:
        for node in row:
            node.update_neighbors(grid)
    return grid

# Main function
def main():
    grid = make_grid()

    start = grid[0][0]
    end = grid[ROWS - 1][COLS - 1]

    start.color = (0,255,0)
    end.color = (255,0,0)

    run = True
    while run:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            if pygame.mouse.get_pressed()[0]:  # Left mouse button
                pos = pygame.mouse.get_pos()
                row = pos[0] // GRID_SIZE
                col = pos[1] // GRID_SIZE
                node = grid[row][col]
                if node != start and node != end:
                    node.color = (0,0,0)

            elif pygame.mouse.get_pressed()[2]:  # Right mouse button
                pos = pygame.mouse.get_pos()
                row = pos[0] // GRID_SIZE
                col = pos[1] // GRID_SIZE
                node = grid[row][col]
                if node != start and node != end:
                    node.color = (0,0,0)

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    for row in grid:
                        for node in row:
                            node.update_neighbors(grid)
                    astar_algorithm(grid, start, end)

        draw_grid(grid)

    pygame.quit()

if __name__ == "__main__":
    main()
