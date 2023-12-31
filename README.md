# A* Algorithm

This repository contains a Python implementation of the A* algorithm with visualization capabilities. It allows users to observe the exploration of nodes and the identified path using matplotlib. The code provides flexibility for experimenting with various heuristic functions to analyze the algorithm's behavior.

## Libraries required

1. pygame
2. numpy
3. matplotlib
   
## Usage

1. Run the program.
2. Enter the grid size (e.g., 25) when prompted.
3. Enter the heuristic function to use (e for Euclidean, m for Manhattan).
4. The program will open a pygame window.
5. In the window, the green node at the top-left is the start node, and the red node in the bottom-right corner is the goal node.
6. Drag the mouse on the white area to draw obstacles, then close the window.
7. The algorithm will start, and upon completion, it will display the output.

## Input grid
![image](https://github.com/Nisarg236/aStar/assets/71684502/6f93b30e-d9c3-497c-a7fe-dc461ce11fc3)

## Output with Euclidian:
![image](https://github.com/Nisarg236/aStar/assets/71684502/98e296e5-2d2f-459e-8737-76cf037bebc5)

In the output, the blue nodes represent the explored nodes, and the red nodes represent the path.

## Input grid:
![image](https://github.com/Nisarg236/aStar/assets/71684502/51afc007-8970-4c45-9c85-4bbf3371e0ac)

## Output using Manhattan:
![image](https://github.com/Nisarg236/aStar/assets/71684502/d9a5ee41-169e-4ae1-b7c8-f331746de8a7)

## How to Contribute

If you'd like to contribute to this project, feel free to submit a pull request or open an issue.

## License

This project is licensed under the [MIT License](LICENSE).



