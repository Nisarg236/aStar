# A* Algorithm

This repository contains a Python implementation of the A* algorithm with visualization capabilities. It allows users to observe the exploration of nodes and the identified path using matplotlib. The code provides flexibility for experimenting with various heuristic functions to analyze the algorithm's behavior.

## Libraries required

1. pygame
2. numpy
3. matplotlib
   
## Usage

1. Run the program.
2. Enter the grid size (e.g. 25) when prompted.
3. Enter the heuristic function to use (e for Euclidean, m for Manhattan).
4. The program will open a pygame window.
5. In the window, the green node at the top-left is the start node, and the red node in the bottom-right corner is the goal node.
6. Drag the mouse on the white area to draw obstacles, then close the window.
7. The algorithm will start, and upon completion, it will display the output.

## Input grid
![image](https://github.com/Nisarg236/aStar/assets/71684502/2968ba6a-1d5e-45a3-a0fd-804faf74fda2)


## Output with Euclidian:
![image](https://github.com/Nisarg236/aStar/assets/71684502/571cdf06-1c54-4194-9ce5-a080c543274c)

In the output, the blue nodes represent the explored nodes, and the red nodes represent the path.

## Input grid:
![image](https://github.com/Nisarg236/aStar/assets/71684502/30edb76d-254e-4138-9c6a-1a256c98b47d)


## Output using Manhattan:
![image](https://github.com/Nisarg236/aStar/assets/71684502/76ea4352-4eab-4dce-a8b9-91d2704362aa)

## How to Contribute

If you'd like to contribute to this project, feel free to submit a pull request or open an issue.

## License

This project is licensed under the [MIT License](LICENSE).



