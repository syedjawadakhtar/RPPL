# RPPL
Robot Planning Python Library

Copyright (c) 2021 Alexander J. LaValle. All rights reserved.
This software is distributed under the simplified BSD license.

This is a simple library for computing collision-free paths for robots among obstacles. It was developed as support material for the course Fundamentals of Sensing, Tracking, and Autonomy at the University of Oulu. 

INSTALLATION:

This library was developed on Windows 10 using Python 3.9.5, and was also tested on Ubuntu 20.04. The following libraries are needed: NumPy, Pygame, NetworkX, Tk, and Matplotlib. These can be installed via pip or whatever installer you use for Python.

Main files:

- valit_simple.py:  Input a weighted graph and the code computes the optimal, stationary cost-to-go function for a goal region using value iteration.

- valit_grids.py:  Runs a GUI that selects one of several 2D planning problems which can be created with draw_circles.py. A 2D grid is constructed based on various neighborhood options. Value iteration is used to compute optimal, stationary cost-to-go function over the grid and an optimal path is computed and shown. An option to instead use Dijkstra's algorithm is also available. Using the GUI, the values can be saved and then read using valit_plot_values.py so that they can be easily visualized as a 3D surface.

- RRT_2D_discs.py:  Generates and solves 2D planning problems by generating random disc obstacles and solving the planning problem using a goal-biased rapidly exploring random tree (RRT).

- RRT_2D_polygons.py:  Similar to RRT_2D_discs.py, but instead works for obstacles that are simple polygons. To create an example problem, use draw_polygons.py.

- RRT_LSR.py:  Solves planning problems using goal bias or bidirectional RRT for a 2D kinematic chain of line segments with a fixed base and arbitrarily many links. The base is fixed.
