#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Cody Dillinger

import math
from math import*


class Vertex:
    def __init__(self, x_val, y_val):  # initialize with x, y coordinates
        self.x = x_val
        self.y = y_val
        self.children = []
        self.parents = []
        self.paths = [[]]

    def add_child(self, child):
        self.children.append(child)

    def add_parent(self, parent):
        self.parents.append(parent)

    def add_path(self, path_):
        self.paths.append(path_)

    def add_path_vertex(self, i, vertex_):
        self.paths[i].append(vertex_)

    x = 0
    y = 0
    obs_next = None                 # for shape edges (obstacles and goal set)
    at_goal_set = False
    k_nearest = 0                   # for avoiding checking points multiple times in recursion searches
    k_near = 0


class Colors:                       # RGB color values
    def __init__(self):
        pass

    white = (255, 255, 255)
    black = (0, 0, 0)
    grey = (128, 128, 128)
    light_green = (0, 255, 0)
    dark_green = (0, 200, 0)
    turquoise = (0, 255, 255)
    light_blue = (0, 0, 255)
    dark_blue = (0, 0, 200)
    light_red = (255, 0, 0)
    dark_red = (200, 0, 0)
    pink = (200, 20, 240)
    purple = (100, 0, 255)
    light_orange = (255, 100, 0)
    dark_orange = (175, 100, 0)


class Color_:                  # for easy iterating between colors for unknown number of robot trees
    def __init__(self, clr):
        self.color_ = clr
        self.color_list = []

    def add_color(self, color_new):
        self.color_list.append(color_new)
    next_ = None
    color_ = (0, 0, 0)


class Window:
    def __init__(self):
        pass

    length = 500
    width = 700


class Dimensions:
    def __init__(self):
        pass
    window_length = 500     # size of pywindow display
    window_width = 700
    tree_radius = 7         # size of radius for steering point towards random vertex
    eta = tree_radius  # size of radius in comparison for min() function in near() function

    # Vertices of static obstacles
    # Algorithm works for any number of obstacles with any number of vertices
    # Only small changes here and within obstacle_generation() function
    # Pay attention to vertex order
    window = [Vertex(0, 0), Vertex(0, 700), Vertex(500, 700), Vertex(500, 0)]
    obstacle1 = [Vertex(0, 455), Vertex(0, 475), Vertex(280, 475), Vertex(280, 455)]
    obstacle2 = [Vertex(220, 225), Vertex(220, 245), Vertex(500, 245), Vertex(500, 225)]

    # For near() vertices functionality, simplified for 2-D motion planning
    unit_ball = 3.1415926
    obs1_volume = (obstacle1[1].y - obstacle1[0].y) * (obstacle1[2].x - obstacle1[1].x)
    obs2_volume = (obstacle2[1].y - obstacle2[0].y) * (obstacle2[2].x - obstacle2[1].x)
    total_space = (window_length * window_width) - (obs1_volume + obs2_volume)  # window size minus obstacle size
    gamma = 2 * sqrt(3) * sqrt(total_space / unit_ball)    # from theorem 38 RRT*