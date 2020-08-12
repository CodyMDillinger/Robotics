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
        self.paths = []
        self.costs = []

    def add_child(self, child):
        self.children.append(child)

    def add_parent(self, parent):
        self.parents.append(parent)

    def add_path(self, path_):
        self.paths.append(path_)     # for list of paths from root node to this vertex

    def add_cost(self, cost_):
        self.costs.append(cost_)     # for list of costs associated with paths in self.paths

    x = 0                           # x,y coordinates
    y = 0
    obs_next = None                 # for shape edges (obstacles and goal set)
    at_goal_set = False
    k_nearest = 0                   # for avoiding checking points multiple times in recursion searches
    k_near = 0
    left_child = None               # for k-d tree (spatial sorting algorithm, faster near/nearest functions)
    right_child = None


class Axes:                         # for scalable add_to_kd_tree() function for any number of state space dimensions
    def __init__(self, axis_):
        self.axis = axis_
    axis = None
    next_ = None


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


class Settings:                # for adjusting game-play
    def __init__(self):
        pass
    robo_velocities = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]


class Dimensions:           # pywindow, obstacles, radii, theorem 38 calculations
    def __init__(self):
        pass
    window_length = 1000    # size of pywindow display
    window_width = 1000
    tree_radius = 10        # size of radius for steering point towards random vertex
    eta = tree_radius * 1.01   # size of radius in comparison for min() function in near() function
    goal_set_size = 14

    # Vertices of static obstacles
    # Algorithm works for any number of obstacles with any number of vertices
    # Only small changes here and within obstacle_generation() function
    # Pay attention to vertex order
    window = [Vertex(0, 0), Vertex(0, window_width), Vertex(window_length, window_width), Vertex(window_length, 0)]
    #obstacle1 = [Vertex(0, 455), Vertex(0, 475), Vertex(280, 475), Vertex(280, 455)]
    obs1 = [Vertex(220, 225), Vertex(220, 245), Vertex(300, 245), Vertex(300, 225)]
    obs2 = [Vertex(200, 420), Vertex(200, 490), Vertex(280, 490), Vertex(280, 420)]
    obs3 = [Vertex(700, 220), Vertex(770, 290), Vertex(725, 200), Vertex(715, 150)]
    obs4 = [Vertex(500, 820), Vertex(480, 890), Vertex(575, 890), Vertex(560, 840)]
    obs5 = [Vertex(800, 620), Vertex(880, 690), Vertex(855, 600), Vertex(815, 550)]
    obs6 = [Vertex(500, 420), Vertex(550, 490), Vertex(575, 400), Vertex(515, 350)]
    obs7 = [Vertex(280, 650), Vertex(340, 665), Vertex(360, 610), Vertex(310, 625)]
    obs8 = [Vertex(500, 420), Vertex(550, 490), Vertex(575, 400), Vertex(515, 350)]
    obs9 = [Vertex(500, 420), Vertex(550, 490), Vertex(575, 400), Vertex(515, 350)]
    obs10 = [Vertex(500, 420), Vertex(550, 490), Vertex(575, 400), Vertex(515, 350)]
    obs_list = [window, obs1, obs2, obs3, obs4, obs5, obs6, obs7, obs8, obs9, obs10]

    # For near() vertices functionality, simplified for 2-D motion planning
    unit_ball = 3.1415926
    obs1_volume = (obs1[1].y - obs1[0].y) * (obs1[2].x - obs1[1].x)
    obs2_volume = (obs2[1].y - obs2[0].y) * (obs2[2].x - obs2[1].x)
    total_space = (window_length * window_width) - (obs1_volume + obs2_volume)  # window size minus obstacle size
    gamma = 2 * sqrt(3) * sqrt(total_space / unit_ball)    # from theorem 38 RRT*

    #unit_ball = 3.1415926
    #obs_volume = 2000 * (len(obs_list) - 1)  # estimation of obstacle volume
    #total_space = (window_length * window_width) - obs_volume  # window size minus obstacle size
    #gamma = 2 * sqrt(3) * sqrt(total_space / unit_ball)  # from theorem 38 RRT*