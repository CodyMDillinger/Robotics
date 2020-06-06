#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Cody Dillinger


class Vertex:
    def __init__(self, x_val, y_val):  # initialize with x, y coordinates
        self.x = x_val
        self.y = y_val
        self.leaves = []

    def add_leaf(self, leaf):
        self.leaves.append(leaf)

    x = 0
    y = 0
    parent = None
    obs_next = None         # for shape edges (obstacles and goal set)
    cost = 0
    at_goal_set = False


class Colors:               # RGB color values
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
