#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Cody Dillinger
# Functions related to sampling procedures

import os, sys, random, math, pygame, time
from pygame.locals import *
from math import*
from classes import Vertex, Dimensions, Settings
##############################################################################################################


def sample_free(path, goal_set, i, sign):                  # return pseudo random Vertex object
    x = random.random() * Dimensions.window_width
    y = random.random() * Dimensions.window_length
    if i == Settings.sample_bias_iterator**(-1) - 1:
        biasx = sign[0][1]
        biasy = sign[1][1]
        i = 0
    else:
        biasx = False
        biasy = False
        i = i + 1
    if biasx == False:
        x_vel = (random.random() * 2 * Settings.robo_vel_max) - Settings.robo_vel_max
    else:
        x_vel = random.random() * Settings.robo_vel_max * sign[0][0]  # bias velocity sampling
    if biasy == False:
        y_vel = (random.random() * 2 * Settings.robo_vel_max) - Settings.robo_vel_max
    else:
        y_vel = random.random() * Settings.robo_vel_max * sign[1][0]   # to help avoid swirling paths
    return Vertex(x, y, x_vel, y_vel), i
##############################################################################################################


# currently un-used
def get_bias_box(goal_x, goal_y):  # get a box within pygame window centered around goal set, for biasing random sample
    w = .5 * sqrt(2) * Dimensions.window_width      # width of box
    h = .5 * sqrt(2) * Dimensions.window_length     # height of box
    if goal_x + w / 2 > Dimensions.window_width:    # if centering around goalset would push box out of pywindow ?
        x_r = Dimensions.window_width
        x_l = Dimensions.window_width - w
    elif goal_x - w / 2 < 0:
        x_l = 0
        x_r = w
    else:
        x_r = goal_x + w / 2
        x_l = goal_x - w / 2
    if goal_y + h / 2 > Dimensions.window_length:
        y_large = Dimensions.window_length
        y_small = Dimensions.window_length - h
    elif goal_y - h / 2 < 0:
        y_small = 0
        y_large = h
    else:
        y_small = goal_y - h / 2
        y_large = goal_y + h / 2
    return x_l, x_r, y_small, y_large             # return x and y vals of the 4 edges of box
##############################################################################################################
