#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Cody Dillinger
# Functions related to geometry and collisions with static obstacles

from math import*
from classes import Vertex, Dimensions, Settings
##############################################################################################################


def steer4(vertex_nearest, vertex_rand, goal_set):    # steer nearest vertex towards random Vertex, within radius
    if norm(vertex_nearest, vertex_rand) < Dimensions.tree_radius:  # if already within radius
        vertex_new = vertex_rand                                    # then no calculation needed
    else:
        ratio1 = (vertex_rand.x - vertex_nearest.x) / (vertex_rand.y - vertex_nearest.y)          # calculate 4-Dimensional "slopes"
        ratio2 = (vertex_rand.x - vertex_nearest.x) / (vertex_rand.x_vel - vertex_nearest.x_vel)
        ratio3 = (vertex_rand.x - vertex_nearest.x) / (vertex_rand.y_vel - vertex_nearest.y_vel)
        new_delta_x = sqrt((Dimensions.tree_radius ** 2) / (1 + (1 / (ratio1 ** 2)) + (1 / (ratio2 ** 2)) + (1 / (ratio3 ** 2))))   # use Euclidean norm to solve
        if vertex_rand.x < vertex_nearest.x:
            new_delta_x = -new_delta_x
        y_new = vertex_nearest.y + (new_delta_x / ratio1)
        x_v_new = vertex_nearest.x_vel + (new_delta_x / ratio2)
        y_v_new = vertex_nearest.y_vel + (new_delta_x / ratio3)
        vertex_new = Vertex(vertex_nearest.x + new_delta_x, y_new, x_v_new, y_v_new)   # initialize vertex
    #print 'vert new before trace inclusivity', vertex_new.x, vertex_new.y, vertex_new.x_vel, vertex_new.y_vel
    vertex_new_ = trace_inclusivity(vertex_nearest, vertex_new, goal_set)              # if hits goal, adjust vertex to intersection pt
    #print 'vert new after trace inclusivity', vertex_new_.x, vertex_new_.y, vertex_new_.x_vel, vertex_new_.y_vel
    return vertex_new_
##############################################################################################################


def steer(vertex_nearest, vertex_rand, goal_set):             # steer nearest vertex towards random Vertex, within some radius
    if norm(vertex_nearest, vertex_rand) < Dimensions.tree_radius:
        vertex_new = vertex_rand
    else:
        theta = atan2(vertex_rand.y - vertex_nearest.y, vertex_rand.x - vertex_nearest.x)   # atan2 accounts for different quadrants
        x_new = vertex_nearest.x + Dimensions.tree_radius * cos(theta)     # new x value
        y_new = vertex_nearest.y + Dimensions.tree_radius * sin(theta)     # new y value
        vertex_new = Vertex(x_new, y_new)                                  # initialize vertex
    vertex_new_ = trace_inclusivity(vertex_nearest, vertex_new, goal_set)  # if hits goal, adjust vertex to intersection pt
    return vertex_new_
##############################################################################################################


def cross_prod(pt1, pt2, pt3):    # get cross product of pt with line connecting two other pts
    #print 'nearest in cross_prod', pt1.x, pt1.y, pt1.x_vel, pt1.y_vel
    if (pt3.y - pt1.y) * (pt2.x - pt1.x) > (pt2.y - pt1.y) * (pt3.x - pt1.x):
        prod_ = 1
    else:
        prod_ = 0
    return prod_
##############################################################################################################


def collision_line(pt1, pt2, pt1_, pt2_):   # check if collision between two lines
    #print 'nearest in collision line', pt1.x, pt1.y, pt1.x_vel, pt1.y_vel
    if cross_prod(pt1, pt1_, pt2_) != cross_prod(pt2, pt1_, pt2_) and cross_prod(pt1, pt2, pt1_) != cross_prod(pt1, pt2, pt2_):
        collision_ = 1
    else:
        collision_ = 0
    return collision_
##############################################################################################################


def line(pt1, pt2):                      # return slope and intercept of 2-D line, given two points
    if pt2.x == pt1.x:                   # avoid divide by zero
        m = None
        b = None
    else:
        m = (pt2.y - pt1.y) / (pt2.x - pt1.x)   # slope
        b = pt1.y - (m * pt1.x)                 # intercept
    return m, b
##############################################################################################################


def intersection(pt1, pt2, pt3, pt4):     # find intersection point of two lines, given two points for each line
    m, b = line(pt1, pt2)                 # get line values from 2 pts
    m2, b2 = line(pt3, pt4)
    if m is not None and m2 is not None:  # for non vertical lines
        x = (b2 - b) / (m - m2)           # calculate x value
        y = (m * x) + b                   # calculate y value
    else:
        if m2 is None:                    # if line 2 was the vertical one
            x = pt3.x
            y = (m * x) + b
        else:                             # else line 1 was the vertical one
            x = pt1.x
            y = (m2 * x) + b2
    return x, y
##############################################################################################################


def collision(pt1, pt2, shape, type_):         # check if pt1 to pt2 collides with any edge of shape
    #print 'nearest in collision', pt1.x, pt1.y, pt1.x_vel, pt1.y_vel
    collision_ = 0; intersect = None
    if type_ == 'obstacles':
        for i in range(len(shape)):  # for all edges of shape
            if collision_line(pt1, pt2, shape[i], shape[i].obs_next) == 1:  # if collision
                collision_ = 1
                break
    elif type_ == 'goal':
        if abs(pt2.x_vel) < Settings.robo_finish_vel and abs(pt2.y_vel) < Settings.robo_finish_vel:
            if shape[0].x <= pt2.x <= shape[2].x and shape[0].y <= pt2.y <= shape[1].y:
                pt2.at_goal_set = True
                collision_ = 1
                intersect = pt2
    else:
        print 'unknown collision check type'
    return collision_, intersect
##############################################################################################################


def collisions(pt1, pt2, list_, type_):      # calling collision function for a list of obstacles
    collision_ = 0; intersect = None
    #print 'nearest in collisions b4 collision', pt1.x, pt1.y, pt1.x_vel, pt1.y_vel
    for i in range(len(list_)):
        collision_, intersect = collision(pt1, pt2, list_[i], type_)
        if collision_ == 1:
            return collision_, intersect     # end loop early if collision found
    if abs(pt2.y_vel) > Settings.robo_vel_max or abs(pt2.x_vel) > Settings.robo_vel_max:
        collision_ = 1
    return collision_, intersect
##############################################################################################################


def trace_inclusivity(x_nearest, x_new, goal_set):  # check that trajectory does not cross through goal set and back out
    collision_, intersect = collision(x_nearest, x_new, goal_set, 'goal')
    if collision_ == 1:                             # if trajectory hits goal set
        x_new = intersect                           # adjust new vertex to the intersection point
    return x_new
##############################################################################################################


def norm(pt1, pt2):    # calculate Euclidean norm of vector between two vertices
    return sqrt((pt1.x - pt2.x)**2 + (pt1.y - pt2.y)**2 + (pt1.x_vel - pt2.x_vel)**2 + (pt1.y_vel - pt2.y_vel)**2)
##############################################################################################################


def dist(pt1, pt2):    # calculate Euclidean distance between two x,y positions
    return sqrt((pt1.x - pt2.x)**2 + (pt1.y - pt2.y)**2)
##############################################################################################################


def vel_magnitude(pt):    # calculate Euclidean distance between two x,y positions
    return sqrt((pt.x_vel ** 2) + (pt.y_vel ** 2))
##############################################################################################################







