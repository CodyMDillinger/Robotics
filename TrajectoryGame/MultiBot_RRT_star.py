#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Cody Dillinger
# This is a simulation of the i-Nash Trajectory Algorithm from http://php.scripts.psu.edu/muz16/pdf/Zhu-Otte-ICRA14.pdf

import os, sys, random, math, pygame, time
from pygame.locals import *
from math import*
from classes import Vertex, Colors, Color_
##############################################################################################################


def create_shape(pts):                           # given a set of Vertices, connect them in order
    for pt_num in range(len(pts) - 1):           # for each Vertex except the last
        pts[pt_num].obs_next = pts[pt_num + 1]   # connect the dots
    pts[len(pts) - 1].obs_next = pts[0]          # connect last Vertex back to first
    return pts
##############################################################################################################


def draw_shape(shape, pywindow):                                          # outline border of shape on pywindow
    for pt_num in range(len(shape)):                                      # for all vertices in the shape
        x1, y1 = shape[pt_num].x, shape[pt_num].y                         # point from
        x2, y2 = shape[pt_num].obs_next.x, shape[pt_num].obs_next.y       # point to
        if x1 == x2:                        # if vertical edge
            x1 = x1 - .5; x2 = x2 - .5      # show display slightly offset left
        elif y1 == y2:                      # if horizontal edge
            y1 = y1 - .5; y2 = y2 - .5      # show display slightly offset up
        pygame.draw.line(pywindow, Colors.black, (x1, y1), (x2, y2), 3)   # connect edge between them
##############################################################################################################


def obstacle_generation():    # return array of obstacles, obstacle is array of Vertex objects connected in a loop
    window = create_shape([Vertex(0, 0), Vertex(0, 700), Vertex(500, 700), Vertex(500, 0)])
    obs1 = create_shape([Vertex(0, 455), Vertex(0, 475), Vertex(280, 475), Vertex(280, 455)])
    obs2 = create_shape([Vertex(220, 225), Vertex(220, 245), Vertex(500, 245), Vertex(500, 225)])
    obstacles = [window, obs1, obs2]
    return obstacles
##############################################################################################################


def add_colors(robo_color):                      # append colors to array to use for large numbers of robot trees
    robo_color.add_color(Color_(Colors.grey))
    robo_color.add_color(Color_(Colors.dark_red))
    robo_color.add_color(Color_(Colors.turquoise))
    robo_color.add_color(Color_(Colors.purple))
    robo_color.add_color(Color_(Colors.light_orange))
    robo_color.add_color(Color_(Colors.pink))
    robo_color.add_color(Color_(Colors.light_red))
    robo_color.add_color(Color_(Colors.light_blue))
    for i in range(len(robo_color.color_list) - 1):
        robo_color.color_list[i].next_ = robo_color.color_list[i + 1]
    robo_color.color_list[len(robo_color.color_list) - 1].next_ = robo_color.color_list[0]
    return robo_color
##############################################################################################################


def color_init(num_robots_):                     # get array of colors same size as num of robots
    robo_colors = Color_((0, 0, 0))              # single color object, with array to be used for list of colors
    robo_colors = add_colors(robo_colors)        # add 9 colors connected in a loop
    colors = [None] * num_robots_
    color_ = robo_colors.color_list[0]
    for i in range(num_robots_):
        colors[i] = color_.color_
        color_ = color_.next_
    return colors
##############################################################################################################


def init_pywindow(title):                                       # first function to be called from main()
    length = 500; width = 700
    pygame.init()                                               # initialize usage of pygame
    pygame.display.set_caption(title)
    pywindow = pygame.display.set_mode((length, width))         # create pygame display
    pywindow.fill(Colors.white)                                 # set background of pygame window to white
    obstacles = obstacle_generation()
    for i in range(len(obstacles)):                             # for all obstacles
        draw_shape(obstacles[i], pywindow)                      # outline borders on pywindow
    pygame.display.flip()                                       # update display with these new shapes
    return pywindow, obstacles
##############################################################################################################


def get_click_pos(i, pt_type, pt, pywindow, color_dot):                # prompt user to click pygame window, read position
    print 'Click robot', i+1, pt_type, 'Vertex'             # prompt user
    waiting = 1; exit_pressed = 0
    while waiting:                                          # loop until user has clicked, or clicked exit
        event_click = pygame.event.poll()                   # read whether user event has occurred
        if event_click.type == pygame.QUIT:                 # if user clicked exit
            waiting = 0
            exit_pressed = 1
        elif event_click.type == pygame.MOUSEBUTTONDOWN and event_click.button == 1:  # if user clicked in pywindow
            x, y = pygame.mouse.get_pos()                                  # get position of the click
            pt.append(Vertex(x, y))                                       # array of all start/end positions
            pygame.draw.circle(pywindow, color_dot, (x, y), 10, 0)        # display starting Vertex with color circle
            pygame.display.flip()                                         # update display with these new shapes
            waiting = 0
    return exit_pressed, pt
##############################################################################################################


def user_prompt(pywindow):         # prompt for num robots, start and end positions, mark on pywindow
    num_robots = input('Enter number of robots: ')
    robo_colors = color_init(num_robots)
    start = []                     # to be used for coordinates of robots start positions
    goal_set = []                  # to be used for coordinates of robots end positions
    i = 0
    running_ = 1
    while i < num_robots and running_:                                 # until start/stop selected for all, or exited
        exit1, start = get_click_pos(i, 'start', start, pywindow, robo_colors[i])      # get start position from user, append
        start[i].parent = start[i]                                     # root node .parent is self to avoid null
        exit2, goal_set = get_click_pos(i, 'end', goal_set, pywindow, robo_colors[i])  # get end position from user, append
        i = i + 1
        if exit1 == 1 or exit2 == 1:                                   # end loop if user exited
            running_ = 0
    goal_set_size = 12
    goal_set2 = []                # updating goal set to be a larger box, rather than just a goal Vertex
    for i in range(num_robots):   # make four Vertices with goal_set at center
        pt1 = Vertex(goal_set[i].x - goal_set_size, goal_set[i].y - goal_set_size)
        pt2 = Vertex(goal_set[i].x - goal_set_size, goal_set[i].y + goal_set_size)
        pt3 = Vertex(goal_set[i].x + goal_set_size, goal_set[i].y + goal_set_size)
        pt4 = Vertex(goal_set[i].x + goal_set_size, goal_set[i].y - goal_set_size)
        goal_box = create_shape([pt1, pt2, pt3, pt4])
        goal_set2.append(goal_box)
        draw_shape(goal_box, pywindow)
    pygame.display.flip()
    return start, goal_set2, num_robots, robo_colors
##############################################################################################################


def sample_free():		         # return pseudo random Vertex object with coordinates uniformly in pywindow
    length = 500; width = 700
    x_rand = Vertex(random.random()*length, random.random()*width)
    return x_rand
##############################################################################################################


def dist(pt1, pt2):              # calculate distance between two points
    return sqrt((pt1.x - pt2.x)**2 + (pt1.y - pt2.y)**2)
##############################################################################################################


def nearest(x_rand, x_tree, x_nearest):                 # recursively search tree for nearest node
    if dist(x_rand, x_tree) < dist(x_rand, x_nearest) and x_tree.at_goal_set is False:  # if distance to next vertex in tree is smaller
        x_nearest = x_tree                              # update it as the nearest
    for i in range(len(x_tree.leaves)):                      # for all leaf vertices
        x_nearest = nearest(x_rand, x_tree.leaves[i], x_nearest)   # call nearest function again
    return x_nearest
##############################################################################################################


def cross_prod(pt1, pt2, pt3):    # get cross product of pt with line connecting two other pts
    if (pt3.y - pt1.y) * (pt2.x - pt1.x) > (pt2.y - pt1.y) * (pt3.x - pt1.x):
        prod = 1
    else: prod = 0
    return prod
##############################################################################################################


def collision_line(pt1, pt2, pt1_, pt2_):   # check if collision between two lines
    if cross_prod(pt1, pt1_, pt2_) != cross_prod(pt2, pt1_, pt2_) and cross_prod(pt1, pt2, pt1_) != cross_prod(pt1, pt2,pt2_):
        collision_ = 1
    else:
        collision_ = 0
    return collision_
##############################################################################################################


def line(pt1, pt2):                         # return slope and intercept of line, given two points
    if pt2.x == pt1.x:                      # avoid divide by zero
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
    return Vertex(x, y)
##############################################################################################################


def collision(pt1, pt2, shape, type_):         # check if pt1 to pt2 collides with any edge of shape
    collision_ = 0; intersect = None
    for i in range(len(shape)):                                         # for all edges of shape
        if collision_line(pt1, pt2, shape[i], shape[i].obs_next) == 1:  # if collision
            collision_ = 1
            if type_ == 'goal':                # if checking collision for goal set, calculate intersection pt
                intersect = intersection(pt1, pt2, shape[i], shape[i].obs_next)
                intersect.at_goal_set = True
            break

    return collision_, intersect
##############################################################################################################


def collisions(pt1, pt2, list_, type_):      # calling collision function for a list of obstacles
    collision_ = 0; intersect = None
    for i in range(len(list_)):
        collision_, intersect = collision(pt1, pt2, list_[i], type_)
        if collision_ == 1:
            break                           # end loop early if collision found
    return collision_, intersect
##############################################################################################################


def trace_inclusivity(x_nearest, x_new, goal_set):  # check that trajectory does not cross through goal set and back out
    x_new_ = x_new
    collision_, intersect = collision(x_nearest, x_new, goal_set, 'goal')
    if collision_ == 1:                             # if trajectory hits goal set
        x_new_ = intersect                          # adjust new vertex to the intersection point
    return x_new_
##############################################################################################################


def steer(x_nearest, x_rand, goal_set):      # steer nearest vertex towards random Vertex, within some radius
    radius = 6                               # arbitrarily sized radius
    theta = atan2(x_rand.y - x_nearest.y, x_rand.x - x_nearest.x)    # atan2 accounts for different quadrants
    x_new = x_nearest.x + radius*cos(theta)  # new x value
    y_new = x_nearest.y + radius*sin(theta)  # new y value
    vertex_new = Vertex(x_new, y_new)        # initialize vertex
    vertex_new_ = trace_inclusivity(x_nearest, vertex_new, goal_set)  # if hits goal, adjust vertex to intersection pt
    return vertex_new_
##############################################################################################################


def add_edge(pt_new, pt_tree, color_, size, pywindow):
    pt_new.parent = pt_tree
    pt_tree.add_leaf(pt_new)
    pygame.draw.line(pywindow, color_, (pt_new.x, pt_new.y), (pt_tree.x, pt_tree.y), size)
    pygame.display.flip()
    return
##############################################################################################################


def near_vertices(vertex_new):
    vertices_near = vertex_new
    return vertices_near
##############################################################################################################


def extend_graph(x_rand, robot_root, obstacles, goal_set, pywindow, color_):  # add vertex and edges to sets for robot i
    x_nearest = nearest(x_rand, robot_root, robot_root)     # find nearest vertex in robot i's graph, starting with root
    vertex_new = steer(x_nearest, x_rand, goal_set)         # steer that vertex towards x_rand within some radius
    collision_, intersect = collisions(x_nearest, vertex_new, obstacles, 'obstacles')
    if collision_ == 0:
        vertices_near = near_vertices(vertex_new)
        add_edge(vertex_new, x_nearest, color_, 1, pywindow)
    return vertex_new
##############################################################################################################


def main():
    pywindow, obstacles = init_pywindow('Something')                       # set up pygame window, dimensions and obstacles
    start, goal_set, num_robots, robo_colors = user_prompt(pywindow)             # prompt for num bots, start, end positions
    k = 1; k_ = 1500
    while k < k_:                                                   # main loop
        for i in range(num_robots):                                 # for all bots
            x_rand = sample_free()                                  # get random Vertex uniformly in pywindow
            x_new = extend_graph(x_rand, start[i], obstacles, goal_set[i], pywindow, robo_colors[i])
            # add vertex, edges for robot i, check static obstacle collisions
    return
##############################################################################################################


if __name__ == '__main__':
    main()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
