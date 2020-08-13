#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Cody Dillinger
# This is the main .py file for a simulation of the i-Nash Policy Algorithm
# Details of this algorithm can be found at http://php.scripts.psu.edu/muz16/pdf/DJ-MZ-AR-IFAC15.pdf
# Root nodes are the starting points of each robot
# This is titled Policy TWO
#   because it expands upon policy 1 by having every robot have two trees form towards each other from root and goal

import pygame, time, math
from math import *
from classes import Colors
from pywindow_funcs import init_pywindow, user_prompt, label_button, iterate_or_stop
from sampling_funcs import sample_free
from list_funcs import init_arrays, update_pt_lists
from pygame_simulator import run_simulator
from high_level_funcs import extend_graph, perform_better_response, check_active
##############################################################################################################


def form_trees_and_paths():  # this is the high-level of the algorithm as described at the source linked above
    pywindow, obstacles, axis, buttons = init_pywindow('i-Nash Policy 1')   # set up pygame window, dimensions and obstacles
    start, goal_set, num_robots, robo_colors, sign, goal = user_prompt(pywindow)  # prompt for num bots, start, end positions
    print 'start', start[0].x, start[0].y, 'goal', goal[0].x, goal[0].y
    all_bots, active_bots, inactive_bots, paths, costs, paths_prev, goal_pts, path_num, new_paths = init_arrays(num_robots)
    k = 1; k_ = 200000                  # total attempted vertices for each robot (if user never clicks end planning)
    samp_bias = 0                       # for biased sampling. See sample_free()
    while k < k_:                       # main loop
        k_end = k
        new_vertices = [None] * num_robots   # get list of new vertices each k iteration
        for i in all_bots:              # for all robots
            vert_rand, samp_bias = sample_free(paths[i], goal_set[i], samp_bias, sign[i])    # get random Vertex in pywindow
            vert_new, new_paths_ = extend_graph(vert_rand, start[i], obstacles, goal_set[i], pywindow, robo_colors[i], k, axis, goal[i])
            goal_pts, new_vertices, new_paths = update_pt_lists(vert_new, goal_pts, new_vertices, i, new_paths_, new_paths)
            k = iterate_or_stop(pywindow, buttons, k, k_, False)  # call this now as scrappy way of seeming more asynchronous
        for i in inactive_bots:
            check_active(new_vertices, i, active_bots, inactive_bots, new_paths)
        for i in active_bots:
            paths_prev[i] = list(paths[i])                 # save previous path list before updating list
        k = iterate_or_stop(pywindow, buttons, k, k_, False)      # call this again as scrappy way of seeming more asynchronous
        for i in active_bots:
            perform_better_response(i, active_bots, paths_prev, paths, costs, pywindow, new_vertices, goal_pts,
                                    robo_colors, new_paths[i], goal[i], start[i])
            k = iterate_or_stop(pywindow, buttons, k, k_, False)  # call this again as scrappy way of seeming more asynchronous
        k = iterate_or_stop(pywindow, buttons, k, k_, True)
        #time.sleep(.05)
    return pywindow, buttons, active_bots, paths, costs, robo_colors, k_end
##############################################################################################################


def main():
    pywindow, buttons, active_bots, paths, costs, robo_colors, k_end = form_trees_and_paths()
    print 'Main loop exited.'
    print 'Used', k_end, 'vertices per robot (minus any attempted that resulted in static obstacle collisions).'
    run_simulator(pywindow, active_bots, paths, costs, robo_colors, buttons)
    print 'Done the whole thing, wow, excellent, nice! Super cool!'
    return
##############################################################################################################


if __name__ == '__main__':
    #rospy.init_node('GroundRobots')
    #controller = BasicRobotController()
    main()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

