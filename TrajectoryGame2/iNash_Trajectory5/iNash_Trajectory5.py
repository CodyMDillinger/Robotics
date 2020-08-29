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
from pywindow_funcs import init_pywindow, user_prompt, iterate_or_stop
from sampling_funcs import sample_free
from list_funcs import init_arrays, update_pt_lists
from high_level_funcs import extend_graph, perform_better_response, check_active
from path_funcs import display_paths
from pygame_simulator import run_simulator
from classes import Colors
from measurements import calculate_times
##############################################################################################################


def form_trees_and_paths():  # this is the high-level of the algorithm as described at the source linked above
    pywindow, obstacles, axis, buttons = init_pywindow('i-Nash Policy 1')   # set up pygame window, dimensions and obstacles
    start, goal_set, num_robots, robo_colors, sign, goal = user_prompt(pywindow)  # prompt for num bots, start, end positions
    print 'start', start[0].x, start[0].y, 'goal', goal[0].x, goal[0].y
    all_bots, active_bots, inactive_bots, paths, costs, paths_prev, goal_pts, path_num, new_paths, time_to_path = init_arrays(num_robots)
    k = 1; k_ = 200000                  # total attempted vertices for each robot (if user never clicks end planning)
    samp_bias = 0                       # for biased sampling. See sample_free()
    start_time = time.time()
    while k < k_:                       # main loop
        k_end = k
        new_vertices = [None] * num_robots   # get list of new vertices each k iteration
        for i in all_bots:                   # for all robots
            vert_rand, samp_bias = sample_free(paths[i], goal_set[i], samp_bias, sign[i])   # get random Vertex in pywindow
            vert_new, new_paths_ = extend_graph(vert_rand, start[i], obstacles,             # use procedure similar to RRG, with dual-tree
                                                goal_set[i], pywindow, robo_colors[i],
                                                k, axis, goal[i])
            goal_pts, new_vertices, new_paths = update_pt_lists(vert_new, goal_pts,
                                                                new_vertices, i, new_paths_,
                                                                new_paths)
            k = iterate_or_stop(pywindow, buttons, k, k_, False)  # call this now as scrappy way of seeming more asynchronous
        for i in inactive_bots:
            check_active(new_vertices, i, active_bots, inactive_bots, new_paths)    # see if any inactive bots have become active
        for i in active_bots:
            paths_prev[i] = list(paths[i])                        # save previous path list before updating list
        k = iterate_or_stop(pywindow, buttons, k, k_, False)      # call this again as scrappy way of seeming more asynchronous
        for i in active_bots:
            perform_better_response(i, active_bots, paths_prev, paths, costs, pywindow,  # check inter-robot collisions, choose low cost paths
                                    new_vertices, goal_pts, robo_colors, new_paths[i],
                                    goal[i], start[i], time_to_path)
            k = iterate_or_stop(pywindow, buttons, k, k_, False)  # call this again as scrappy way of seeming more asynchronous
        k = iterate_or_stop(pywindow, buttons, k, k_, True)
        #time.sleep(.05)   # slows overall tree growth, can be useful for some debugging
    time_waited = calculate_times(time_to_path, start_time)
    #display_paths(goal[0].paths, pywindow, Colors.black)
    #for i in range(len(goal_pts[0])):
        #display_paths(goal_pts[0][i].paths, pywindow, Colors.black)
    return pywindow, buttons, active_bots, paths, costs, robo_colors, k_end, time_to_path, time_waited
##############################################################################################################


def main():
    pywindow, buttons, active_bots, paths, costs, robo_colors, k_end, robot_times, total_time = form_trees_and_paths()
    print 'Main loop exited.'
    print 'Total number of', k_end, 'vertices per robot (including attempted vertices that resulted in static obstacle collisions)'
    print 'Amount of time taken for robots to find their first path:', robot_times
    print 'Total time waited before clicking run simulation:', total_time
    #display_paths(paths, pywindow, Colors.light_green)
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

