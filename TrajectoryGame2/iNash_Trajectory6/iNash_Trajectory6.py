#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Cody Dillinger
# This is the main .py file for a simulation of the i-Nash Policy Algorithm
# Details of this algorithm can be found at http://php.scripts.psu.edu/muz16/pdf/DJ-MZ-AR-IFAC15.pdf
# Root nodes are the starting points of each robot
# This is titled Policy TWO
# because it expands upon policy 1 by having every robot have two trees form towards each other from root and goal

import pygame, time, math, cProfile, re
from math import *
from pywindow_funcs import init_pywindow, user_prompt, iterate_or_stop
from sampling_funcs import sample_free
from list_funcs import init_arrays, update_pt_lists
from high_level_funcs import extend_graph, perform_better_response, check_active
from path_funcs import display_paths
from pygame_simulator import run_simulator
from classes import Colors, Vertex
from measurements import calculate_times, write_data_to_csv
##############################################################################################################


def form_trees_and_paths(pywindow, obstacles, axis, buttons, start, goal_set, num_robots,
                         robo_colors, sign, goal, all_bots, active_bots, inactive_bots,
                         paths, costs, paths_prev, goal_pts, path_num, new_paths, time_to_path, changed):  # this is the high-level of the algorithm as described at the source linked above
    print 'in form trees and paths. printing all input things:', start, goal_set, num_robots,\
        goal, all_bots, active_bots, inactive_bots, paths, costs, paths_prev,\
        goal_pts, path_num, new_paths, time_to_path, changed
    k = 1; k_ = 200000                  # total attempted vertices for each robot (if user never clicks end planning)
    samp_bias = 0                       # for biased sampling. See sample_free()
    start_time = time.time()
    nash_success = True
    while k < k_:                       # main loop
        k_end = k
        new_vertices = [None] * num_robots   # get list of new vertices each k iteration
        for i in all_bots:                   # for all robots
            vert_rand, samp_bias = sample_free(paths[i], goal_set[i], samp_bias, sign[i])   # get random Vertex in pywindow
            vert_new, new_paths_ = extend_graph(vert_rand, start[i], obstacles,             # use procedure similar to RRG, with dual-tree Connect
                                                goal_set[i], pywindow, robo_colors[i],
                                                k, axis, goal[i])
            goal_pts, new_vertices, new_paths = update_pt_lists(vert_new, goal_pts,
                                                                new_vertices, i, new_paths_,
                                                                new_paths)
            k = iterate_or_stop(pywindow, buttons, k, k_, False, time_to_path)  # call this now as scrappy way of seeming more asynchronous
        for i in inactive_bots:
            check_active(new_vertices, i, active_bots, inactive_bots, new_paths)    # see if any inactive bots have become active
        for i in active_bots:
            paths_prev[i] = list(paths[i])                        # save previous path list before updating list
        k = iterate_or_stop(pywindow, buttons, k, k_, False, time_to_path)      # call this again as scrappy way of seeming more asynchronous
        for i in active_bots:
            perform_better_response(i, active_bots, paths_prev, paths, changed, costs, pywindow,  # check inter-robot collisions, choose low cost paths
                                    new_vertices, goal_pts, robo_colors, new_paths[i],
                                    goal[i], start[i], time_to_path)
            k = iterate_or_stop(pywindow, buttons, k, k_, False, time_to_path)  # call this again as scrappy way of seeming more asynchronous
        k = iterate_or_stop(pywindow, buttons, k, k_, True, time_to_path)
        if time.time() - start_time > 850:
            k = k_
            nash_success = False
    time_waited = calculate_times(time_to_path, start_time)
    #display_paths(goal[0].paths, pywindow, Colors.black)
    num_paths = [None] * num_robots
    for j in range(len(goal)):
        num_paths[j] = len(goal[j].paths)
    for j in range(len(goal_pts)):
        for i in range(len(goal_pts[j])):
            num_paths[j] = num_paths[j] + len(goal_pts[j][i].paths)
    print 'num paths for all bots', num_paths
    return pywindow, buttons, active_bots, paths, costs, robo_colors, k_end, time_to_path, time_waited, num_robots, nash_success
##############################################################################################################


def run_test(pywindow, obstacles, axis, buttons, start, goal_set, num_robots, robo_colors, sign, goal, test_num):

    all_bots, active_bots, inactive_bots, paths, costs, \
        paths_prev, goal_pts, path_num, new_paths, time_to_path, changed = init_arrays(num_robots)

    not_ready = True
    while not_ready:            # start kd axis with x checking
        if axis.axis != 'x':
            axis = axis.next_
        else:
            not_ready = False

    pywindow, buttons, active_bots, paths, costs, robo_colors, k_end,\
        robot_times_costs, total_time, numbots, nash_success = \
        form_trees_and_paths(pywindow, obstacles, axis, buttons, start, goal_set, num_robots,
                             robo_colors, sign, goal, all_bots, active_bots, inactive_bots,
                             paths, costs, paths_prev, goal_pts, path_num, new_paths, time_to_path, changed)
    print 'Main loop exited.'
    print 'Total number of', k_end, 'vertices per robot (including attempted vertices that resulted in static obstacle collisions)'
    print 'That makes', k_end * numbots, 'total vertices'
    print 'Total time waited before clicking run simulation:', total_time
    print 'This makes', (k_end * numbots / total_time), 'vertices per second on average'
    print 'robot times:', robot_times_costs
    paths_ = [None] * len(paths)
    for i in range(len(paths)):
        paths_[i] = []
        for j in range(len(paths[i])):
            paths_[i].append(paths[i][j].vertex)
    robot_times = [None] * len(robot_times_costs)
    nash_time = 0
    for i in range(len(robot_times)):
        robot_times[i] = robot_times_costs[i][0].time
        if nash_time < robot_times[i]:
            nash_time = robot_times[i]
    if nash_success:
        nash_time = nash_time / numbots  # estimated nash time if this were to be calculated decentralized as intended
        print 'nash time:', nash_time
        write_data_to_csv(robot_times_costs, numbots, 'all', 'All_Data2.csv')
        write_data_to_csv(nash_time, numbots, 'Nash', 'Nash_Data2.csv')
        """if test_num > len(robo_colors):
            display_paths(paths, pywindow, robo_colors[len(robo_colors) - 1])
        else:
            display_paths(paths, pywindow, robo_colors[test_num])"""
        #run_simulator(pywindow, active_bots, paths_, costs, robo_colors, buttons)
    else:
        print 'took too long to find nash'
    return nash_success
##############################################################################################################


def main():
    pywindow, obstacles, axis, buttons = init_pywindow('i-Nash Trajectory')  # set up pygame window, dimensions and obstacles
    start, goal_set, num_robots, robo_colors, sign, goal = user_prompt(pywindow)  # prompt for num bots, start, end positions
    num_tests = input('how many tests to run? ')
    success_total = 0
    while success_total < num_tests:
        print 'running test number:', success_total + 1
        for i in range(len(start)):
            print 'start:', start[i].x, start[i].y, 'goal', goal[i].x, goal[i].y
        nash_success = run_test(pywindow, obstacles, axis, buttons, start, goal_set, num_robots, robo_colors, sign, goal, success_total)
        if nash_success:
            success_total = success_total + 1
        for i in range(len(goal)):
            goal[i] = Vertex(goal[i].x, goal[i].y, 0, 0)
            goal[i].tree_num = 2
            start[i] = Vertex(start[i].x, goal[i].y, 0, 0)
    print 'Done the whole thing, wow, excellent, nice! Super cool!'
    return
##############################################################################################################


if __name__ == '__main__':
    # rospy.init_node('GroundRobots')
    # controller = BasicRobotController()
    cProfile.run('main()')
    #for i in range(4):
    #    main()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

