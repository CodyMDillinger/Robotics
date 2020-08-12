#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Cody Dillinger
# This is the main .py file for a simulation of the i-Nash Policy Algorithm
# Details of this algorithm can be found at http://php.scripts.psu.edu/muz16/pdf/DJ-MZ-AR-IFAC15.pdf
# Root nodes are the starting points of each robot

import pygame, time, math, roslib, rospy
from controller import BasicRobotController
from math import *
from classes import Colors
from pywindow_funcs import init_pywindow, user_prompt, label_button, iterate_or_stop
from sampling_funcs import sample_free
from list_funcs import init_arrays, update_pt_lists
from pygame_simulator import run_simulator
from high_level_funcs import extend_graph, perform_better_response, check_active
##############################################################################################################


def form_trees_and_paths():  # this is the high-level of the algorithm as described at the source linked above
    pywindow, obstacles, axis, buttons = init_pywindow(
        'i-Nash Policy 1')   # set up pygame window, dimensions and obstacles
    start, goal_set, num_robots, robo_colors, sign = user_prompt(pywindow)  # prompt for num bots, start, end positions
    all_bots, active_bots, inactive_bots, paths, costs, paths_prev, goal_pts, path_num = init_arrays(num_robots)
    k = 1; k_ = 4 * 75000               # total attempted vertices for each robot / 4
    samp_bias = 0                       # for biased sampling. See sample_free()
    while k < k_:                       # main loop
        new_vertices = [None] * num_robots   # get list of new vertices each k iteration
        for i in all_bots:              # for all robots
            vert_rand, samp_bias = sample_free(paths[i], goal_set[i], samp_bias,
                                               sign[i])    # get random Vertex in pywindow
            vert_new = extend_graph(vert_rand, start[i], obstacles, goal_set[i], pywindow, robo_colors[i], k, axis)
            goal_pts, new_vertices = update_pt_lists(vert_new, goal_pts, new_vertices, i)
            k = iterate_or_stop(pywindow, buttons, k, k_)  # call this now as scrappy way of seeming more asynchronous
        for i in inactive_bots:
            check_active(new_vertices, i, active_bots, inactive_bots)
        for i in active_bots:
            paths_prev[i] = list(paths[i])                 # save previous path list before updating list
        k = iterate_or_stop(pywindow, buttons, k, k_)      # call this now as scrappy way of seeming more asynchronous
        for i in active_bots:
            perform_better_response(i, active_bots, paths_prev, paths, costs, pywindow, new_vertices, goal_pts,
                                    robo_colors)
            k = iterate_or_stop(pywindow, buttons, k, k_)  # call this now as scrappy way of seeming more asynchronous
        # time.sleep(.05)
    return pywindow, buttons, active_bots, paths, costs, robo_colors
##############################################################################################################


def measurement(robot_number):
    if robot_number == 1:
        pose = controller.robot1()
    elif robot_number == 2:
        pose = controller.robot2()
    elif robot_number == 3:
        pose = controller.robot3()
    elif robot_number == 4:
        pose = controller.robot4()
    x = pose[0]
    y = pose[1]
    xr = pose[3]
    yr = pose[4]
    zr = pose[5]
    w = pose[6]
    yaw = atan2(2 * (w * zr + xr * yr), 1 - 2 * (yr * yr + zr * zr))  # Get the yaw euler angle from quaternion
    return x, y, yaw
##############################################################################################################


def angle_adjustment(angle):
    if angle > 3.14:
        angle = angle - 7.28
    elif angle < -3.14:
        angle = angle + 7.28
    return angle
##############################################################################################################


def move1(x, y):
    robot1_x, robot1_y, yaw = measurement(1)
    angle = atan2(y - robot1_y, x - robot1_x)
    angle_difference = angle - yaw
    angle_difference = angle_adjustment(angle_difference)
    while abs(angle_difference) > 0.05:
        robot1_x, robot1_y, yaw = measurement(1)
        angle = atan2(y - robot1_y, x - robot1_x)
        angle_difference = angle - yaw
        angle_difference = angle_adjustment(angle_difference)
        controller.SetCommand1(0, 2 * angle_difference)
    distance = sqrt((x - robot1_x) * (x - robot1_x) + (y - robot1_y) * (y - robot1_y))
    while distance > 0.05:
        robot1_x, robot1_y, yaw = measurement(1)
        angle = atan2(y - robot1_y, x - robot1_x)
        angle_difference = angle - yaw
        angle_difference = angle_adjustment(angle_difference)
        distance = sqrt((x - robot1_x) * (x - robot1_x) + (y - robot1_y) * (y - robot1_y))
        controller.SetCommand1(2 * distance, 2 * angle_difference)
    controller.SetCommand1(0, 0)
    time.sleep(0.1)
##############################################################################################################


def move2(x, y):
    robot2_x, robot2_y, yaw = measurement(2)
    angle = atan2(y - robot2_y, x - robot2_x)
    angle_difference = angle - yaw
    angle_difference = angle_adjustment(angle_difference)
    while abs(angle_difference) > 0.05:
        robot2_x, robot2_y, yaw = measurement(2)
        angle = atan2(y - robot2_y, x - robot2_x)
        angle_difference = angle - yaw
        angle_difference = angle_adjustment(angle_difference)
        controller.SetCommand2(0, 2 * angle_difference)
    distance = sqrt((x - robot2_x) * (x - robot2_x) + (y - robot2_y) * (y - robot2_y))
    while distance > 0.05:
        robot2_x, robot2_y, yaw = measurement(2)
        angle = atan2(y - robot2_y, x - robot2_x)
        angle_difference = angle - yaw
        angle_difference = angle_adjustment(angle_difference)
        distance = sqrt((x - robot2_x) * (x - robot2_x) + (y - robot2_y) * (y - robot2_y))
        controller.SetCommand2(2 * distance, 2 * angle_difference)
    controller.SetCommand2(0, 0)
    time.sleep(0.1)
##############################################################################################################


def move3(x, y):
    robot3_x, robot3_y, yaw = measurement(3)
    angle = atan2(y - robot3_y, x - robot3_x)
    angle_difference = angle - yaw
    while abs(angle_difference) > 0.05:
        robot3_x, robot3_y, yaw = measurement(3)
        angle = atan2(y - robot3_y, x - robot3_x)
        angle_difference = angle - yaw
        angle_difference = angle_adjustment(angle_difference)
        controller.SetCommand3(0, 2 * angle_difference)
    distance = sqrt((x - robot3_x) * (x - robot3_x) + (y - robot3_y) * (y - robot3_y))
    while distance > 0.05:
        robot3_x, robot3_y, yaw = measurement(3)
        angle = atan2(y - robot3_y, x - robot3_x)
        angle_difference = angle - yaw
        angle_difference = angle_adjustment(angle_difference)
        distance = sqrt((x - robot3_x) * (x - robot3_x) + (y - robot3_y) * (y - robot3_y))
        controller.SetCommand3(2 * distance, 2 * angle_difference)
    controller.SetCommand3(0, 0)
    time.sleep(0.1)
##############################################################################################################


def move4(x, y):
    robot4_x, robot4_y, yaw = measurement(4)
    angle = atan2(y - robot4_y, x - robot4_x)
    angle_difference = angle - yaw
    angle_difference = angle_adjustment(angle_difference)
    while abs(angle_difference) > 0.05:
        robot4_x, robot4_y, yaw = measurement(4)
        angle = atan2(y - robot4_y, x - robot4_x)
        angle_difference = angle - yaw
        angle_difference = angle_adjustment(angle_difference)
        controller.SetCommand4(0, 2 * angle_difference)
    distance = sqrt((x - robot4_x) * (x - robot4_x) + (y - robot4_y) * (y - robot4_y))
    while distance > 0.05:
        robot4_x, robot4_y, yaw = measurement(4)
        angle = atan2(y - robot4_y, x - robot4_x)
        angle_difference = angle - yaw
        angle_difference = angle_adjustment(angle_difference)
        distance = sqrt((x - robot4_x) * (x - robot4_x) + (y - robot4_y) * (y - robot4_y))
        controller.SetCommand4(2 * distance, 2 * angle_difference)
    controller.SetCommand4(0, 0)
    time.sleep(0.1)
##############################################################################################################


def move(x1, y1, x2, y2, x3, y3, x4, y4):
    robot1_x, robot1_y, yaw1 = measurement(1)
    robot2_x, robot2_y, yaw2 = measurement(2)
    robot3_x, robot3_y, yaw3 = measurement(3)
    robot4_x, robot4_y, yaw4 = measurement(4)

    angle1 = atan2(y1 - robot1_y, x1 - robot1_x)
    angle_difference1 = angle1 - yaw1
    angle2 = atan2(y2 - robot2_y, x2 - robot2_x)
    angle_difference2 = angle2 - yaw2
    angle3 = atan2(y3 - robot3_y, x3 - robot3_x)
    angle_difference3 = angle3 - yaw3
    angle4 = atan2(y4 - robot4_y, x4 - robot4_x)
    angle_difference4 = angle4 - yaw4

    angle_difference1 = angle_adjustment(angle_difference1)
    angle_difference2 = angle_adjustment(angle_difference2)
    angle_difference3 = angle_adjustment(angle_difference3)
    angle_difference4 = angle_adjustment(angle_difference4)

    while abs(angle_difference1) > 0.05 or abs(angle_difference2) > 0.05 or abs(angle_difference3) > 0.05 or abs(
            angle_difference4) > 0.05:
        robot1_x, robot1_y, yaw1 = measurement(1)
        robot2_x, robot2_y, yaw2 = measurement(2)
        robot3_x, robot3_y, yaw3 = measurement(3)
        robot4_x, robot4_y, yaw4 = measurement(4)

        angle1 = atan2(y1 - robot1_y, x1 - robot1_x)
        angle_difference1 = angle1 - yaw1
        angle2 = atan2(y2 - robot2_y, x2 - robot2_x)
        angle_difference2 = angle2 - yaw2
        angle3 = atan2(y3 - robot3_y, x3 - robot3_x)
        angle_difference3 = angle3 - yaw3
        angle4 = atan2(y4 - robot4_y, x4 - robot4_x)
        angle_difference4 = angle4 - yaw4

        angle_difference1 = angle_adjustment(angle_difference1)
        angle_difference2 = angle_adjustment(angle_difference2)
        angle_difference3 = angle_adjustment(angle_difference3)
        angle_difference4 = angle_adjustment(angle_difference4)

        controller.SetCommand1(0, 2 * angle_difference1)
        controller.SetCommand2(0, 2 * angle_difference2)
        controller.SetCommand3(0, 2 * angle_difference3)
        controller.SetCommand4(0, 2 * angle_difference4)

    distance1 = sqrt((x1 - robot1_x) * (x1 - robot1_x) + (y1 - robot1_y) * (y1 - robot1_y))
    distance2 = sqrt((x2 - robot2_x) * (x2 - robot2_x) + (y2 - robot2_y) * (y2 - robot2_y))
    distance3 = sqrt((x3 - robot3_x) * (x3 - robot3_x) + (y3 - robot3_y) * (y3 - robot3_y))
    distance4 = sqrt((x4 - robot4_x) * (x4 - robot4_x) + (y4 - robot4_y) * (y4 - robot4_y))

    while distance1 > 0.05 or distance2 > 0.05 or distance3 > 0.05 or distance4 > 0.05:
        robot1_x, robot1_y, yaw1 = measurement(1)
        robot2_x, robot2_y, yaw2 = measurement(2)
        robot3_x, robot3_y, yaw3 = measurement(3)
        robot4_x, robot4_y, yaw4 = measurement(4)

        angle1 = atan2(y1 - robot1_y, x1 - robot1_x)
        angle_difference1 = angle1 - yaw1
        angle2 = atan2(y2 - robot2_y, x2 - robot2_x)
        angle_difference2 = angle2 - yaw2
        angle3 = atan2(y3 - robot3_y, x3 - robot3_x)
        angle_difference3 = angle3 - yaw3
        angle4 = atan2(y4 - robot4_y, x4 - robot4_x)
        angle_difference4 = angle4 - yaw4

        angle_difference1 = angle_adjustment(angle_difference1)
        angle_difference2 = angle_adjustment(angle_difference2)
        angle_difference3 = angle_adjustment(angle_difference3)
        angle_difference4 = angle_adjustment(angle_difference4)

        distance1 = sqrt((x1 - robot1_x) * (x1 - robot1_x) + (y1 - robot1_y) * (y1 - robot1_y))
        distance2 = sqrt((x2 - robot2_x) * (x2 - robot2_x) + (y2 - robot2_y) * (y2 - robot2_y))
        distance3 = sqrt((x3 - robot3_x) * (x3 - robot3_x) + (y3 - robot3_y) * (y3 - robot3_y))
        distance4 = sqrt((x4 - robot4_x) * (x4 - robot4_x) + (y4 - robot4_y) * (y4 - robot4_y))

        controller.SetCommand1(2 * distance1, 2 * angle_difference1)
        controller.SetCommand2(2 * distance2, 2 * angle_difference2)
        controller.SetCommand3(2 * distance3, 2 * angle_difference3)
        controller.SetCommand4(2 * distance4, 2 * angle_difference4)
    time.sleep(0.1)
##############################################################################################################


def control_robots(trajectories):
    running_ = True
    while running_:
        move(1, 1, 1, -1, -1, -1, -1, 1)
        move(-1, 1, 1, 1, 1, -1, -1, -1)
        move(-1, -1, -1, 1, 1, 1, 1, -1)
        move(1, -1, -1, -1, -1, 1, 1, 1)
    return
##############################################################################################################


def main():
    pywindow, buttons, active_bots, paths, costs, robo_colors = form_trees_and_paths()
    print 'main loop exited'
    label_button(pywindow, buttons[0], 0, 'Running 2D', Colors.dark_green, Colors.white)
    pygame.display.flip()
    trajectories, call_gazebo = run_simulator(pywindow, active_bots, paths, costs, robo_colors, buttons)
    if call_gazebo is True:
        control_robots(trajectories)
    return
##############################################################################################################


if __name__ == '__main__':
    rospy.init_node('GroundRobots')
    controller = BasicRobotController()
    main()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

