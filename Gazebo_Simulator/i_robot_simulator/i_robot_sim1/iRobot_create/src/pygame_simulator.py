#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Cody Dillinger
# This file contains functions related to the 2-d pygame simulator

import math, time
import pygame
from classes import Colors, Vertex, Settings
from trajectories import get_traj_num
from path_funcs import display_path
from pywindow_funcs import label_button, wait_for_gazebo_call
##############################################################################################################


# returns discrete position similarly to get_discrete_states() in dynamics.py
def get_position(pt1, time_, traj):   # difference: for a specific time, not fixed number of states with some time_step
    ux = traj.u_x1
    uy = traj.u_y1
    v1_x = pt1.x_vel + (ux * traj.ts1_x)
    v1_y = pt1.y_vel + (uy * traj.ts1_y)
    x1 = pt1.x + (pt1.x_vel * traj.ts1_x) + (.5 * ux * (traj.ts1_x ** 2))
    y1 = pt1.y + (pt1.y_vel * traj.ts1_y) + (.5 * uy * (traj.ts1_y ** 2))
    if time_ < traj.ts1_x:
        x_ = pt1.x + (pt1.x_vel * time_) + (.5 * ux * (time_ ** 2))
        v_x = pt1.x_vel + (ux * time_)
    else:
        x_ = x1 + (v1_x * (time_ - traj.ts1_x)) - (.5 * ux * ((time_ - traj.ts1_x) ** 2))
        v_x = v1_x - (ux * (time_ - traj.ts1_x))
    if time_ < traj.ts1_y:
        y_ = pt1.y + (pt1.y_vel * time_) + (.5 * uy * (time_ ** 2))
        v_y = pt1.y_vel + (uy * time_)
    else:
        y_ = y1 + (v1_y * (time_ - traj.ts1_y)) - (.5 * uy * ((time_ - traj.ts1_y) ** 2))
        v_y = v1_y - (uy * (time_ - traj.ts1_y))
    return Vertex(x_, y_, v_x, v_y)
##############################################################################################################


def get_new_path_list(robots, paths, costs):  # returns list of pts with equal time step for all robots
    final_time = max(costs)
    time_step = Settings.time_step
    num_steps = int(math.ceil(final_time / time_step))
    positions = [None] * len(robots)
    for i in range(len(robots)):
        positions[i] = [None] * num_steps
    path_pts = [None] * len(robots)            # used for current path point (position = path point + trajectory value)
    time_pts = [None] * len(robots)            # stores time at which robot is at pt in path_pts
    for j in range(len(robots)):               # for all robots
        #print 'getting new path list for robot', robots[j]
        path_traj= [None] * len(paths[j])
        #print 'size of path for robot', robots[j], 'is', len(paths[j])
        for i in range(len(paths[j]) - 1):
            #print 'robot', robots[j], 'vertex number', i
            traj_num = get_traj_num(paths[j][i], paths[j][i + 1])
            path_traj[i] = traj_num
        path_pts[j] = 0
        time_pts[j] = 0
        #print 'entering time_step loop. final time:', final_time
        for i in range(num_steps):                      # for all time steps
            time_ = time_step * i
            if time_ > costs[j]:                        # if robot j arrives sooner than other robots
                positions[j][i] = paths[j][len(paths[j]) - 1]  # then keep it at the final goalset point
            else:
                # if time > (pt in path).(trajectory for that pt).(final time of that trajectory)
                trajectory = paths[j][path_pts[j]].trajectories[path_traj[path_pts[j]]]
                if time_ - time_pts[j] > trajectory.t_f:      # if time > arrival time of trajectory between these two points
                    path_pts[j] = path_pts[j] + 1             # then update pt and time to next pt and next time
                    time_pts[j] = time_pts[j] + trajectory.t_f
                positions[j][i] = get_position(paths[j][path_pts[j]], time_ - time_pts[j], trajectory)  # return discrete states
    del trajectory, time_pts, path_pts, path_traj
    return positions
##############################################################################################################


def eliminate_collision_paths(paths_, costs_, robots):
    paths = list(paths_)
    costs = list(costs_)
    for i in range(len(paths_)):
        if not paths_[i]:  # if there is an empty path, cost is large or None
            paths.remove([])
            remove_none = True
            for j in robots:
                if i == j:
                    robots.remove(j)
                    costs.remove(Settings.collision_cost)
                    remove_none = False
                    break
            if remove_none is True:
                costs.remove(None)
    return paths, costs
##############################################################################################################


def run_simulator(pywindow, robots, paths_, costs_, colors, buttons):  # display robot movement along paths
    paths, costs = eliminate_collision_paths(paths_, costs_, robots)   # erase active bots that don't have coll_free path
    del paths_, costs_
    for i in range(len(paths)):
        display_path(paths[i], pywindow, colors[robots[i]])
        time.sleep(.2)
    positions = get_new_path_list(robots, paths, costs)       # get discrete values with same time_step for all bots
    for i in range(len(positions[0])):                        # for number of time_steps
        for j in range(len(robots)):                          # for each robot
            x = positions[j][i].x
            y = positions[j][i].y
            if i > 0:                                         # if not first point
                x_prev = positions[j][i - 1].x                # "erase" previous point
                y_prev = positions[j][i - 1].y
                pygame.draw.circle(pywindow, Colors.white, (int(x_prev), int(y_prev)), Settings.robo_size, 0)
            pygame.draw.circle(pywindow, colors[robots[j]], (int(x), int(y)), Settings.robo_size, 0)  # display new point
        pygame.display.flip()
        time.sleep(Settings.simulation_speed)
    print 'Done 2D simulation'
    call_gazebo = wait_for_gazebo_call(buttons)
    if call_gazebo is True:
        label_button(pywindow, buttons[1], 1, 'Running 3D', Colors.dark_green, Colors.white)
        pygame.display.flip()
        # call gazebo
    return positions, call_gazebo
##############################################################################################################
