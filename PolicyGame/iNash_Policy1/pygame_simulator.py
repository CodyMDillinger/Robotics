#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Cody Dillinger
# This file contains functions related to the 2-d pygame simulator

import math, time
import pygame
from classes import Colors, Vertex
from geometry_procedures import get_traj_num
##############################################################################################################


# returns discrete position similarly to get_discrete_states() in dynamics.py
def get_position(pt1, time, traj):   # difference: for a specific time, not fixed number of states with some time_step
    ux = traj.u_x1
    uy = traj.u_y1
    v1_x = pt1.x_vel + (ux * traj.ts1_x)
    v1_y = pt1.y_vel + (uy * traj.ts1_y)
    x1 = pt1.x + (pt1.x_vel * traj.ts1_x) + (.5 * ux * (traj.ts1_x ** 2))
    y1 = pt1.y + (pt1.y_vel * traj.ts1_y) + (.5 * uy * (traj.ts1_y ** 2))
    if time < traj.ts1_x:
        x = pt1.x + (pt1.x_vel * time) + (.5 * ux * (time ** 2))
    else:
        x = x1 + (v1_x * (time - traj.ts1_x)) - (.5 * ux * ((time - traj.ts1_x) ** 2))
    if time < traj.ts1_y:
        y = pt1.y + (pt1.y_vel * time) + (.5 * uy * (time ** 2))
    else:
        y = y1 + (v1_y * (time - traj.ts1_y)) - (.5 * uy * ((time - traj.ts1_y) ** 2))
    return Vertex(x, y, 0, 0)
##############################################################################################################


def get_new_path_list(robots, paths, costs):  # returns list of pts with equal time step for all robots
    final_time = max(costs)
    time_step = .05
    num_steps = int(math.ceil(final_time / time_step))
    positions_j = [None] * num_steps
    positions = [positions_j] * len(robots)
    path_pts = [None] * len(robots)            # used for current path point (position = path point + trajectory value)
    time_pts = [None] * len(robots)            # stores time at which robot is at pt in path_pts
    path_traj = [None] * len(robots)           # path trajectories will store both the path points and the trajectory indices
    for j in range(len(robots)):               # for all robots
        path_traj[j] = [None] * len(paths[j])  # initialize path trajectory list with size of paths
        for i in range(len(paths[j]) - 1):
            traj_num = get_traj_num(paths[j][i], paths[j][i + 1])
            path_traj[j][i] = [paths[j][i], traj_num]
        path_pts[j] = 0
        time_pts[j] = 0
        for i in range(num_steps):                      # for all time steps
            time_ = time_step * i
            if time_ > costs[j]:                        # if robot j arrives sooner than other robots
                positions[j][i] = positions[j][i - 1]   # then keep it at the final goalset point
            else:
                # if time > (pt in path).(trajectory for that pt).(final time of that trajectory)
                path = paths[j]
                pth_pt = path_pts[j]
                vertex = path[pth_pt]
                trajectories = vertex.trajectories
                trajectory = trajectories[path_traj[j][path_pts[j]][1]]
                if time_ - time_pts[j] > trajectory.t_f:    # if time > arrival time of trajectory between these two points
                    path_pts[j] = path_pts[j] + 1           # then update pt and time to next pt and next time
                    time_pts[j] = time_pts[j] + trajectory.t_f
                positions[j][i] = get_position(paths[j][path_pts[j]], time_ - time_pts[j], trajectory)  # return discrete states
    print 'first positions', positions[0]
    print 'second positions', positions[1]
    return positions, time_step
##############################################################################################################


def run_simulator(pywindow, robots, paths, costs, colors):    # display robot movement along paths
    positions, time_step = get_new_path_list(robots, paths, costs)   # get discrete values with same time_step for all bots
    for i in range(len(positions[0])):                        # for number of time_steps
        for j in range(len(robots)):                          # for each robot
            print 'j', j
            x = int(positions[j][i].x)
            y = int(positions[j][i].y)
            print 'x, y', x, y
            if i > 0:                                         # if not first point
                x_prev = int(positions[j][i - 1].x)           # "erase" previous point
                y_prev = int(positions[j][i - 1].y)
                pygame.draw.circle(pywindow, Colors.white, (x_prev, y_prev), 10, 0)
            pygame.draw.circle(pywindow, colors[j], (x, y), 10, 0)  # display new point
        pygame.display.flip()
        time.sleep(time_step)
    return
##############################################################################################################
