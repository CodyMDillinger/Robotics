#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Cody Dillinger
# Functions related to path generation, path optimizing, and inter-path collisions

from math import*
from classes import Dimensions, Settings, Colors, Trajectory
from trajectories import get_traj_num
from pywindow_funcs import world_to_y_plot, world_to_x_plot
# from dynamics import get_equi_time_discrete_states
# from pygame_simulator import get_position
from classes import Vertex
import pygame, math
##############################################################################################################


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


def draw_traj(window, color_, pt, traj, size):
    for i in range(len(traj.statevals) - 1):
        pygame.draw.line(window, color_, (traj.statevals[i].x, traj.statevals[i].y),
                         (traj.statevals[i + 1].x, traj.statevals[i + 1].y), size)
        pygame.draw.line(window, color_, (world_to_x_plot(traj.statevals[i].x, traj.statevals[i].x_vel)),
                         (world_to_x_plot(traj.statevals[i + 1].x, traj.statevals[i + 1].x_vel)), size)
        pygame.draw.line(window, color_, (world_to_y_plot(traj.statevals[i].y, traj.statevals[i].y_vel)),
                         (world_to_y_plot(traj.statevals[i + 1].y, traj.statevals[i + 1].y_vel)), size)
    pygame.display.flip()
    return
##############################################################################################################


def display_paths(paths, pywindow, color_):
    print 'num paths to display:', len(paths)
    for i in range(len(paths)):
        display_path(paths[i], pywindow, color_)
    return
##############################################################################################################


def display_path(pts, pywindow, color_):
    for i in range(len(pts) - 1):
        traj_num = get_traj_num(pts[i], pts[i + 1])
        draw_traj(pywindow, color_, pts[i], pts[i].trajectories[traj_num], 6)
        pygame.draw.line(pywindow, color_, (pts[i].x, pts[i].y), (pts[i + 1].x, pts[i + 1].y), 2)
        pygame.draw.line(pywindow, color_, (world_to_x_plot(pts[i].x, pts[i].x_vel)), (world_to_x_plot(pts[i + 1].x, pts[i + 1].x_vel)), 2)
        pygame.draw.line(pywindow, color_, (world_to_y_plot(pts[i].y, pts[i].y_vel)), (world_to_y_plot(pts[i + 1].y, pts[i + 1].y_vel)), 2)
    pygame.display.flip()
    return
##############################################################################################################


def path_generation(vertex, robot_root):
    paths_parents = []                             # list of paths, path = list of vertices
    costs_parents = []                             # list of costs associated with some paths
    paths = []
    costs = []
    for i in range(len(vertex.parents)):           # for all parents
        parent = vertex.parents[i]
        pths, csts = path_generation2(parent, robot_root)      # get all paths and costs for that parent
        paths_parents.append(pths)                 # append paths to path list
        costs_parents.append(csts)                 # append costs to cost list
    # separating a list of lists of paths into one list of paths:
    for i in range(len(paths_parents)):                       # for number of lists of paths (number of parents)
        traj_num = get_traj_num(vertex.parents[i], vertex)
        cost = vertex.parents[i].trajectories[traj_num].t_f   # additional_cost is arrival time of trajectory from parent
        for j in range(len(paths_parents[i])):                # for number of paths in one of those lists
            paths.append(paths_parents[i][j])                 # append that path to new total list
            costs.append(costs_parents[i][j] + cost)          # append that (parent cost) + (additional_cost) to new total cost list
    for i in range(len(paths)):                # for every path in paths
        paths[i].append(vertex)                # add the current vertex to that path
    if len(paths) == 0:                        # if this vertex is the root node
        paths = [[vertex]]                     # only "path" is itself, and has zero cost
        costs = [0.0]
    return paths, costs
##############################################################################################################


def path_generation2(vertex, robot_root):  # tree traversal. get all paths root to vertex. update vertex.path_list[[]] and vertex.costs[]
    if vertex.paths == [] or vertex.tree_num == 2:  # if paths not calculated yet, or if it's in tree2 it can have new parents so new paths
        paths, costs = path_generation(vertex, robot_root)      # then call main path generation procedure
        paths, costs, = fix_and_set_paths(vertex, paths, costs, robot_root)
    else:                      # otherwise use the paths already calculated, since tree1 only adds children
        paths = [None] * len(vertex.paths)
        costs = list(vertex.costs)
        for p in range(len(vertex.paths)):
            path = []
            for v in range(len(vertex.paths[p])):
                path.append(vertex.paths[p][v])
                if vertex.paths[p][v] == vertex:
                    break
            paths[p] = path
    return paths, costs
##############################################################################################################


def fix_and_set_paths(vertex, paths, costs, robot_root):
    paths_ = list(paths)
    costs_ = list(costs)
    num_deletions = 0
    for i in range(len(paths)):
        if paths[i][0] != robot_root:   # eliminate "paths" that don't start at the robot root
            paths_.pop(i - num_deletions)
            costs_.pop(i - num_deletions)
            num_deletions = num_deletions + 1
    del paths, costs, num_deletions
    for i in range(len(paths_)):         # update vertex objects to store these paths/costs
        vertex.add_path(paths_[i])
        vertex.add_cost(costs_[i])
    return paths_, costs_
##############################################################################################################


def find_optimal_path(paths, costs, opt_path, opt_cost, i):
    changed = False
    if len(opt_path) == 0:                              # if this is first time getting a path for robot i
        opt_path = paths[0]                             # just select first path option. optimizations happen later
        opt_cost = costs[0]
        changed = True
        print 'first path cost for robot', i,':', opt_cost
    else:
        if len(costs) != len(paths):
            print 'path and cost list not aligned?'
        for j in range(len(paths)):             # for all paths
            if costs[j] < opt_cost:             # if the cost is lower
                opt_cost = costs[j]             # then update it as the optimal one
                opt_path = paths[j]
                changed = True
                print 'more optimal path found for robot', i, ', cost:', opt_cost
                # print 'path_num:', j
                string = []
                for k in range(len(opt_path)):
                    string.append(floor(opt_path[k].x)); string.append(floor(opt_path[k].y))
                # print string
                return opt_path, opt_cost, changed     # return upon first optimal path found for computational efficiency
    return opt_path, opt_cost, changed
##############################################################################################################


def print_paths(num_robots, goal_pts, pywindow):
    repeat = False
    for i in range(num_robots):  # for all bots
        for j in range(len(goal_pts[i])):  # for all goal pts for that bot
            for P in range(len(goal_pts[i][j].paths)):  # for all points for that goal pt
                string = []
                string.append(goal_pts[i][j].costs[P])
                for F in range(len(goal_pts[i][j].paths[P])):
                    string.append(floor(goal_pts[i][j].paths[P][F].x))
                    string.append(floor(goal_pts[i][j].paths[P][F].y))
                    print 'vertex.paths for all vertices in path', P, ':', goal_pts[i][j].paths[P][F].paths
                    for F2 in range(len(goal_pts[i][j].paths[P])):
                        if goal_pts[i][j].paths[P][F] == goal_pts[i][j].paths[P][F2]:
                            repeat = True
                print 'robot', i, ', goalpt', j, ',path', P, 'cost', string
                display_path(goal_pts[i][j].paths[P], pywindow, Colors.dark_green)  # display
                print 'repeat is ', repeat
    return
##############################################################################################################


def get_time(path, robo_num, times, traj__):     # return list of times that robot will be at corresponding vertex
    if len(times) == 0:
        times.append(0)
        for i in range(len(path) - 1):                          # for all vertices in path
            traj_num = get_traj_num(path[i], path[i + 1])
            add_time = path[i].trajectories[traj_num].t_f / traj__.num_disc_vals
            for j in range(traj__.num_disc_vals - 1):           # for all discrete trajectory points between two pts
                times.append(times[i] + add_time)               # add a time value
    # else times is already calculated
    return
##############################################################################################################


def get_equi_time_discrete_states(pt1, pt2, traj):
    step = Settings.time_step * 5                # only used for collision checking, so don't need as many points
    num_steps = int(math.ceil(traj.t_f / step))  # larger time step to decrease computations
    states = [None] * num_steps
    for i in range(num_steps):
        time_ = i * step
        if time_ >= traj.t_f or i == num_steps - 1:
            states[i] = pt2
        else:
            states[i] = get_position(pt1, time_, traj)
        traj.add_statevals(states[i])
    return
##############################################################################################################


# given a set of vertices, this returns a fuller list including discrete states in between two vertices
def get_new_path(path):
    new_path = []
    for i in range(len(path) - 1):
        traj_num = get_traj_num(path[i], path[i + 1])  # returns index of trajectory for point i that leads to point i+1
        if len(path[i].trajectories[traj_num].statevals) == 0:
            get_equi_time_discrete_states(path[i], path[i + 1], path[i].trajectories[traj_num])
        for j in range(len(path[i].trajectories[traj_num].statevals) - 1):
            new_path.append(path[i].trajectories[traj_num].statevals[j])
    return new_path
##############################################################################################################


def paths_collision_free(path1, path2, i):
    collision_free = 1
    if path2 is not None:
        """for i in range(max(len(path1), len(path2)) - 1):
            if i > len(path2) - 1:
                pt2 = path2[len(path2) - 1]
                pt1 = path1[i]
                pt2_ = path2[len(path2) - 1]
                if i + 1 <= len(path1) - 1:
                    pt1_ = path1[i + 1]
                else:
                    pt1_ = path1[i]
            elif i > len(path1) - 1:
                pt1 = path1[len(path1) - 1]
                pt2 = path2[i]
                pt1_ = path1[len(path1) - 1]
                if i + 1 <= len(path2) - 1:
                    pt2_ = path2[i + 1]
                else:
                    pt2_ = path2[i]
            else:
                pt1 = path1[i]
                pt2 = path2[i]
                if i + 1 <= len(path1) - 1:
                    pt1_ = path1[i + 1]
                else:
                    pt1_ = path1[i]
                if i + 1 <= len(path2) - 1:
                    pt2_ = path2[i + 1]
                else:
                    pt2_ = path2[i]
            traj1 = get_traj_num(pt1, pt1_)
            traj2 = get_traj_num(pt2, pt2_)
            
            if abs(pt1.x - pt2.x) <= Settings.inter_robot_col_dist and abs(
                pt1.y - pt2.y) <= Settings.inter_robot_col_dist:
                collision_free = 0
                break"""
        new_path1 = get_new_path(path1)              # returns array of all vertices with trajectory points in between
        new_path2 = get_new_path(path2)
        for i in range(max(len(new_path1), len(new_path2))):     # for all of longer path
            if i > len(new_path2) - 1:               # if path2 shorter than path1 and we are iterating past the end
                pt2 = new_path2[len(new_path2) - 1]  # then use the last point of path2
            else:
                pt2 = new_path2[i]
            if i > len(new_path1) - 1:
                pt1 = new_path1[len(new_path1) - 1]
            else:
                pt1 = new_path1[i]
            if abs(pt1.x - pt2.x) <= Settings.inter_robot_col_dist and abs(  # if paths collide at this time
                pt1.y - pt2.y) <= Settings.inter_robot_col_dist:
                collision_free = 0                                           # then return collision
                break
    return collision_free
##############################################################################################################


def collision_free_path(path_check, paths_collide, i):   # see if path_check collides with any path in paths_collide
    collision_free = 1
    for k in range(len(paths_collide)):
        collision_free = paths_collision_free(path_check, paths_collide[k], i)  # call function for all comparison paths
        if collision_free == 0:
            break
    return collision_free
##############################################################################################################
