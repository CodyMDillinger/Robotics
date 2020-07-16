#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Cody Dillinger
# Functions related to path generation, path optimizing, and inter-path collisions

from math import*
from classes import Dimensions, Settings
from geometry_procedures import dist
from pywindow_funcs import world_to_y_plot, world_to_x_plot
import pygame
##############################################################################################################


def display_path(pts, pywindow, color_):
    for i in range(len(pts) - 1):
        pygame.draw.line(pywindow, color_, (pts[i].x, pts[i].y), (pts[i + 1].x, pts[i + 1].y), 6)
        pygame.draw.line(pywindow, color_, (world_to_x_plot(pts[i].x, pts[i].x_vel)), (world_to_x_plot(pts[i + 1].x, pts[i + 1].x_vel)), 5)
        pygame.draw.line(pywindow, color_, (world_to_y_plot(pts[i].y, pts[i].y_vel)), (world_to_y_plot(pts[i + 1].y, pts[i + 1].y_vel)), 5)
    pygame.display.flip()
    return
##############################################################################################################


def path_generation(vertex):
    #print 'running path gen 1', 'vertex parents:', vertex.parents
    paths_parents = []                             # list of paths, path = list of vertices
    costs_parents = []                             # list of costs associated with some paths
    paths = []
    costs = []
    for i in range(len(vertex.parents)):           # for all parents
        parent = vertex.parents[i]
        pths, csts = path_generation2(parent)      # get all paths and costs for that parent
        paths_parents.append(pths)                 # append paths to path list
        costs_parents.append(csts)                 # append costs to cost list
    # separating a list of lists of paths into one list of paths:
    for i in range(len(paths_parents)):                        # for number of lists of paths (number of parents)
        distance = dist(vertex, vertex.parents[i])             # calculate distance for each new parent
        for j in range(len(paths_parents[i])):                 # for number of paths in one of those lists
            paths.append(paths_parents[i][j])                  # append that path to new total list
            costs.append(costs_parents[i][j] + distance)       # append that parent cost + distance to new total cost list
    for i in range(len(paths)):                # for every path in paths
        paths[i].append(vertex)                # add the current vertex to that path
    if len(paths) == 0:                        # if this vertex is the root node
        paths = [[vertex]]                     # only "path" is itself, and has zero cost
        costs = [0]
    return paths, costs
##############################################################################################################


def path_generation2(vertex):  # tree traversal. get all paths root to vertex. update vertex.path_list[[]] and vertex.costs[]
    #print 'running path gen 2, vertex parents:', vertex.parents
    if vertex.paths == []:                          # if paths not calculated yet for this vertex
        paths, costs = path_generation(vertex)      # then call main path generation procedure
        vertex.paths = list(paths)                  # update vertex objects to store these paths/costs
        vertex.costs = list(costs)
    else:                                           # otherwise use the paths already calculated
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


def find_optimal_path(paths, costs, opt_path, opt_cost, i):
    changed = False
    if len(opt_path) == 0:                              # if this is first time getting a path for robot i
        opt_path = paths[0]                             # just select first path option. optimizations happen later
        opt_cost = costs[0]
        changed = True
        print 'first path cost:', opt_cost
        print 'first path:'
        for k in range(len(opt_path)):
            print opt_path[k].x, opt_path[k].y, ' '
    else:
        if len(costs) != len(paths):
            print 'path and cost list not aligned?'
        for j in range(len(paths)):             # for all paths
            if costs[j] < opt_cost:             # if the cost is lower
                opt_cost = costs[j]             # then update it as the optimal one
                opt_path = paths[j]
                changed = True
                print 'more optimal path found for robot', i, ', cost:', opt_cost
                print 'path_num:', j
                string = []
                for k in range(len(opt_path)):
                    string.append(floor(opt_path[k].x)); string.append(floor(opt_path[k].y))
                #print string
                return opt_path, opt_cost, changed     # return upon first optimal path found for computational efficiency
    return opt_path, opt_cost, changed
##############################################################################################################


def get_time(path, robo_num, times):  # simple version for holonomic, velocity is controlled / constant-magnitude
    if len(times) == 0:
        times.append(0)
        for i in range(len(path) - 1):
            add_time = dist(path[i], path[i + 1]) / Settings.robo_velocities[robo_num]
            times.append(times[i] + add_time)
    # else times is already calculated
    return
##############################################################################################################


def paths_collision_free(path1, path2, robo1_num, robo2_num, times1, times2):                  # check for collisions between two paths
    collision_free = 1
    j = 0
    get_time(path1, robo1_num, times1)
    if path2 is not None:
        get_time(path2, robo2_num, times2)
        for k in range(len(path1)):
            j = 0
            robo_ability = Dimensions.tree_radius / Settings.robo_velocities[robo1_num]
            #print 'times1 size:', len(times1), 'path1 size:', len(path1)
            #print 'times2 size:', len(times2), 'path2 size:', len(path2)
            ignore = False
            while times1[k] - times2[j] > robo_ability * 1.1:
                #print 'j', j
                #print 'time dif', times1[k] - times2[j]
                if j == len(times2) - 1:
                    ignore = True
                    break
                j = j + 1
            if ignore is False:
                if abs(path1[k].x - path2[j].x) < 10 and abs(path1[k].y - path2[j].y) < 10:
                    collision_free = 0
                    #print 'collision at times', times1[k], times2[j], 'and locations', path1[k].x, path2[j].x, path1[k].y, path2[j].y
                    return collision_free
    # else path2 is None so collision_free = 1
    return collision_free
##############################################################################################################


def collision_free_path(path_check, paths_collide, i):      # see if path_check collides with either path in paths_collide
    j = i - 1
    times_i = []
    times_j = []
    for k in range(2):
        collision_free = paths_collision_free(path_check, paths_collide[k], i, j, times_i, times_j)    # call function for both paths
        if collision_free == 0:
            return collision_free
        j = i + 1
        times_j = []        # reset for next bot. re-use bot i times.
    return collision_free
##############################################################################################################
