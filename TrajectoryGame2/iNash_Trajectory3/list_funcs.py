#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Cody Dillinger
# Functions related to updating or creating generic lists


def update_active(active, inactive, i):    # update the list of active robots by adding robot i
    added = False
    inactive.remove(i)                     # remove robot i from inactive list
    for j in range(len(active)):           # for all robots previously in the active list
        if active[j] > i:                  # if the element is greater than robot i
            active.insert(j, i)            # then this is where it gets inserted
            added = True
            break
    if added is False:
        active.append(i)
    return
##############################################################################################################


def init_arrays(num_bots):                   # init array sizes for easier modifications later
    all_bots = range(num_bots)
    active_bots = []                         # indices of bots that have SOME path to goal
    inactive_bots = range(num_bots)          # initially all bots are inactive
    paths = []
    paths_prev = []
    goal_pts = []
    costs = []
    path_num = []
    for i in range(num_bots):                # set of i paths, one per robot
        paths.append([])                     # to store current robot paths
        paths_prev.append([])                # to store previous paths before better_response() procedure
        goal_pts.append([])                  # to store points at goal set
        costs.append(None)                   # to store costs of paths in paths[]
    return all_bots, active_bots, inactive_bots, paths, costs, paths_prev, goal_pts, path_num
##############################################################################################################


def update_pt_lists(vertex_new, goal_pts, new_vertices, i):
    if vertex_new is not None:
        if vertex_new.at_goal_set:
            goal_pts[i].append(vertex_new)  # use for easier path_generation() later
    new_vertices[i] = vertex_new            # use for easier checking of whether bot is active
    return goal_pts, new_vertices
##############################################################################################################
