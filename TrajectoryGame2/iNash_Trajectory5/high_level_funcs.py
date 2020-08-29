#!/usr/bin/env python
# -*- coding: utf-8 -*-
# This contains the high level functions called from iNash_policy. The main functions.

import pygame, time
from list_funcs import update_active
from dynamics import solve_bvp_4d
from geometry_procedures import collisions, steer4, norm
from search_algorithms import nearest2, add_to_kd_tree, near_vertices2
from path_funcs import path_generation2, find_optimal_path, collision_free_path, display_path, print_paths
from classes import Settings, Colors
##############################################################################################################


def add_edge(pt1, pt2, color_, size, pywindow):  # update children and parents for new connections
    pt1.add_parent(pt2)                          # update parent list of new vertex
    pt2.add_child(pt1)                           # update child list of vertex that the new one is connecting to
    trajectory1 = solve_bvp_4d(pt2, pt1)         # calculate dynamically feasible trajectory btwn two points
    # traj1_states uses global number of states between points
    # traj1_states2 uses global step_time for states between points
    # trajectory2 = solve_bvp_4d(pt_new, pt_tree)
    pt2.add_trajectory(trajectory1)
    # pt_tree.add_trajectory(trajectory2)
    #pygame.draw.line(pywindow, color_, (pt1.x, pt1.y), (pt2.x, pt2.y), size)
    # draw_traj(pywindow, color_, pt_new, trajectory1, size)
    #pygame.display.flip()
    return
##############################################################################################################


def extend_graph(vertex_rand, robot_root, obstacles, goal_set, pywindow, color_, k, x, goal):   # add vertex and edges to sets for robot i
    if k == 1:
        add_to_kd_tree(goal, robot_root, x)
    new_paths = False  # initialize
    if k % 2 == 0:     # nearest is from tree1 from root
        tree_num = 1
        first = robot_root
    else:              # nearest is from tree2 from goal (two trees growing towards each other for increased path variety)
        tree_num = 2
        first = goal
    vertex_nearest2 = nearest2(vertex_rand, robot_root, robot_root, first, x, tree_num)
    vertex_new2 = steer4(vertex_nearest2, vertex_rand, goal_set, tree_num)
    collision_, intersect = collisions(vertex_nearest2, vertex_new2, obstacles, 'obstacles')
    if collision_ == 0:
        if tree_num == 1:
            add_edge(vertex_new2, vertex_nearest2, color_, 1, pywindow)  # edge direction depends on which tree nearest vertex is in
        else:
            add_edge(vertex_nearest2, vertex_new2, Colors.dark_red, 1, pywindow)  # edge direction depends on which tree nearest vertex is in
        vertices_near2, radius = near_vertices2(vertex_new2, robot_root, robot_root, k, [], x, tree_num)  # computationally efficient function
        for i in range(len(vertices_near2)):                                            # for all near vertices
            if vertices_near2[i] != vertex_nearest2:                                    # since already added nearest
                collision_, intersect = collisions(vertices_near2[i], vertex_new2, obstacles, 'obstacles')
                if collision_ == 0:                                                     # if no collision
                    if tree_num == 1:
                        add_edge(vertex_new2, vertices_near2[i], color_, 1, pywindow)   # edge direction depends on which tree near vertex is in
                    else:
                        add_edge(vertices_near2[i], vertex_new2, Colors.dark_red, 1, pywindow)   # edge direction depends on which tree near vertex is in
        add_to_kd_tree(vertex_new2, robot_root, x)        # kd tree for spatial sorting and faster nearest/near searching
        if tree_num == 1:
            tree_num_other = 2
            first = goal
        else:
            tree_num_other = 1
            first = robot_root
        vertex_nearest_opposite_tree = nearest2(vertex_new2, robot_root, robot_root, first, x, tree_num_other)
        if norm(vertex_new2, vertex_nearest_opposite_tree) <= radius:
            new_paths = True
            if tree_num == 1:    # if new point is in tree 1
                add_edge(vertex_nearest_opposite_tree, vertex_new2, color_, 1, pywindow)   # new point is parent in edge between trees
            else:                # if new point is in tree 2
                add_edge(vertex_new2, vertex_nearest_opposite_tree, Colors.dark_red, 1, pywindow)   # new point is child in edge between trees
    else:
        del vertex_new2                                   # save storage if point not used
        vertex_new2 = None
    return vertex_new2, new_paths
##############################################################################################################


# minimize cost while avoiding collisions with paths in pi_
def better_response(pi_, goalpts, vertex, path_prev_i,
                    pywindow, costs_i, i, color_, new_paths,
                    goal, root, time_to_path):
    if vertex is not None:
        if vertex.at_goal_set and vertex.tree_num == 1:                    # only obtain new paths if new pt is at goal
            path_list_vertex, cost_list_vertex = path_generation2(vertex, root)  # updates vertex objects to contain path/cost list
        elif new_paths is True:
            path_list_vertex, cost_list_vertex = path_generation2(goal, root)    # if new path has formed between trees,
    collision_free_paths = []
    costs = []
    for j in range(len(goalpts) + 1):
        if j == len(goalpts):
            #print 'number of paths from exact goal point', len(goal.paths)
            for k in range(len(goal.paths)):
                if collision_free_path(goal.paths[k], pi_, i) == 1:
                    collision_free_paths.append(goal.paths[k])
                    costs.append(goal.costs[k])
        else:
            for k in range(len(goalpts[j].paths)):
                if collision_free_path(goalpts[j].paths[k], pi_, i) == 1:
                    collision_free_paths.append(goalpts[j].paths[k])
                    costs.append(goalpts[j].costs[k])
    if len(collision_free_paths) == 0:
        print 'paths exist but no inter-robot-collision-free paths for robot', i, 'yet'
        return [], Settings.collision_cost
    optimal_path = list(path_prev_i)
    optimal_cost = costs_i
    optimal_path, optimal_cost, changed = find_optimal_path(collision_free_paths, costs, optimal_path, optimal_cost, i)
    if path_prev_i == [] and len(optimal_path) > 0:
        time_to_path[i] = time.time()
    """if changed:
        #print 'path_prev:'
        #for k in range(len(path_prev_i)):
            #print path_prev_i[k].x, path_prev_i[k].y
        #print 'path now:'
        #for k in range(len(optimal_path)):
            #print optimal_path[k].x, optimal_path[k].y
        display_path(path_prev_i, pywindow, Colors.black)
        display_path(optimal_path, pywindow, color_)"""
    return optimal_path, optimal_cost                  # feasible paths here is list of paths for one robot
##############################################################################################################


def perform_better_response(i, active_bots, paths_prev, paths,
                            costs, pywindow, new_vertices, goal_pts,
                            colors_, new_paths, goal, root, time_to_path):
    pi_ = [None] * (len(active_bots) - 1)
    k = 0
    for q in active_bots:                # for all active robots
        if q != i:                       # if this active bot is not the one we are collision checking with
            if paths[q]:                 # if path not empty (if bot active but hasn't found collision free path yet)
                pi_[k] = paths[q]        # we will collision check with it
            k = k + 1
    # check for collisions with these bots
    paths[i], costs[i] = better_response(pi_, goal_pts[i], new_vertices[i],
                                         paths_prev[i], pywindow, costs[i],
                                         i, colors_[i], new_paths, goal, root,
                                         time_to_path)
    return
##############################################################################################################


def check_active(new_vertices, i, active_bots, inactive_bots, new_paths):
    if new_vertices[i] is not None:
        if new_vertices[i].at_goal_set:                   # if previously inactive bot is now active
            update_active(active_bots, inactive_bots, i)  # then update the inactive and active lists
            print 'Robot', i, 'became active. Active robots:', active_bots
        elif new_paths[i] is True:
            update_active(active_bots, inactive_bots, i)
            print 'Robot', i, 'became active. Active robots:', active_bots
    return
##############################################################################################################
