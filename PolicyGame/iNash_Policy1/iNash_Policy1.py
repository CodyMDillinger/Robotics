#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Cody Dillinger
# This is the main .py file for a simulation of the i-Nash Policy Algorithm
# Details of this algorithm can be found at http://php.scripts.psu.edu/muz16/pdf/DJ-MZ-AR-IFAC15.pdf
# Root nodes are the starting points of each robot
# Left and Right child nodes also separately used for k-d tree faster nearest/near neighbor searching

import pygame, time, math
from math import *
from classes import Colors
from pywindow_funcs import init_pywindow, user_prompt, label_button, iterate_or_stop
from sampling_funcs import sample_free
from geometry_procedures import collisions, steer4, norm
from search_algorithms import nearest2, add_to_kd_tree, near_vertices2
from path_funcs import path_generation2, find_optimal_path, collision_free_path, display_path, print_paths
from list_funcs import update_active, init_arrays, update_pt_lists
from dynamics import solve_bvp_4d
##############################################################################################################


def add_edge(pt_new, pt_tree, color_, size, pywindow):  # update children and parents for new connections
    pt_new.add_parent(pt_tree)                          # update parent list of new vertex
    pt_tree.add_child(pt_new)                           # update child list of vertex that the new one is connecting to
    trajectory1 = solve_bvp_4d(pt_tree, pt_new)         # calculate dynamically feasible trajectory btwn two points
    # trajectory2 = solve_bvp_4d(pt_new, pt_tree)
    pt_tree.add_trajectory(trajectory1)
    # pt_tree.add_trajectory(trajectory2)
    # pygame.draw.line(pywindow, color_, (pt_new.x, pt_new.y), (pt_tree.x, pt_tree.y), size)
    # draw_traj(pywindow, color_, pt_new, trajectory1, size)
    # pygame.display.flip()
    return
##############################################################################################################


def extend_graph(vertex_rand, robot_root, obstacles, goal_set, pywindow, color_, k, x):  # add vertex and edges to sets for robot i
    # vertex_nearest = nearest(vertex_rand, robot_root, robot_root, k)                   # find nearest vertex in robot i's graph, starting with root
    vertex_nearest2 = nearest2(vertex_rand, robot_root, robot_root, robot_root, x)       # computationally efficient function
    # vertex_new = steer(vertex_nearest, vertex_rand, goal_set)                          # steer that vertex towards x_rand within some radius
    vertex_new2 = steer4(vertex_nearest2, vertex_rand, goal_set)
    collision_, intersect = collisions(vertex_nearest2, vertex_new2, obstacles, 'obstacles')
    if collision_ == 0:
        # if vertex_new2.at_goal_set:
            # print 'no obstacle collision for goal set vertex, added to kd tree'
        # vertices_near = near_vertices(vertex_new2, robot_root, k, [])
        vertices_near2 = near_vertices2(vertex_new2, robot_root, robot_root, k, [], x)  # computationally efficient function
        # if vertex_new2.at_goal_set:
            # print 'near verts found:', len(vertices_near2)
        edges_added = 0
        for i in range(len(vertices_near2)):                                    # for all near vertices
            collision_, intersect = collisions(vertices_near2[i], vertex_new2, obstacles, 'obstacles')
            if collision_ == 0:                                                 # if no collision
                # if vertex_new2.at_goal_set:
                # print 'no collision for near vertex, adding edge'
                add_edge(vertex_new2, vertices_near2[i], color_, 1, pywindow)   # connect the dots
                edges_added = edges_added + 1
            else:
                pass  # print 'collision with near vertex'
        if edges_added == 0:
            # print 'no edge had been added'
            del vertex_new2
            return None
        else:
            add_to_kd_tree(vertex_new2, robot_root, x)  # kd tree for spatial sorting and faster nearest/near searching
    else:
        # print 'collision, extend procedure returning None'
        del vertex_new2      # save storage if point not used
        return None
    return vertex_new2
##############################################################################################################


def better_response(pi_, goalpts, vertex, path_prev_i, pywindow, costs_i, i, color_):  # minimize cost while avoiding collisions with paths in pi_
    if vertex is not None:
        if vertex.at_goal_set:                  # only obtain new paths if new pt is at goal
            path_list_vertex, cost_list_vertex = path_generation2(vertex)  # updates vertex objects to contain path/cost list
            #print 'new vertex at goal set, calling path_gen from better_response procedure'
            #print 'new goal set vertex has parents = ', vertex.parents
            #print 'path_gen2 return costs, paths values:'
            #for j in range(len(path_list_vertex)):
                #print cost_list_vertex[j], path_list_vertex[j]
    collision_free_paths = []
    costs = []
    for j in range(len(goalpts)):
        for k in range(len(goalpts[j].paths)):
            #print 'goal path:', goalpts[j].paths[k]
            #print 'pi 0:', pi_[0]
            #print 'pi 1:', pi_[1]
            if collision_free_path(goalpts[j].paths[k], pi_, i) == 1:
                #print 'path is collision free'
                collision_free_paths.append(goalpts[j].paths[k])
                costs.append(goalpts[j].costs[k])
    if len(collision_free_paths) == 0:
        print 'paths exist but no inter-robot-collision-free paths for robot', i, 'yet'
        return [], 9999999.0
    optimal_path = list(path_prev_i)
    optimal_cost = costs_i
    optimal_path, optimal_cost, changed = find_optimal_path(collision_free_paths, costs, optimal_path, optimal_cost, i)
    if changed:
        #print 'path_prev:'
        #for k in range(len(path_prev_i)):
            #print path_prev_i[k].x, path_prev_i[k].y
        #print 'path now:'
        #for k in range(len(optimal_path)):
            #print optimal_path[k].x, optimal_path[k].y
        display_path(path_prev_i, pywindow, Colors.white)
        display_path(optimal_path, pywindow, color_)
    return optimal_path, optimal_cost                  # feasible paths here is list of paths for one robot
##############################################################################################################


def perform_better_response(q, active_bots, paths_prev, paths, costs, pywindow, new_vertices, goal_pts, colors_):
    i = active_bots[q]
    if q != 0:                            # if iteration is not first
        j = active_bots[q - 1]
        if paths[j] != []:
            path_compare1 = paths[j]      # compare to robot j < i
        else:
            path_compare1 = None          # if there is a robot j < i but hasn't found collision free path yet
    else:
        path_compare1 = None              # else no robot j < i
    if q != len(active_bots) - 1:         # if iteration is not last
        j2 = active_bots[q + 1]
        if paths[j2] != []:
            path_compare2 = paths[j2]     # compare to robot j > i
        else:
            path_compare2 = None          # if there is a robot j > i but hasn't found collision free path yet
    else:
        path_compare2 = None              # else no robot j > i
    pi_ = [path_compare1, path_compare2]  # set of two robot paths to compare to
    paths[i], costs[i] = better_response(pi_, goal_pts[i], new_vertices[i], paths_prev[i], pywindow, costs[i], i, colors_[i])  # check for collisions with these bots
    return
##############################################################################################################


def check_active(new_vertices, i, active_bots, inactive_bots):
    if new_vertices[i] is not None:
        if new_vertices[i].at_goal_set:  # if previously inactive bot is now active
            update_active(active_bots, inactive_bots, i)  # then update the inactive and active lists
            print 'robot', i, 'became active'
    return
##############################################################################################################


def main():
    pywindow, obstacles, axis, buttons = init_pywindow('i-Nash Policy 1')    # set up pygame window, dimensions and obstacles
    start, goal_set, num_robots, robo_colors, sign = user_prompt(pywindow)     # prompt for num bots, start, end positions
    all_bots, active_bots, inactive_bots, paths, costs, paths_prev, goal_pts, path_num = init_arrays(num_robots)
    k = 1; k_ = 4 * 75000                                                  # total attempted vertices for each robot / 4
    samp_bias = 0                                                          # for biased sampling. See sample_free()
    while k < k_:                                                          # main loop
        new_vertices = [None] * num_robots                                 # get list of new vertices each k iteration
        for i in all_bots:                                                 # for all robots
            vert_rand, samp_bias = sample_free(paths[i], goal_set[i], samp_bias, sign[i])  # get random Vertex in pywindow
            vert_new = extend_graph(vert_rand, start[i], obstacles, goal_set[i], pywindow, robo_colors[i], k, axis)
            goal_pts, new_vertices = update_pt_lists(vert_new, goal_pts, new_vertices, i)
            k = iterate_or_stop(pywindow, buttons, k, k_)
        for i in inactive_bots:
            check_active(new_vertices, i, active_bots, inactive_bots)
        for i in active_bots:
            paths_prev[i] = list(paths[i])                        # save previous path list before updating list
        k = iterate_or_stop(pywindow, buttons, k, k_)
        for q in range(len(active_bots)):                         # use q for easier j < i and j2 > i calculation
            perform_better_response(q, active_bots, paths_prev, paths, costs, pywindow, new_vertices, goal_pts, robo_colors)
            k = iterate_or_stop(pywindow, buttons, k, k_)
        # time.sleep(.05)
    print 'main loop exited'
    label_button(pywindow, buttons[0], 0, 'Running', Colors.dark_green, Colors.white)
    pygame.display.flip()
    # print_paths(num_robots, goal_pts, pywindow)
    return
##############################################################################################################


if __name__ == '__main__':
    main()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
