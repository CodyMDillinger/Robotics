#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Cody Dillinger
# This is the main .py file for a simulation of the i-Nash Trajectory Algorithm
# Details of this algorithm can be found at http://php.scripts.psu.edu/muz16/pdf/Zhu-Otte-ICRA14.pdf
# Graphs are "directed," where direction is parent nodes to child nodes as labeled in the Vertex class
# Root nodes are the starting points of each robot
# Left and Right child nodes also separately used for k-d tree faster nearest/near neighbor searching

import pygame, time
from math import *
from classes import Colors
from pywindow_funcs import init_pywindow, user_prompt, world_to_y_plot, world_to_x_plot
from sampling_funcs import sample_free
from geometry_procedures import collisions, steer4, norm
from search_algorithms import nearest2, add_to_kd_tree, near_vertices2
from path_funcs import path_generation2, find_optimal_path, collision_free_path, display_path
from list_funcs import update_active, init_arrays, update_pt_lists
##############################################################################################################


def add_edge(pt_new, pt_tree, color_, size, pywindow):   # update children and parents for new connections
    pt_new.add_parent(pt_tree)                           # update parent list of new vertex
    pt_tree.add_child(pt_new)                            # update child list of vertex that the new one is connecting to
    pygame.draw.line(pywindow, color_, (pt_new.x, pt_new.y), (pt_tree.x, pt_tree.y), size)
    # pygame.draw.line(pywindow, color_, (world_to_x_plot(pt_new.x, pt_new.x_vel)), (world_to_x_plot(pt_tree.x, pt_tree.x_vel)), size)
    # pygame.draw.line(pywindow, color_, (world_to_y_plot(pt_new.y, pt_new.y_vel)), (world_to_y_plot(pt_tree.y, pt_tree.y_vel)), size)
    pygame.display.flip()
    return
##############################################################################################################


def extend_graph(vertex_rand, robot_root, obstacles, goal_set, pywindow, color_, k, x):  # add vertex and edges to sets for robot i
    # vertex_nearest = nearest(vertex_rand, robot_root, robot_root, k)                   # find nearest vertex in robot i's graph, starting with root
    vertex_nearest2 = nearest2(vertex_rand, robot_root, robot_root, robot_root, x)       # computationally efficient function
    # vertex_new = steer(vertex_nearest, vertex_rand, goal_set)                          # steer that vertex towards x_rand within some radius
    vertex_new2 = steer4(vertex_nearest2, vertex_rand, goal_set)
    collision_, intersect = collisions(vertex_nearest2, vertex_new2, obstacles, 'obstacles')
    if collision_ == 0:
        add_to_kd_tree(vertex_new2, robot_root, x)            # kd tree for spatial sorting and faster nearest/near searching
        # if vertex_new2.at_goal_set:
            # print 'no obstacle collision for goal set vertex, added to kd tree'
        # vertices_near = near_vertices(vertex_new2, robot_root, k, [])
        vertices_near2 = near_vertices2(vertex_new2, robot_root, robot_root, k, [], x)  # computationally efficient function
        # if vertex_new2.at_goal_set:
            # print 'near verts found:', len(vertices_near2)
        for i in range(len(vertices_near2)):                                    # for all near vertices
            collision_, intersect = collisions(vertices_near2[i], vertex_new2, obstacles, 'obstacles')
            if collision_ == 0:                                                 # if no collision
                # if vertex_new2.at_goal_set:
                # print 'no collision for near vertex, adding edge'
                add_edge(vertex_new2, vertices_near2[i], color_, 1, pywindow)   # connect the dots
            else:
                pass  # print 'collision with near vertex'
    else:
        # print 'collision, extend procedure returning None'
        del vertex_new2      # save storage if point not used
        return None
    return vertex_new2
##############################################################################################################


def better_response(pi_, goalpts, vertex, path_prev_i, pywindow, costs_i, i, color_):  # minimize cost while avoiding collisions with paths in pi_
    if vertex is not None:
        if vertex.at_goal_set:                  # only obtain new paths if new pt is at goal
            #print 'new vertex at goal set, calling path_gen from better_response procedure'
            path_list_vertex, cost_list_vertex = path_generation2(vertex)
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
        print 'paths exist but no collision free paths for robot', i
        return [], 9999999
    optimal_path = list(path_prev_i)
    optimal_cost = costs_i
    optimal_path, optimal_cost, changed = find_optimal_path(collision_free_paths, costs, optimal_path, optimal_cost, i)
    if changed:
        display_path(path_prev_i, pywindow, Colors.black)
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


def main():
    pywindow, obstacles, axis = init_pywindow('i-Nash Trajectory Final 4')    # set up pygame window, dimensions and obstacles
    start, goal_set, num_robots, robo_colors = user_prompt(pywindow)          # prompt for num bots, start, end positions
    all_bots, active_bots, inactive_bots, paths, costs, paths_prev, goal_pts, path_num = init_arrays(num_robots)
    k = 1; k_ = 75000
    while k < k_:                                                             # main loop
        new_vertices = [None] * num_robots                                    # get list of new vertices each k iteration
        for i in all_bots:                                                    # for all robots
            vertex_rand = sample_free(paths[i], goal_set[i])                  # get random Vertex in pywindow
            vertex_new = extend_graph(vertex_rand, start[i], obstacles, goal_set[i], pywindow, robo_colors[i], k, axis)
            goal_pts, new_vertices = update_pt_lists(vertex_new, goal_pts, new_vertices, i)
        for i in inactive_bots:
            if new_vertices[i] is not None:
                if new_vertices[i].at_goal_set:                   # if previously inactive bot is now active
                    update_active(active_bots, inactive_bots, i)  # then update the inactive and active lists
                    print 'robot', i, 'became active'
        for i in active_bots:
            paths_prev[i] = list(paths[i])                        # save previous path list before updating list
        for q in range(len(active_bots)):                         # use q for easier j < i and j2 > i calculation
            perform_better_response(q, active_bots, paths_prev, paths, costs, pywindow, new_vertices, goal_pts, robo_colors)
        #time.sleep(.05)
        if vertex_new is not None:
            k = k + 1
    print 'main loop exited'
    repeat = False
    for i in range(num_robots):                                   # for all bots
        for j in range(len(goal_pts[i])):                         # for all goal pts for that bot
            for P in range(len(goal_pts[i][j].paths)):            # for all points for that goal pt
                string = []
                string.append(goal_pts[i][j].costs[P])
                for F in range(len(goal_pts[i][j].paths[P])):
                    string.append(floor(goal_pts[i][j].paths[P][F].x))
                    string.append(floor(goal_pts[i][j].paths[P][F].y))
                    #print 'vertex.paths for all vertices in path', P, ':', goal_pts[i][j].paths[P][F].paths
                    for F2 in range(len(goal_pts[i][j].paths[P])):
                        if goal_pts[i][j].paths[P][F] == goal_pts[i][j].paths[P][F2]:
                            repeat = True
                #print 'robot', i, ', goalpt', j, ',path', P, 'cost', string
                display_path(goal_pts[i][j].paths[P], pywindow, Colors.dark_green)  # display
                #print 'repeat is ', repeat
    return
##############################################################################################################


if __name__ == '__main__':
    main()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
