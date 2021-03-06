#!/usr/bin/env python
# -*- coding: utf-8 -*-
# This contains the high level functions called from iNash_policy. The main functions.

from list_funcs import update_active
from dynamics import solve_bvp_4d
from geometry_procedures import collisions, steer4
from search_algorithms import nearest2, add_to_kd_tree, near_vertices2
from path_funcs import path_generation2, find_optimal_path, collision_free_path
from classes import Settings
##############################################################################################################


def add_edge(pt_new, pt_tree, color_, size, pywindow):  # update children and parents for new connections
    pt_new.add_parent(pt_tree)                          # update parent list of new vertex
    pt_tree.add_child(pt_new)                           # update child list of vertex that the new one is connecting to
    trajectory1 = solve_bvp_4d(pt_tree, pt_new) # calculate dynamically feasible trajectory btwn two points
    # traj1_states uses global number of states between points
    # traj1_states2 uses global step_time for states between points
    # trajectory2 = solve_bvp_4d(pt_new, pt_tree)
    pt_tree.add_trajectory(trajectory1)
    # pt_tree.add_trajectory(trajectory2)
    #pygame.draw.line(pywindow, color_, (pt_new.x, pt_new.y), (pt_tree.x, pt_tree.y), size)
    # draw_traj(pywindow, color_, pt_new, trajectory1, size)
    #pygame.display.flip()
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
        return [], Settings.collision_cost
    optimal_path = list(path_prev_i)
    optimal_cost = costs_i
    optimal_path, optimal_cost, changed = find_optimal_path(collision_free_paths, costs, optimal_path, optimal_cost, i)
    #if changed:
        #print 'path_prev:'
        #for k in range(len(path_prev_i)):
            #print path_prev_i[k].x, path_prev_i[k].y
        #print 'path now:'
        #for k in range(len(optimal_path)):
            #print optimal_path[k].x, optimal_path[k].y
        #display_path(path_prev_i, pywindow, Colors.black)
        #display_path(optimal_path, pywindow, color_)
    return optimal_path, optimal_cost                  # feasible paths here is list of paths for one robot
##############################################################################################################


def perform_better_response(i, active_bots, paths_prev, paths, costs, pywindow, new_vertices, goal_pts, colors_):
    pi_ = [None] * (len(active_bots) - 1)
    k = 0
    for q in active_bots:                # for all active robots
        if q != i:                       # if this active bot is not the one we are collision checking with
            if paths[q]:                 # if path not empty (if bot active but hasn't found collision free path yet)
                pi_[k] = paths[q]        # we will collision check with it
            k = k + 1
    paths[i], costs[i] = better_response(pi_, goal_pts[i], new_vertices[i], paths_prev[i], pywindow, costs[i], i, colors_[i])  # check for collisions with these bots
    return
##############################################################################################################


def check_active(new_vertices, i, active_bots, inactive_bots):
    if new_vertices[i] is not None:
        if new_vertices[i].at_goal_set:                   # if previously inactive bot is now active
            update_active(active_bots, inactive_bots, i)  # then update the inactive and active lists
            print 'Robot', i, 'became active. Active robots:', active_bots
    return
##############################################################################################################
