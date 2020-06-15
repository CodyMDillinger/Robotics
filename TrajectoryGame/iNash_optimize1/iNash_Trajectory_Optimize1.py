#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Cody Dillinger
# This is a simulation of the i-Nash Trajectory Algorithm from http://php.scripts.psu.edu/muz16/pdf/Zhu-Otte-ICRA14.pdf
# Graphs are "directed," where direction is parent nodes to child nodes as labeled in the "Vertex" class
# Root nodes are the starting points of each robot

import os, sys, random, math, pygame, time
from pygame.locals import *
from math import*
from classes import Vertex, Colors, Color_, Dimensions
##############################################################################################################


def create_shape(pts):                           # given a set of Vertices, connect them in order
    for pt_num in range(len(pts) - 1):           # for each Vertex except the last
        pts[pt_num].obs_next = pts[pt_num + 1]   # connect the dots
    pts[len(pts) - 1].obs_next = pts[0]          # connect last Vertex back to first
    return pts
##############################################################################################################


def draw_shape(shape, pywindow):                                          # outline border of shape on pywindow
    for pt_num in range(len(shape)):                                      # for all vertices in the shape
        x1, y1 = shape[pt_num].x, shape[pt_num].y                         # point from
        x2, y2 = shape[pt_num].obs_next.x, shape[pt_num].obs_next.y       # point to
        if x1 == x2:                        # if vertical edge
            x1 = x1 - .5; x2 = x2 - .5      # show display slightly offset left
        elif y1 == y2:                      # if horizontal edge
            y1 = y1 - .5; y2 = y2 - .5      # show display slightly offset up
        pygame.draw.line(pywindow, Colors.black, (x1, y1), (x2, y2), 3)   # connect edge between them
##############################################################################################################


def obstacle_generation():    # return array of obstacles, obstacle is array of Vertex objects connected in a loop
    window = create_shape(Dimensions.window)
    obs1 = create_shape(Dimensions.obstacle1)
    obs2 = create_shape(Dimensions.obstacle2)
    obstacles = [window, obs1, obs2]
    return obstacles
##############################################################################################################


def add_colors(robo_color):                      # append colors to array to use for large numbers of robot trees
    robo_color.add_color(Color_(Colors.grey))
    robo_color.add_color(Color_(Colors.dark_red))
    robo_color.add_color(Color_(Colors.turquoise))
    robo_color.add_color(Color_(Colors.purple))
    robo_color.add_color(Color_(Colors.light_orange))
    robo_color.add_color(Color_(Colors.pink))
    robo_color.add_color(Color_(Colors.light_red))
    robo_color.add_color(Color_(Colors.light_blue))
    for i in range(len(robo_color.color_list) - 1):
        robo_color.color_list[i].next_ = robo_color.color_list[i + 1]                       # connect them in order
    robo_color.color_list[len(robo_color.color_list) - 1].next_ = robo_color.color_list[0]  # connect the last to the first
    return robo_color
##############################################################################################################


def color_init(num_robots_):                     # get array of colors same size as num of robots
    robo_colors = Color_((0, 0, 0))              # single color object, with array to be used for list of colors
    robo_colors = add_colors(robo_colors)        # add 9 colors connected in a loop
    colors = [None] * num_robots_
    color_ = robo_colors.color_list[0]
    for i in range(num_robots_):                 # for all bots
        colors[i] = color_.color_                # set color
        color_ = color_.next_
    return colors
##############################################################################################################


def init_pywindow(title):                                       # first function to be called from main()
    pygame.init()                                               # initialize usage of pygame
    pygame.display.set_caption(title)
    pywindow = pygame.display.set_mode((Dimensions.window_length, Dimensions.window_width))         # create pygame display
    pywindow.fill(Colors.white)                                 # set background of pygame window to white
    obstacles = obstacle_generation()
    for i in range(len(obstacles)):                             # for all obstacles
        draw_shape(obstacles[i], pywindow)                      # outline borders on pywindow
    pygame.display.flip()                                       # update display with these new shapes
    return pywindow, obstacles
##############################################################################################################


def get_click_pos(i, pt_type, pt, pywindow, color_dot):     # prompt user to click pygame window, read position
    print 'Click robot', i+1, pt_type, 'Vertex'             # prompt user
    waiting = 1; exit_pressed = 0
    while waiting:                                          # loop until user has clicked, or clicked exit
        event_click = pygame.event.poll()                   # read whether user event has occurred
        if event_click.type == pygame.QUIT:                 # if user clicked exit
            waiting = 0
            exit_pressed = 1
        elif event_click.type == pygame.MOUSEBUTTONDOWN and event_click.button == 1:  # if user clicked in pywindow
            x, y = pygame.mouse.get_pos()                                 # get position of the click
            pt.append(Vertex(x, y))                                       # array of all start/end positions
            pygame.draw.circle(pywindow, color_dot, (x, y), 10, 0)        # display starting Vertex with color circle
            pygame.display.flip()                                         # update display with these new shapes
            waiting = 0
    return exit_pressed, pt
##############################################################################################################


def user_prompt(pywindow):         # prompt for num robots, start and end positions, mark on pywindow
    num_robots = input('Enter number of robots: ')
    robo_colors = color_init(num_robots)
    start = []                     # to be used for coordinates of robots start positions
    goal_set = []                  # to be used for coordinates of robots end positions
    i = 0
    running_ = 1
    while i < num_robots and running_:                                 # until start/stop selected for all, or exited
        exit1, start = get_click_pos(i, 'start', start, pywindow, robo_colors[i])      # get start position from user, append
        exit2, goal_set = get_click_pos(i, 'end', goal_set, pywindow, robo_colors[i])  # get end position from user, append
        i = i + 1
        if exit1 == 1 or exit2 == 1:                                   # end loop if user exited
            running_ = 0
    goal_set_size = 12
    goal_set2 = []                # updating goal set to be a larger box, rather than just a goal Vertex
    for i in range(num_robots):   # make four Vertices with goal_set at center
        pt1 = Vertex(goal_set[i].x - goal_set_size, goal_set[i].y - goal_set_size)
        pt2 = Vertex(goal_set[i].x - goal_set_size, goal_set[i].y + goal_set_size)
        pt3 = Vertex(goal_set[i].x + goal_set_size, goal_set[i].y + goal_set_size)
        pt4 = Vertex(goal_set[i].x + goal_set_size, goal_set[i].y - goal_set_size)
        goal_box = create_shape([pt1, pt2, pt3, pt4])
        goal_set2.append(goal_box)
        draw_shape(goal_box, pywindow)
    pygame.display.flip()
    return start, goal_set2, num_robots, robo_colors
##############################################################################################################


def sample_free():		         # return pseudo random Vertex object with coordinates uniformly in pywindow
    return Vertex(random.random() * Dimensions.window_length, random.random() * Dimensions.window_width)
##############################################################################################################


def dist(pt1, pt2):              # calculate distance between two points
    return sqrt((pt1.x - pt2.x)**2 + (pt1.y - pt2.y)**2)
##############################################################################################################


def nearest(x_rand, current_vertex, x_nearest, k):              # recursively search tree for nearest node
    current_vertex.k_nearest = k
    if dist(x_rand, current_vertex) < dist(x_rand, x_nearest) and current_vertex.at_goal_set is False:  # if distance to next vertex in tree is smaller
        x_nearest = current_vertex                              # update it as the nearest
    for i in range(len(current_vertex.children)):               # for all children vertices
        if current_vertex.children[i].k_nearest != k:           # if next vertex not checked yet
            x_nearest = nearest(x_rand, current_vertex.children[i], x_nearest, k)   # call nearest function again
    return x_nearest
##############################################################################################################


def cross_prod(pt1, pt2, pt3):    # get cross product of pt with line connecting two other pts
    if (pt3.y - pt1.y) * (pt2.x - pt1.x) > (pt2.y - pt1.y) * (pt3.x - pt1.x):
        prod = 1
    else: prod = 0
    return prod
##############################################################################################################


def collision_line(pt1, pt2, pt1_, pt2_):   # check if collision between two lines
    if cross_prod(pt1, pt1_, pt2_) != cross_prod(pt2, pt1_, pt2_) and cross_prod(pt1, pt2, pt1_) != cross_prod(pt1, pt2,pt2_):
        collision_ = 1
    else:
        collision_ = 0
    return collision_
##############################################################################################################


def line(pt1, pt2):                         # return slope and intercept of line, given two points
    if pt2.x == pt1.x:                      # avoid divide by zero
        m = None
        b = None
    else:
        m = (pt2.y - pt1.y) / (pt2.x - pt1.x)   # slope
        b = pt1.y - (m * pt1.x)                 # intercept
    return m, b
##############################################################################################################


def intersection(pt1, pt2, pt3, pt4):     # find intersection point of two lines, given two points for each line
    m, b = line(pt1, pt2)                 # get line values from 2 pts
    m2, b2 = line(pt3, pt4)
    if m is not None and m2 is not None:  # for non vertical lines
        x = (b2 - b) / (m - m2)           # calculate x value
        y = (m * x) + b                   # calculate y value
    else:
        if m2 is None:                    # if line 2 was the vertical one
            x = pt3.x
            y = (m * x) + b
        else:                             # else line 1 was the vertical one
            x = pt1.x
            y = (m2 * x) + b2
    return Vertex(x, y)
##############################################################################################################


def collision(pt1, pt2, shape, type_):         # check if pt1 to pt2 collides with any edge of shape
    collision_ = 0; intersect = None
    for i in range(len(shape)):                                         # for all edges of shape
        if collision_line(pt1, pt2, shape[i], shape[i].obs_next) == 1:  # if collision
            collision_ = 1
            if type_ == 'goal':                # if checking collision for goal set, calculate intersection pt
                intersect = intersection(pt1, pt2, shape[i], shape[i].obs_next)
                intersect.at_goal_set = True
            break

    return collision_, intersect
##############################################################################################################


def collisions(pt1, pt2, list_, type_):      # calling collision function for a list of obstacles
    collision_ = 0; intersect = None
    for i in range(len(list_)):
        collision_, intersect = collision(pt1, pt2, list_[i], type_)
        if collision_ == 1:
            break                            # end loop early if collision found
    return collision_, intersect
##############################################################################################################


def trace_inclusivity(x_nearest, x_new, goal_set):  # check that trajectory does not cross through goal set and back out
    collision_, intersect = collision(x_nearest, x_new, goal_set, 'goal')
    if collision_ == 1:                             # if trajectory hits goal set
        x_new = intersect                           # adjust new vertex to the intersection point
    return x_new
##############################################################################################################


def steer(x_nearest, x_rand, goal_set):                               # steer nearest vertex towards random Vertex, within some radius
    theta = atan2(x_rand.y - x_nearest.y, x_rand.x - x_nearest.x)     # atan2 accounts for different quadrants
    x_new = x_nearest.x + Dimensions.tree_radius * cos(theta)         # new x value
    y_new = x_nearest.y + Dimensions.tree_radius * sin(theta)         # new y value
    vertex_new = Vertex(x_new, y_new)                                 # initialize vertex
    vertex_new_ = trace_inclusivity(x_nearest, vertex_new, goal_set)  # if hits goal, adjust vertex to intersection pt
    return vertex_new_
##############################################################################################################


def add_edge(pt_new, pt_tree, color_, size, pywindow):   # update children and parents for new connections
    pt_new.add_parent(pt_tree)                           # update parent list of new vertex
    pt_tree.add_child(pt_new)                            # update child list of vertex that the new one is connecting to
    #pygame.draw.line(pywindow, color_, (pt_new.x, pt_new.y), (pt_tree.x, pt_tree.y), size)
    #pygame.display.flip()
    return
##############################################################################################################


def near_vertices(vertex_new, current_vertex, k, vertices_near):  # return all vertices within near_radius
    current_vertex.k_near = k                                     # for avoiding checking one vertex multiple times
    near_radius = min(Dimensions.gamma * sqrt(log(k) / k), Dimensions.eta)       # function of numPoints
    if dist(vertex_new, current_vertex) < near_radius and (current_vertex.at_goal_set is False):  # if within radius
        vertices_near.append(current_vertex)                                     # add to list of near vertices
    for i in range(len(current_vertex.children)):                                # for all children of this vertex
        if current_vertex.children[i].k_near != k:                               # if that child not checked yet
            vertices_near = near_vertices(vertex_new, current_vertex.children[i], k, vertices_near)  # call function again
    return vertices_near
##############################################################################################################


def extend_graph(vertex_rand, robot_root, obstacles, goal_set, pywindow, color_, k):  # add vertex and edges to sets for robot i
    vertex_nearest = nearest(vertex_rand, robot_root, robot_root, k)                  # find nearest vertex in robot i's graph, starting with root
    vertex_new = steer(vertex_nearest, vertex_rand, goal_set)                         # steer that vertex towards x_rand within some radius
    collision_, intersect = collisions(vertex_nearest, vertex_new, obstacles, 'obstacles')
    if collision_ == 0:
        vertices_near = near_vertices(vertex_new, robot_root, k, [])
        for i in range(len(vertices_near)):                                    # for all near vertices
            collision_, intersect = collisions(vertices_near[i], vertex_new, obstacles, 'obstacles')
            if collision_ == 0:                                                # if no collision
                add_edge(vertex_new, vertices_near[i], color_, 1, pywindow)    # connect the dots
    return vertex_new
##############################################################################################################


def path_generation2(vertex):  # tree traversal. get all paths root to vertex. update vertex.path_list[[]] and vertex.costs[]
    paths_parents = []                             # list of paths, path = list of vertices
    costs_parents = []                             # list of costs associated with some paths
    paths = []
    costs = []
    for i in range(len(vertex.parents)):           # for all parents
        parent = vertex.parents[i]
        pths, csts = path_generation2(parent)       # get all paths and costs for that parent
        paths_parents.append(pths)                 # append paths to path list
        costs_parents.append(csts)                 # append costs to cost list
    # separating a list of lists of paths into one list of paths:
    for i in range(len(paths_parents)):                        # for number of lists of paths (number of parents)
        distance = dist(vertex, vertex.parents[i])             # calculate distance for each new parent
        for j in range(len(paths_parents[i])):                 # for number of paths in one of those lists
            paths.append(paths_parents[i][j])                  # append that path to new total list
            costs.append(costs_parents[i][j] + distance)       # append that parent cost + distance to new total cost list
    for i in range(len(paths)):                 # for every path in paths
        paths[i].append(vertex)                 # add the current vertex to that path
        vertex.add_path(paths[i])               # add these paths to vertex object to avoid recalculating later
        vertex.add_cost(costs[i])               # add these costs to vertex object '' '' ''
    if len(paths) == 0:                         # if this vertex is the root node
        paths = [[vertex]]                      # only "path" is itself, and has zero cost
        costs = [0]
        vertex.add_cost(0)
        vertex.add_path([vertex])
    return paths, costs
##############################################################################################################


#def path_generation(vertex):  # tree traversal. get all paths root to vertex. update vertex.path_list[[]] and vertex.costs[]
#    if vertex.paths == []:                             # if haven't calculated paths to this point yet
#        paths_parents = []                             # list of paths, path = list of vertices
#        costs_parents = []                             # list of costs associated with some paths
#        paths = []
#        costs = []
#        for i in range(len(vertex.parents)):           # for all parents
#            parent = vertex.parents[i]
#            pths, csts = path_generation(parent)       # get all paths and costs for that parent
#            paths_parents.append(pths)                 # append paths to path list
#            costs_parents.append(csts)                 # append costs to cost list
#        # separating a list of lists of paths into one list of paths:
#        for i in range(len(paths_parents)):                        # for number of lists of paths (number of parents)
#            distance = dist(vertex, vertex.parents[i])             # calculate distance for each new parent
#            for j in range(len(paths_parents[i])):                 # for number of paths in one of those lists
#                paths.append(paths_parents[i][j])                  # append that path to new total list
#                costs.append(costs_parents[i][j] + distance)       # append that parent cost + distance to new total cost list
#        for i in range(len(paths)):                 # for every path in paths
#            paths[i].append(vertex)                 # add the current vertex to that path
#            vertex.add_path(paths[i])               # add these paths to vertex object to avoid recalculating later
#            vertex.add_cost(costs[i])               # add these costs to vertex object '' '' ''
#        if len(paths) == 0:                         # if this vertex is the root node
#            paths = [[vertex]]                      # only "path" is itself, and has zero cost
#            costs = [0]
#            vertex.add_cost(0)
#            vertex.add_path([vertex])
#    else:                                           # if already traversed from this vertex
#        paths = vertex.paths                        # use those saved paths
#        costs = vertex.costs                        # use those saved costs
#    return paths, costs
##############################################################################################################


def set_to(path):                # set two paths equal without making them the same instance of object
    path_ = []
    for i in range(len(path)):   # for whole path
        path_.append(path[i])    # copy element (vertex)
    return path_
##############################################################################################################


def find_optimal_path(goalpts, opt_path, opt_cost, i):
    changed = False
    if len(opt_path) == 0:                              # if this is first time getting a path for robot i
        opt_path = goalpts[0].paths[0]                  # just select first path option. optimizations happen later
        opt_cost = goalpts[0].costs[0]
        changed = True
        print 'first path cost:', opt_cost
        clr = Colors.turquoise
    else:
        clr = Colors.dark_orange
        for j in range(len(goalpts)):                   # for all vertices at the goal set
            for q in range(len(goalpts[j].costs)):      # for all paths to root from that vertex
                if len(goalpts[j].costs) != len(goalpts[j].paths):
                    print 'path and cost lists not aligned?'
                if goalpts[j].costs[q] < opt_cost:      # if the path is more optimal
                    opt_path = goalpts[j].paths[q]      # then update path and cost
                    opt_cost = goalpts[j].costs[q]
                    changed = True
                    print 'more optimal path found for robot', i, ', cost:', opt_cost
                    print 'goal point:', j, floor(goalpts[j].x), floor(goalpts[j].y), 'path_num:', q
                    string = []
                    for F in range(len(opt_path)):
                        string.append(floor(opt_path[F].x)); string.append(floor(opt_path[F].y))
                    #print string
                    return opt_path, opt_cost, changed, clr           # break the loop for computational efficiency
    return opt_path, opt_cost, changed, clr
##############################################################################################################


def better_response(pi_, goalpts, vertex, path_prev_i, pywindow, costs_i, i):  # minimize cost while avoiding collisions with paths in pi_
    if vertex.at_goal_set:                                         # only obtain new paths if new pt is at goal
        path_list_vertex, cost_list_vertex = path_generation2(vertex)
        string = []
        print 'new goal set vertex.'
        for i in range(len(path_list_vertex)):
            for j in range(len(path_list_vertex[i])):
                string.append(floor(path_list_vertex[i][j].x))
                string.append(floor(path_list_vertex[i][j].y))
            #print string
    # enter collision free path procedure here for inter-robot collisions

    optimal_path = set_to(path_prev_i)
    optimal_cost = costs_i
    optimal_path, optimal_cost, changed, clr = find_optimal_path(goalpts, optimal_path, optimal_cost, i)
    if changed:
        #display_path(path_prev_i, pywindow, Colors.white)
        display_path(optimal_path, pywindow, clr)
    return optimal_path, optimal_cost                      # feasible paths here is list of paths for one robot
##############################################################################################################


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


def display_path(pts, pywindow, color_):
    for i in range(len(pts) - 1):
        pygame.draw.line(pywindow, color_, (pts[i].x, pts[i].y), (pts[i + 1].x, pts[i + 1].y), 6)
    pygame.display.flip()
    return
##############################################################################################################


def main():
    path_exists = False
    pywindow, obstacles = init_pywindow('iNash trajectory, optimization 1, no inter-robot checking')         # set up pygame window, dimensions and obstacles
    start, goal_set, num_robots, robo_colors = user_prompt(pywindow)          # prompt for num bots, start, end positions
    all_bots, active_bots, inactive_bots, paths, costs, paths_prev, goal_pts, path_num = init_arrays(num_robots)
    k = 1; k_ = 4000
    while k < k_:                                                             # main loop
        new_vertices = [None] * num_robots                                    # get list of new vertices each k iteration
        for i in all_bots:                                                    # for all robots
            vertex_rand = sample_free()                                       # get random Vertex uniformly in pywindow
            vertex_new = extend_graph(vertex_rand, start[i], obstacles, goal_set[i], pywindow, robo_colors[i], k)
            if vertex_new.at_goal_set:
                path_exists = True
                goal_pts[i].append(vertex_new)                # use for easier path_generation() later
            else:
                path_exists = False
            new_vertices[i] = vertex_new                      # use for easier checking of whether bot is active
        for i in inactive_bots:
            if new_vertices[i].at_goal_set:                   # if previously inactive bot is now active
                update_active(active_bots, inactive_bots, i)  # then update the inactive and active lists
                print 'robot', i + 1, 'became active'
        for i in active_bots:
            paths_prev[i] = set_to(paths[i])                 # save previous path list before updating list
        for q in range(len(active_bots)):                    # use q for easier j and j2 calculation
            i = active_bots[q]
            if q != 0:                                       # if iteration is not first
                j = active_bots[q - 1]
                path_compare1 = paths[j]                     # compare to robot j < i
            else:
                path_compare1 = None                         # else no robot j < i
            if q != len(active_bots) - 1:                    # if iteration is not last
                j2 = active_bots[q + 1]
                path_compare2 = paths_prev[j2]               # compare to robot j > i
            else:
                path_compare2 = None                         # else no robot j > i
            pi_ = [path_compare1, path_compare2]             # set of two robot paths to compare to
            paths[i], costs[i] = better_response(pi_, goal_pts[i], new_vertices[i], paths_prev[i], pywindow, costs[i], i)  # check for collisions with these bots
            #string = []
            #if path_exists:
                #for F in range(len(paths[i])):
                    #string.append(floor(paths[i][F].x))
                    #string.append(floor(paths[i][F].y))
                #print string
        k = k + 1
    print 'main loop exited'
    for i in range(num_robots):                             # for all bots
        for j in range(len(goal_pts[i])):                   # for all goal pts for that bot
            for P in range(len(goal_pts[i][j].paths)):      # for all points for that goal pt
                if P > 2:
                    break
                string = []
                for F in range(len(goal_pts[i][j].paths[P])):
                    string.append(floor(goal_pts[i][j].paths[P][F].x))
                    string.append(floor(goal_pts[i][j].paths[P][F].y))
                print 'robot', i, ', goalpt', j, ',path', P, ':', string
                #display_path(goal_pts[i][j].paths[P], pywindow, Colors.dark_green)  # display
    return
##############################################################################################################


if __name__ == '__main__':
    main()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
