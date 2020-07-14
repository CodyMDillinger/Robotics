#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Cody Dillinger
# This is a simulation of the i-Nash Trajectory Algorithm from http://php.scripts.psu.edu/muz16/pdf/Zhu-Otte-ICRA14.pdf
# Graphs are "directed," where direction is parent nodes to child nodes as labeled in the "Vertex" class
# Root nodes are the starting points of each robot
# Left and Right child nodes also separately used for k-d tree faster nearest/near neighbor searching

import os, sys, random, math, pygame, time
from pygame.locals import *
from math import*
from classes import Vertex, Colors, Color_, Dimensions, Settings, Axes
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
    obstacles = []
    for i in range(len(Dimensions.obs_list)):
        obstacles.append(create_shape(Dimensions.obs_list[i]))
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
    x = Axes('x')                                               # for scalable number of dimensions for kd-tree
    y = Axes('y')
    x.next_ = y
    y.next_ = x
    return pywindow, obstacles, x                               # kd tree will start first row comparing x
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
    goal_set2 = []                # updating goal set to be a larger box, rather than just a goal Vertex
    for i in range(num_robots):   # make four Vertices with goal_set at center
        pt1 = Vertex(goal_set[i].x - Dimensions.goal_set_size, goal_set[i].y - Dimensions.goal_set_size)
        pt2 = Vertex(goal_set[i].x - Dimensions.goal_set_size, goal_set[i].y + Dimensions.goal_set_size)
        pt3 = Vertex(goal_set[i].x + Dimensions.goal_set_size, goal_set[i].y + Dimensions.goal_set_size)
        pt4 = Vertex(goal_set[i].x + Dimensions.goal_set_size, goal_set[i].y - Dimensions.goal_set_size)
        goal_box = create_shape([pt1, pt2, pt3, pt4])
        goal_set2.append(goal_box)
        draw_shape(goal_box, pywindow)
    pygame.display.flip()
    return start, goal_set2, num_robots, robo_colors
##############################################################################################################


def get_bias_box(goal_x, goal_y):  # get a box within pygame window centered around goal set, for biasing random sample
    w = .5 * sqrt(2) * Dimensions.window_width      # width of box
    h = .5 * sqrt(2) * Dimensions.window_length     # height of box
    if goal_x + w / 2 > Dimensions.window_width:    # if centering around goalset would push box out of pywindow ?
        x_r = Dimensions.window_width
        x_l = Dimensions.window_width - w
    elif goal_x - w / 2 < 0:
        x_l = 0
        x_r = w
    else:
        x_r = goal_x + w / 2
        x_l = goal_x - w / 2
    if goal_y + h / 2 > Dimensions.window_length:
        y_large = Dimensions.window_length
        y_small = Dimensions.window_length - h
    elif goal_y - h / 2 < 0:
        y_small = 0
        y_large = h
    else:
        y_small = goal_y - h / 2
        y_large = goal_y + h / 2
    return x_l, x_r, y_small, y_large             # return x and y vals of the 4 edges of box
##############################################################################################################


def sample_free(path, goal_set):      # return pseudo random Vertex object
    # if path == []:                   # with coordinates uniformly in pywindow
    return Vertex(random.random() * Dimensions.window_length, random.random() * Dimensions.window_width)
    # else:                            # with coordinates biased towards goal
        # x_l, x_r, y_sm, y_lar = get_bias_box(goal_set[0].x + Dimensions.goal_set_size, goal_set[0].y + Dimensions.goal_set_size)
        # vert = Vertex((random.random() * (x_r - x_l)) + x_l, (random.random() * (y_lar - y_sm)) + y_sm)
        # print vert.x, vert.y
        # return vert
##############################################################################################################


def dist(pt1, pt2):                  # calculate distance between two vertices
    return sqrt((pt1.x - pt2.x)**2 + (pt1.y - pt2.y)**2)
##############################################################################################################


# this nearest is not being used. using nearest2 **********************
def nearest(vertex_rand, current_vertex, vertex_nearest, k):    # recursively exhaustively search tree for nearest node
    current_vertex.k_nearest = k
    if dist(vertex_rand, current_vertex) < dist(vertex_rand, vertex_nearest) and current_vertex.at_goal_set is False:  # if distance to next vertex in tree is smaller
        vertex_nearest = current_vertex                         # update it as the nearest  -  is this accidentally changing root node value?
    for i in range(len(current_vertex.children)):               # for all children vertices
        if current_vertex.children[i].k_nearest != k:           # if next vertex not checked yet
            vertex_nearest = nearest(vertex_rand, current_vertex.children[i], vertex_nearest, k)   # call nearest function again
    return vertex_nearest
##############################################################################################################


# traverses portion of tree, excluding regions that are guaranteed to have no closer points
def nearest2(vertex_rand, current_vertex, prev_vertex, nearest_, axis_):           # uses kd tree to efficiently return approximate nearest vertex
    #print 'optimized nearest, axis:', axis_.axis
    if dist(current_vertex, vertex_rand) < dist(nearest_, vertex_rand) and current_vertex.at_goal_set is False:
        nearst = current_vertex
    else:
        nearst = nearest_

    """
    this would be an example of an exhaustive search, which we are not doing:
    if current_vertex.left_child is not None:
        nearst = nearest2(vertex_rand, current_vertex.left_child, current_vertex, nearst, axis_.next_)
    if current_vertex.right_child is not None:
        nearst = nearest2(vertex_rand, current_vertex.right_child, current_vertex, nearst, axis_.next_)
    return nearst
    """

    # traverse in direction of current vertex towards random vertex
    # get temp vertex, which is on corner of bounding box exclusion
    # if dist(temp, rand) is not greater than dist(nearest, rand)
    # then traverse in direction of current vertex away from random vertex
    # this method allows exclusions of certain parts of the tree (pruning)
    # which is great for large numbers of vertices

    if getattr(vertex_rand, axis_.axis) > getattr(current_vertex, axis_.axis):  # if random vertex is to the right of or above the current vertex
        next_vert1 = current_vertex.right_child     # then traverse towards the random vertex (right)
        next_vert2 = current_vertex.left_child
        #print 'rand is to right of current', getattr(vertex_rand, axis_.axis), '>', getattr(current_vertex, axis_.axis)
    else:
        next_vert1 = current_vertex.left_child      # else traverse towards the random vertex (left)
        next_vert2 = current_vertex.right_child
        #print 'rand is to left of current', getattr(vertex_rand, axis_.axis), '<=', getattr(current_vertex, axis_.axis)
    if next_vert1 is not None:                      # if there is a child in this direction
        nearst = nearest2(vertex_rand, next_vert1, current_vertex, nearst, axis_.next_)
    if next_vert2 is not None:                      # if there is a child in this direction
        if abs(getattr(current_vertex, axis_.axis) - getattr(vertex_rand, axis_.axis)) > dist(nearst, vertex_rand):
            # prune subtree
            pass
        else:
            nearst = nearest2(vertex_rand, next_vert2, current_vertex, nearst, axis_.next_)
        """prev = abs(getattr(prev_vertex, axis_.next_.axis) - getattr(vertex_rand, axis_.next_.axis))
        curr = abs(getattr(current_vertex, axis_.next_.axis) - getattr(vertex_rand, axis_.next_.axis))
        if prev > curr or prev_vertex == current_vertex:
            # traverse second child if previous traversal was towards random vertex in that axis, or if this is first
            nearst = nearest2(vertex_rand, next_vert2, current_vertex, nearst, axis_.next_)
        else:
            # consider a bounded box exclusion if previous traversal was away from random vertex
            if axis_.axis == 'x':           # get temp point on corner of bounding box
                pt_temp = Vertex(current_vertex.x, prev_vertex.y)
            else:
                pt_temp = Vertex(prev_vertex.x, current_vertex.y)
            if abs(getattr(current_vertex, axis_.axis) - getattr(vertex_rand, axis_.axis)) > dist(nearst, vertex_rand):
                # prune subtree
                pass
            else:
                nearst = nearest2(vertex_rand, next_vert2, current_vertex, nearst, axis_.next_)
            #if dist(pt_temp, vertex_rand) <= dist(nearst, vertex_rand):
            #    nearst = nearest2(vertex_rand, next_vert2, current_vertex, nearst, axis_.next_)
            del pt_temp
        del prev
        del curr"""

    return nearst
##############################################################################################################


def cross_prod(pt1, pt2, pt3):    # get cross product of pt with line connecting two other pts
    if (pt3.y - pt1.y) * (pt2.x - pt1.x) > (pt2.y - pt1.y) * (pt3.x - pt1.x):
        prod_ = 1
    else:
        prod_ = 0
    return prod_
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
    if dist(x_nearest, x_rand) < Dimensions.tree_radius:
        vertex_new = x_rand
        vertex_new_ = trace_inclusivity(x_nearest, vertex_new, goal_set)
    else:
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
    pygame.draw.line(pywindow, color_, (pt_new.x, pt_new.y), (pt_tree.x, pt_tree.y), size)
    pygame.display.flip()
    return
##############################################################################################################


def add_to_kd_tree(vertex_new, node, axis_):                            # use kd tree for spatial sorting, faster near/nearest searching
    #print 'add to kd tree, axis:', axis_.axis
    if getattr(vertex_new, axis_.axis) <= getattr(node, axis_.axis):    # indirect attribute access, if less than node val
        if node.left_child is not None:                                 # if left child already exists
            #print 'traversing left since ', getattr(vertex_new, axis_.axis), '<', getattr(node, axis_.axis)
            temp = node.left_child
            add_to_kd_tree(vertex_new, temp, axis_.next_)               # traverse left. alternate between x and y comparisons
            del temp
        else:
            #print 'adding left child'
            node.left_child = vertex_new                                # else no left child exists, create one
    else:
        if node.right_child is not None:                                # if right child already exists
            #print 'traversing right since ', getattr(vertex_new, axis_.axis), '>', getattr(node, axis_.axis)
            temp = node.right_child
            add_to_kd_tree(vertex_new, temp, axis_.next_)   # traverse right. alternate between x and y comparisons
            del temp
        else:
            #print 'adding right child'
            node.right_child = vertex_new                               # else no right child exists, create one
    return
##############################################################################################################


def near_vertices(vertex_new, current_vertex, k, vertices_near):  # return all vertices within near_radius
    current_vertex.k_near = k                                     # for avoiding checking one vertex multiple times
    if k == 1:
        near_radius = Dimensions.eta
    else:
        near_radius = min(Dimensions.gamma * sqrt(log(k) / k), Dimensions.eta)       # function of numPoints
    #print 'near radius:', near_radius
    #print 'distance btwn vert_new and vert_current:', dist(vertex_new, current_vertex)
    if dist(vertex_new, current_vertex) <= near_radius and (current_vertex.at_goal_set is False):  # if within radius
        vertices_near.append(current_vertex)                                     # add to list of near vertices
    for i in range(len(current_vertex.children)):                                # for all children of this vertex
        if current_vertex.children[i].k_near != k:                               # if that child not checked yet
            vertices_near = near_vertices(vertex_new, current_vertex.children[i], k, vertices_near)  # call function again
    return vertices_near
##############################################################################################################


def near_vertices2(vertex_new, current_vertex, prev_vertex, k, vertices_near, axis_):           # uses kd tree to efficiently return approximate nearest vertex
    if k == 1:
        radius = Dimensions.eta
    else:
        radius = min(Dimensions.gamma * sqrt(log(k) / k), Dimensions.eta)       # function of numPoints
    if radius > dist(current_vertex, vertex_new) > 0 and current_vertex.at_goal_set is False:
        vertices_near.append(current_vertex)

    if getattr(vertex_new, axis_.axis) > getattr(current_vertex,
                                                 axis_.axis):  # if new vertex is to the right of or above the current vertex
        next_vert1 = current_vertex.right_child  # then traverse towards the new vertex (right)
        next_vert2 = current_vertex.left_child
    else:
        next_vert1 = current_vertex.left_child  # else traverse towards the new vertex (left)
        next_vert2 = current_vertex.right_child
    if next_vert1 is not None:  # if there is a child in this direction
        vertices_near = near_vertices2(vertex_new, next_vert1, current_vertex, k, vertices_near, axis_.next_)
    if next_vert2 is not None:  # if there is a child in this direction
        if abs(getattr(current_vertex, axis_.axis) - getattr(vertex_new, axis_.axis)) > radius:
            # prune subtree
            pass
        else:
            vertices_near = near_vertices2(vertex_new, next_vert2, current_vertex, k, vertices_near, axis_.next_)
    return vertices_near
##############################################################################################################


def extend_graph(vertex_rand, robot_root, obstacles, goal_set, pywindow, color_, k, x):  # add vertex and edges to sets for robot i
    #vertex_nearest = nearest(vertex_rand, robot_root, robot_root, k)                    # find nearest vertex in robot i's graph, starting with root
    vertex_nearest2 = nearest2(vertex_rand, robot_root, robot_root, robot_root, x)       # computationally efficient function
    #vertex_new = steer(vertex_nearest, vertex_rand, goal_set)                           # steer that vertex towards x_rand within some radius
    vertex_new2 = steer(vertex_nearest2, vertex_rand, goal_set)
    collision_, intersect = collisions(vertex_nearest2, vertex_new2, obstacles, 'obstacles')
    if collision_ == 0:
        #print 'no collision'
        add_to_kd_tree(vertex_new2, robot_root, x)                   # kd tree for spatial sorting and faster nearest/near searching
        #vertices_near = near_vertices(vertex_new2, robot_root, k, [])
        vertices_near2 = near_vertices2(vertex_new2, robot_root, robot_root, k, [], x)
        for i in range(len(vertices_near2)):                                    # for all near vertices
            collision_, intersect = collisions(vertices_near2[i], vertex_new2, obstacles, 'obstacles')
            if collision_ == 0:                                                # if no collision
                #print 'no collision with near vertex'
                add_edge(vertex_new2, vertices_near2[i], color_, 1, pywindow)    # connect the dots
            else:
                pass  #print 'collision with near vertex'
    else:
        #print 'collision, extend procedure returning None'
        del vertex_new2      # save storage if point not used
        return None
    return vertex_new2
##############################################################################################################


def path_generation(vertex):
    #print 'running path gen 1'
    paths_parents = []                             # list of paths, path = list of vertices
    costs_parents = []                             # list of costs associated with some paths
    paths = []
    costs = []
    #print 'vertex has', len(vertex.parents), 'parents'
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
    #print 'number of paths:', len(paths)
    for i in range(len(paths)):                # for every path in paths
        paths[i].append(vertex)                # add the current vertex to that path
        #vertex.add_path(paths[i])             # add these paths to vertex object to avoid recalculating later
        #vertex.add_cost(costs[i])             # add these costs to vertex object '' '' ''
    if len(paths) == 0:                        # if this vertex is the root node
        paths = [[vertex]]                     # only "path" is itself, and has zero cost
        costs = [0]
    return paths, costs
##############################################################################################################


def path_generation2(vertex):  # tree traversal. get all paths root to vertex. update vertex.path_list[[]] and vertex.costs[]
    if vertex.paths == []:
        paths, costs = path_generation(vertex)      # main path generation procedure
        vertex.paths = list(paths)
        vertex.costs = list(costs)
    else:
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


def set_to(path):                # set two paths equal without making them the same instance of object
    path_ = []
    for i in range(len(path)):   # for whole path
        path_.append(path[i])    # copy element (vertex)
    return path_
##############################################################################################################


def find_optimal_path(paths, costs, opt_path, opt_cost, i):
    changed = False
    if len(opt_path) == 0:                              # if this is first time getting a path for robot i
        opt_path = paths[0]                             # just select first path option. optimizations happen later
        opt_cost = costs[0]
        changed = True
        print 'first path cost:', opt_cost
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


def better_response(pi_, goalpts, vertex, path_prev_i, pywindow, costs_i, i, color_):  # minimize cost while avoiding collisions with paths in pi_
    if vertex is not None:
        if vertex.at_goal_set:                                                     # only obtain new paths if new pt is at goal
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


def update_pt_lists(vertex_new, goal_pts, new_vertices, i):
    if vertex_new is not None:
        if vertex_new.at_goal_set:
            goal_pts[i].append(vertex_new)  # use for easier path_generation() later
    new_vertices[i] = vertex_new            # use for easier checking of whether bot is active
    return goal_pts, new_vertices
##############################################################################################################


def main():
    pywindow, obstacles, axis = init_pywindow('i-Nash Trajectory Final 1')    # set up pygame window, dimensions and obstacles
    start, goal_set, num_robots, robo_colors = user_prompt(pywindow)          # prompt for num bots, start, end positions
    all_bots, active_bots, inactive_bots, paths, costs, paths_prev, goal_pts, path_num = init_arrays(num_robots)
    k = 1; k_ = 8000
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
            paths_prev[i] = list(paths[i])                      # save previous path list before updating list
        for q in range(len(active_bots)):                         # use q for easier j < i and j2 > i calculation
            perform_better_response(q, active_bots, paths_prev, paths, costs, pywindow, new_vertices, goal_pts, robo_colors)
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
                    print 'vertex.paths for all vertices in path', P, ':', goal_pts[i][j].paths[P][F].paths
                    for F2 in range(len(goal_pts[i][j].paths[P])):
                        if goal_pts[i][j].paths[P][F] == goal_pts[i][j].paths[P][F2]:
                            repeat = True
                #print 'robot', i, ', goalpt', j, ',path', P, 'cost', string
                print
                display_path(goal_pts[i][j].paths[P], pywindow, Colors.dark_green)  # display
                print 'repeat is ', repeat
    return
##############################################################################################################


if __name__ == '__main__':
    main()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
