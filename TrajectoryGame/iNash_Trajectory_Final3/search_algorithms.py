#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Cody Dillinger
# Functions related to finding near vertices and nearest vertices to a given vertex

from math import*
from classes import Dimensions
from geometry_procedures import dist, norm
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
def nearest2(vertex_rand, current_vertex, prev_vertex, nearest_,
             axis_):  # uses kd tree to efficiently return approximate nearest vertex
    # print 'optimized nearest, axis:', axis_.axis
    if norm(current_vertex, vertex_rand) < norm(nearest_, vertex_rand) and current_vertex.at_goal_set is False:
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

    Actual design:
    traverse in direction of current vertex towards random vertex
    if the current vertex is further in the previous axis direction from vert_rand than the closest is in Euclidean distance,
    then don't traverse in direction of current vertex away from random vertex
    this method allows exclusions of certain parts of the tree (pruning)
    which is great for large numbers of vertices
    """

    if getattr(vertex_rand, axis_.axis) > getattr(current_vertex, axis_.axis):  # if random vertex is to the right of or above the current vertex
        next_vert1 = current_vertex.right_child  # then traverse towards the random vertex (right)
        next_vert2 = current_vertex.left_child
    else:
        next_vert1 = current_vertex.left_child   # else traverse towards the random vertex (left)
        next_vert2 = current_vertex.right_child
    if next_vert1 is not None:  # if there is a child in this direction
        nearst = nearest2(vertex_rand, next_vert1, current_vertex, nearst, axis_.next_)
    if next_vert2 is not None:  # if there is a child in this direction
        if abs(getattr(current_vertex, axis_.axis) - getattr(vertex_rand, axis_.axis)) > norm(nearst, vertex_rand):
            # prune subtree
            pass
        else:
            nearst = nearest2(vertex_rand, next_vert2, current_vertex, nearst, axis_.next_)
    return nearst
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
    if radius > norm(current_vertex, vertex_new) > 0 and current_vertex.at_goal_set is False:
        vertices_near.append(current_vertex)
    if getattr(vertex_new, axis_.axis) > getattr(current_vertex, axis_.axis):  # if new vertex is to the right of or above the current vertex
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
