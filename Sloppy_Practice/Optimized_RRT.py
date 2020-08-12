#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Cody Dillinger
# This .py script will utilize the optimized RRT algorithm, referred to as RRT*, to generate a single path from starting point to destination
# As the path planning tree forms, it displays the tree to the user in a pygame window using the pygame library
# See https://arxiv.org/abs/1105.1186 for more information regarding the analysis of RRT* and related motion planning algorithms
# In the algorithm, V is the set of vertices created from all samples up to some point in time, here this is an array of point objects
#		    E is the set of edges created between all samples up to some point in time, here this is the pointer from one array element to another
#####################################################################################################################################

import os, sys, random, math, pygame, time    # import necessary libraries
from pygame.locals import *
from math import*
pygame.init()                             # initialize usage of pygame

length = 500                              # set constants for pygame window size
width = 700
maxVelocity = 10                          # max velocity of drone
radius = 10				  # constant radius for steer() function
xStart = 450; yStart = 50
xEnd= 50; yEnd = 650
totalSpace = (width*length) - (280*20*2)  # total size of window minus size of obstacles
unitBall = pi				  # see https://arxiv.org/abs/1105.1186
gamma = 2 * sqrt(3) * sqrt( (totalSpace / unitBall) )   # RRT* asymptotically stable if gamma* > (2*(1+1/d))^(1/d)*(u(Xfree)/ζd)^(1/d)
destRadius = 10				  # radius within destination to be considered "at destination"

white = 255, 255, 255; black = 0, 0, 0    # RGB color values, black for obstacles
red =   255, 0, 0;     green = 0, 255, 0  # green and red for destination and starting point respectively
blue =  0, 0, 255;     pink =  200, 20, 240

pygame.init()                                           # initialize usage of pygame
pyWindow = pygame.display.set_mode( (length, width) )   # create pygame display

#######################################
class Point:                             # tree array will contain these point objects
  x = 0
  y = 0
  last = None
  cost = 0
  time = 0
  def __init__(self, xVal, yVal):
    self.x = xVal
    self.y = yVal
#######################################
def distance( pt1, pt2 ):                # return distance between two points from two point objects
  return sqrt( (pt1.x - pt2.x)*(pt1.x - pt2.x) + (pt1.y - pt2.y)*(pt1.y - pt2.y) )
#######################################
def sampleFree():		         # return pseudo random point object
  xRand = Point( random.random()*length, random.random()*width )
  #add bias towards destination ? Right now there is bias towards center technically, which works well for current layout
  return xRand
#######################################
def nearest(xRand1, tree1):	         # return point object from tree which is closest to random sampled point
  iClosest = 0				 # (brute force method - check all vertices)
  for i in range(len(tree1)):
    if ( distance(tree1[i], xRand1) < distance(tree1[iClosest], xRand1) ):
      iClosest = i
  return tree1[iClosest]
#######################################
def steer(xNearest1, xRand1, radius1):            # return point object in direction of random sample but within radius of nearest vertex
  if (distance(xNearest1,xRand1) > radius1):

    deltaY = (xRand1.y - xNearest1.y)
    deltaX = (xRand1.x - xNearest1.x)

#    atan2() function already does all of this
#    m = deltaY / deltaX
#    theta = atan(abs(m))			  # note: used abs(theta) due to inverted y axis

#    if deltaX == 0:				  # for unlikely case of random sample having same x val
#      xNew = xNearest1.x
#      if deltaY > 0:
#        yNew = xNearest1.y + radius1
#      elif deltaY < 0:
#        yNew = xNearest1.y - radius1
#    if deltaY == 0:				  # for unlikely case of random sample having same y val
#      yNew = xNearest1.y
#      if deltaX > 0:
#        xNew = xNearest1.x + radius1
#    elif deltaX < 0:
#       xNew = xNearest1.x - radius1

#    if deltaX > 0:				  # 4 cases for 4 quadrants
#      if deltaY > 0:
#        xNew = xNearest1.x + radius1*cos(theta)
#        yNew = xNearest1.y + radius1*sin(theta)
#      else:
#        xNew = xNearest1.x + radius1*cos(theta)
#        yNew = xNearest1.y - radius1*sin(theta)
#    else:
#      if deltaY > 0:
#        xNew = xNearest1.x - radius1*cos(theta)
#        yNew = xNearest1.y + radius1*sin(theta)
#      else:
#        xNew = xNearest1.x - radius1*cos(theta)
#        yNew = xNearest1.y - radius1*sin(theta)

    theta = atan2(deltaY, deltaX)
    xNew = xNearest1.x + radius1*cos(theta)
    yNew = xNearest1.y + radius1*sin(theta)
    xNew1 = Point(xNew, yNew)
  else:
    xNew1 = xRand1
  return xNew1
#######################################
def collision(pt, ptNew):	   # checking for intersection, not made to easily scale to 3+ dimensions
  if (ptNew.x <= 0 or ptNew.x >= length or ptNew.y <= 0 or ptNew.y >= width): #if ptNew out of window then collision
    return 1
  x1=0;   x2=280; y1=500; y2=520   # obstacle 1 rectangular boundaries
  x3=220; x4=600; y3=250; y4=270   # obstacle 2 rectangular boundaries
  deltaY = ptNew.y - pt.y
  deltaX = ptNew.x - pt.x
  m = deltaY / deltaX
  mInv = 1 / m
  if (
    (pt.y + m*(x1 - pt.x) < y2 and pt.y + m*(x1 - pt.x) > y1 ) or
    (pt.y + m*(x2 - pt.x) < y2 and pt.y + m*(x2 - pt.x) > y1 ) or
    (pt.x + mInv*(y1 - pt.y) < x2 and pt.x + mInv*(y1 - pt.y) > x1 ) or
    (pt.x + mInv*(y2 - pt.y) < x2 and pt.x + mInv*(y2 - pt.y) > x1 )
    ):
    return 1                       # if hit obstacle 1
  if (
    (pt.y + m*(x3 - pt.x) < y4 and pt.y + m*(x3 - pt.x) > y3 ) or
    (pt.y + m*(x4 - pt.x) < y4 and pt.y + m*(x4 - pt.x) > y3 ) or
    (pt.x + mInv*(y3 - pt.y) < x4 and pt.x + mInv*(y3 - pt.y) > x3 ) or
    (pt.x + mInv*(y4 - pt.y) < x4 and pt.x + mInv*(y4 - pt.y) > x3 )
    ):
    return 1                       # if hit obstacle 2
  return 0
#######################################
def near(rad, tree1, xNew1):       # find vertices that are within radius of xNew1 (brute force method - check all vertices)
  nearVertices = []
  for i in range(len(tree1)):
    if (distance(tree1[i], xNew1) < rad):
      nearVertices.append(tree1[i])
  return nearVertices
#######################################
def cost(pt):			   # probs dont need this
  cost = pt.last.cost + distance(pt, pt.last)
  return cost
#######################################
def addEdge(pt1, pt2, color, size):	   	   #pts are just element nums. pt1 to pt2 - direction is important
  pygame.draw.line(pyWindow, color, [pt1.x, pt1.y], [pt2.x, pt2.y], size)
  pygame.display.flip()
  return 0

#####################################################################################################################################
#####################################################################################################################################
def main():
  #put these into initPygame() function
  pyWindow.fill(white)                                       # set background of pygame window to white  
  pygame.draw.circle(pyWindow, red, (xStart, yStart), 10, 0) # display starting point with red circle
  pygame.draw.circle(pyWindow, green, (xEnd, yEnd), 10, 0)   # display destination point with green circle
  pygame.draw.rect(pyWindow, black, (0, 500, 280, 20), 0)    # display black rectangle obstacle
  pygame.draw.rect(pyWindow, black, (220, 250, 280, 20), 0)  # display black rectangle obstacle
  pygame.display.flip()                                      # update display with these new shapes

  vertexNum = input('Enter number of vertices for tree ')    # adjustable number of tree vertices each time you run the code

  tree = []; tree.append( Point(xStart,yStart) )             # create new tree with starting vertex at 250, 150
  target = Point(xEnd, yEnd)                                 # create destination target at 450, 600
  tree[0].last = tree[0]				     # first vertex .last points to itself
  numVertices = 1					     # starting with one vertex
  nearDest = []						     # stores points arbitrarily close to the destination
  
  while numVertices < vertexNum:
    xRand = sampleFree()				     # get random sample within pygame window size, as a point object
    xNearest = nearest(xRand, tree)			     # get vertex nearest to the random sample, as point object
    xNew = steer(xNearest, xRand, radius)		     # get point in direction of xrand from xNearest (if it is too far away), as a point object
    #print 'xNear = ', floor(xNearest.x), floor(xNearest.y)
    #print 'xRand = ', floor(xRand.x), floor(xRand.y)
    #print 'xNew1 = ', floor(xNew.x), floor(xNew.y)
    if (collision(xNearest, xNew) == 0):		     # if no collision
      numVertices = numVertices+1			     # number of vertices or nodes, in the tree
      nearRadius = min(gamma * sqrt(log(numVertices) / numVertices),  radius+5)
      nearVertices = near(nearRadius, tree, xNew)	     # return point objects. nearRadius is function of num existing samples
      tree.append(xNew)				  	     # append this point element to array regardless of path
      xMin = xNearest;					     # variable for lowest cost point, initializing under assumption of xNearest
      costMin = xMin.cost		    	 	     # variable for lowest cost. May be changed for one of Near vertices
      for j in range(len(nearVertices)):		     # connect along a minimum cost path by checking near vertices
        costNear = nearVertices[j].cost + distance(nearVertices[j],xNew)     # cost of path through near vertex
        if (collision(tree[j],xNew) == 0 and costNear < costMin):            # if near path is lower cost and no collision
          xMin = nearVertices[j]					     # update xMin for this shorter path
          costMin = costNear						     # update costMin for this shorter path
      xNew.last = xMin
      xNew.cost = costMin
      addEdge(xMin, xNew, black, 1)					     # add edge on pygame visual AND update parent
      for j in range(len(nearVertices)):				     # rewire tree / update parents if xnear is not on its shortest path
        newCost = xNew.cost + distance(xNew,tree[j])			     # potential new cost of xNear
        if (collision(nearVertices[j],xNew) == 0 and newCost < nearVertices[j].cost):  # if potential new path < that points existing path
          addEdge(nearVertices[j], nearVertices[j].last, white, 1)	     # erase edge on pygame visual
          nearVertices[j].last = xNew					     # adjust parent node
          addEdge(xNew, nearVertices[j], black, 1)			     # add edge on pygame visual

    if distance(xNew, target) < destRadius:
      nearDest.append(xNew)

  # end while loop ######################
  destMin = nearDest[0]
  for j in (range(len(nearDest) - 1)):
    destMin = min(destMin, nearDest[j+1])
  path = destMin
  i = 0
  while not(path.x == xStart and path.y == yStart):
    addEdge(path, path.last, blue, 5)
    path = path.last

  return

##################################################################################################################################
if __name__ == '__main__':
  main()
  running = True
  while running:
    for event in pygame.event.get():
      if event.type == pygame.QUIT:
        running = False
