# Cody Dillinger - Cody.M.Dillinger@gmail.com
# This .py script will utilize the optimized RRT algorithm, referred to as RRT*, to generate a single path from starting point to destination
# As the path planning tree forms, it displays the tree to the user in a pygame window using the pygame library
# See https://arxiv.org/abs/1105.1186 for more information regarding the analysis of RRT* and related motion planning algorithms
#####################################################################################################################################
# Below is a summary of this RRT* algorithm:
# Initialize a tree with no edges as a starting point, where the tree is an array of point objects
# Then for some number n samples,
# sampleFree()  take a random sample in space
# nearest()     find the nearest vertex in the tree to that random sample
# steer()       find point in the direction of random sample but within some radius of the nearest vertex
# collision()   If there is no obstacle on the edge between steer() point and nearest() vertex,
# near()        check for other vertices in tree that are within some radius of the steer() point, where the radius is a functin of the number of samples
# Add the steer() point to the tree
# costCompare() for each vertices in near() check if the total path cost from starting point to this vertex is less than through the nearest() vertex
# addEdge()     add edge between steer() point and the chosen lowest-cost path from list of near() vertices
# updateTree()  update parents of the vertices
#####################################################################################################################################
#####################################################################################################################################

import sys, random, math, pygame, time    # import necessary libraries
from pygame.locals import *
from math import*
pygame.init()                             # initialize usage of pygame

length = 500                              # set constants for pygame window size
width = 700
maxVelocity = 10                          # max velocity of drone

white = 255, 255, 255; black = 0, 0, 0;   # RGB color values, black for obstacles
red =   255, 0, 0;     green = 0, 255, 0  # green and red for destination and starting point respectively
blue =  0, 0, 255;     pink =  200, 20, 240

pygame.init()                                           # initialize usage of pygame
pyWindow = pygame.display.set_mode( (length, width) )   # create pygame display

class point:                        # tree array will contain these point objects
  x = 0
  y = 0
  last = None
  cost = 0
  time = 0
  def __init__(self, xVal, yVal):
    self.x = xVal
    self.y = yVal
 
def distance( pt1, pt2 ):          # return distance between two points from two point objects
  return sqrt( (pt1.x - pt2.x)^2 - (pt1.y - pt2.y)^2 )

def sampleFree():
  return 0

def nearest():
  return 0

def steer():
  return 0

def collision():
  return 0

def near():
  return 0

def costCompare():
  return 0

def addEdge():
  return 0
  
def updateTree():
  return 0

##################################################################################################################################
def main():
  vertexNum = input('Enter number of vertices')             # adjustable number of tree vertices each time you run the code
  
  pyWindow.fill(white)                                      # set background of pygame window to white  
  pygame.draw.circle(pyWindow, red,   (250, 150), 10, 0)    # display starting point with red circle
  pygame.draw.circle(pyWindow, green, (450, 600), 10, 0)    # display destination point with green circle
  pygame.draw.rect(pyWindow, black, (0, 400, 280, 20), 0)   # display black rectangle obstacle
  pygame.draw.rect(pyWindow, black, (220, 250, 280, 20), 0) # display black rectangle obstacle
  pygame.display.flip()                                     # update display with these new shapes
  
  tree = []; tree.append( point(250, 150) )                 # create new tree with starting vertex at 250, 150
  target = point(450, 600)                                  # create destination target at 450, 600
##################################################################################################################################
if __name__ == '__main__':
  main()
  running = True
  while running:
    for event in pygame.event.get():
      if event.type == pygame.QUIT:
        running = False
