#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Cody Dillinger

import os, sys, random, math, pygame, time    # import necessary libraries
from pygame.locals import *
from math import*
pygame.init()                                 # initialize usage of pygame

class point:
    def __init__(self, xVal, yVal):
        self.x = xVal
        self.y = yVal
    x = 0
    y = 0
    last = None
    cost = 0
    time = 0

class edge:
    def __init__(self, pt_1, pt_2):
        self.pt1 = pt_1
        self.pt2 = pt_2
    pt1 = 0
    pt2 = 0

def main():
    point1 = point(1, 1)
    point2 = point(2, 2)
    edge1 = edge(point1, point2)
    print edge1
    return

##################################################################################################################################
  if __name__ == '__main__':
      main()
      running = True
      while running:
          for event in pygame.event.get():
              if event.type == pygame.QUIT:
                  running = False