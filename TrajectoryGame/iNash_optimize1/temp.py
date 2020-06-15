import os, sys, random, math, pygame, time
from pygame.locals import *
from math import*

def set_to(path):                # set two paths equal without making them the same instance of object
    path_ = []
    for i in range(len(path)):   # for whole path
        path_.append(path[i])    # copy element (vertex)
    return path_


def function(orig_i):
    newthing = orig_i
    newthing = 70
    return newthing

thing = [1, 2, 3, 4, 5]
thing2 = set_to(thing)
thing2[2] = 47
print thing
print thing2

original = [5, 5, 5, 5, 5]
original2 = [5, 5, 5, 5, 5]
original2[3] = function(original[3])
print original
print original2