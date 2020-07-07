import os, sys, random, math, pygame, time
from pygame.locals import *
from math import*

class thing:
    def __init__(self, stuff_):
        self.stuff = stuff_

    stuff = 0

a = thing(1)
b = thing(2)
c = thing(3)

array = [a, b, c]

array2 = []
for i in range(len(array)):  # for whole path
    array2.append(array[i])  # copy element (vertex)

for i in range(3):
    print array[i].stuff
    print array2[i].stuff

array[0] = thing(100)

for i in range(3):
    print array[i].stuff
    print array2[i].stuff

print abs(-5)