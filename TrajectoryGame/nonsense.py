from collections import namedtuple
from operator import itemgetter
from pprint import pformat
from classes import Vertex

class Node(namedtuple('Node', 'location left_child right_child')):
    def __repr__(self):
        return pformat(tuple(self))


class Obstacle:
    def __init__(self):
        pass
    points = []


class Point:
    def __init__(self, xVal, yVal):
        self.x = xVal
        self.y = yVal
        self.leaves = []

    def add_leaf(self, leaf):
        self.leaves.append(leaf)

    x = 0
    y = 0
    next_ = None


class Iteration:
    def __init__(self):
        pass
    a = 'a'
    b = 'b'
    c = 'c'


def kdtree(point_list, depth):
    if not point_list:
        return None
    k = 2
    axis = depth
    point_list.sort(key=itemgetter(axis))
    median = len(point_list) // 2

    return Node(
        location=point_list[median],
        left_child=kdtree(point_list[:median], depth + 1),
        right_child=kdtree(point_list[median + 1:], depth + 1)
    )


def ccw(A, B, C):
    return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x)


# Return true if line segments AB and CD intersect
def intersect(A, B, C, D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)


def thing():
    return 1, 2, 3, 4, 5


class colors:
    def __init__(self):
        pass
    val = 255, 255, 255
    black = 0, 0, 0
    grey = 128, 128, 128
    light_green = 0, 255, 0


def main():

    return

if __name__ == '__main__':
    main()
