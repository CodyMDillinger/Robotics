from math import *
from classes import Vertex, Trajectory, Settings
from dynamics import solve_bvp_4d, call_bvp_2d, get_discrete_states, bvp_fixed_tf


pt1 = Vertex(5, 16, 20, 50)
pt2 = Vertex(5.1, 5, 20.1, 60)
solve_bvp_4d(pt1, pt2)
