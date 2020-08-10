from math import *
from classes import Vertex, Trajectory, Settings
from dynamics import solve_bvp_4d, call_bvp_2d, get_discrete_states, bvp_fixed_tf

thing = [[1, 2], None, [3, 4], None, None]
print thing
thing.remove(None)
print thing
thing.pop(1)
print thing