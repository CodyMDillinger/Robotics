#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Cody Dillinger
# Classes used in i-Nash algorithms

from math import *
##############################################################################################################


class Vertex:
    def __init__(self, x_pos, y_pos, x_vel_, y_vel_):  # initialize with all states
        self.x = x_pos          # state 1
        self.y = y_pos          # state 2
        self.x_vel = x_vel_     # state 3
        self.y_vel = y_vel_     # state 4
        self.children = []      # child nodes in tree
        self.parents = []       # parent nodes in tree
        self.paths = []         # all paths to this point
        self.costs = []         # all costs for those paths
        self.trajectories = []  # trajectories to child nodes, includes this point and child point
        self.tree_num = 1       # for determining whether to add to tree1 (from start) or tree2 (from goal)

    def add_trajectory(self, traj):
        self.trajectories.append(traj)

    def add_child(self, child):
        self.children.append(child)

    def add_parent(self, parent):
        self.parents.append(parent)

    def add_path(self, path_):
        self.paths.append(path_)  # for list of paths from root node to this vertex

    def add_cost(self, cost_):
        self.costs.append(cost_)  # for list of costs associated with paths in self.paths

    def pop_path(self, i):
        self.paths.pop(i)

    def pop_cost(self, i):
        self.costs.pop(i)

    # 4 states. 2nd order system coordinates
    x = 0  # state 1
    y = 0  # state 2
    x_vel = 0  # state 3
    y_vel = 0  # state 4
    obs_next = None  # for shape edges (obstacles and goal set)
    at_goal_set = False
    k_nearest = 0  # for avoiding checking points multiple times in recursion searches
    k_near = 0
    left_child = None  # for k-d tree (spatial sorting algorithm, faster near/nearest functions)
    right_child = None
##############################################################################################################


class PathVertex:                       # path defined as a list of these objects
    def __init__(self, vertex_):
        self.vertex = vertex_           # will be set to vertex object
        self.traj_element = None        # store index of trajectory in trajectories
##############################################################################################################


class Trajectory:  # storing policy values for bang-bang control and state trajectory
    def __init__(self, ux1, ts1x, uy1, ts1y, tf):
        self.u_x1 = ux1
        self.ts1_x = ts1x
        self.u_y1 = uy1
        self.ts1_y = ts1y
        self.t_f = tf
        self.num_disc_vals = 15    # num discrete pts to include on trajectory. including initial and final values
        self.statevals = []
        # self.states2 = []        # discrete samples from continuous trajectory
        # self.states = []         # discrete samples from continuous trajectory
        #for i in range(self.num_disc_vals):
            # self.states.append(Vertex(0, 0, 0, 0))

    def add_statevals(self, state_):
        self.statevals.append(state_)

    # def add_states2(self, state_):  # since we do not know exact number of states with global time_step
        # self.states2.append(state_)

    u_x1 = 0    # x direction, first control value before switch
    ts1_x = 0   # switching time for bang-bang controller
    u_y1 = 0    # y direction, first control value before switch
    ts1_y = 0   # switching time for bang-bang controller
    t_f = 0     # arrival time! exciting stuff. wow. it made it all the way there.
##############################################################################################################


class Axes:  # for scalable add_to_kd_tree() function for any number of state space dimensions
    def __init__(self, axis_):
        self.axis = axis_

    axis = None
    next_ = None
##############################################################################################################


class Colors:  # RGB color values
    def __init__(self):
        pass

    white = (255, 255, 255)
    black = (0, 0, 0)
    grey = (128, 128, 128)
    light_green = (0, 255, 0)
    dark_green = (0, 200, 0)
    turquoise = (0, 255, 255)
    light_blue = (0, 0, 255)
    dark_blue = (0, 0, 200)
    light_red = (255, 0, 0)
    dark_red = (200, 0, 0)
    pink = (200, 20, 240)
    purple = (100, 0, 255)
    light_orange = (255, 100, 0)
    dark_orange = (175, 100, 0)
##############################################################################################################


class Color_:  # for easy iterating between colors for unknown number of robot trees
    def __init__(self, clr):
        self.color_ = clr
        self.color_list = []

    def add_color(self, color_new):
        self.color_list.append(color_new)

    next_ = None
    color_ = (0, 0, 0)
##############################################################################################################


class Settings:                           # for adjusting game-play
    def __init__(self):
        pass

    # note: sample bias currently requires inverse to be integer - see sample_free()
    robo_size = 7                         # size of dot representing robot in position plot
    robo_size_vel = 5                     # size of dot representing robot in velocity plot
    sample_bias_iterator = 1.0 / 2.0      # fraction of random sampling that uses biased velocity sampling
    robo_vel_max = 75                     # max velocity for all robots, size of velocity plot changes with this
    robo_finish_vel = 50                  # magnitude of maximum velocity allowed at goal set
    goal_set_size = 14                    # half width of position goal set box
    robo_mass = 1                         # mass of point-mass robot, for 4-d 2-pt boundary value solving
    max_actuator_force = 50.0             # maximum force used in bang-bang control
    force_normalized = max_actuator_force / robo_mass   # acceleration = force / mass
    button_text = ['Start 2D Sim',
                   'Start 3D Sim']        # INITIALIZE button text. Button location defined in Dimensions
    inter_robot_col_dist = robo_size * 2.1  # inter-robot distance to be considered a collision
    time_step = .05                       # how frequently each discrete robot position is calculated (seconds)
    simulation_speed = time_step / 2.0    # wait() time between GUI updates in simulation (not a resolution adjustment)
    collision_cost = 9999999.0            # inf cost for inter-robot collisions
##############################################################################################################


class Dimensions:  # pywindow, obstacles, radii, theorem 38 calculations
    def __init__(self):
        pass

    # size of pywindow display
    window_length = 1200   # length is y length
    window_width = 1200    # width is x length
    line_width = 3
    tree_radius = 40.0    # size of radius for steering point towards random vertex
    eta = tree_radius * 1.0001  # size of radius in comparison for min() function in near() function

    # Static Obstacle Definitions
    # Algorithm works for any number of obstacles with any number of vertices
    # Code will automatically account for any obstacle listed here.
    # Pay attention to vertex order
    window = [Vertex(0, 0, 0, 0), Vertex(0, window_length, 0, 0), Vertex(window_width, window_length, 0, 0),
              Vertex(window_width, 0, 0, 0)]
    obs_list = []
    obs_list.append(window)
    #obs_list.append([Vertex(220, 125, 0, 0), Vertex(220, 145, 0, 0), Vertex(300, 145, 0, 0), Vertex(300, 125, 0, 0)])
    #obs_list.append([Vertex(200+200, 420+400, 0, 0), Vertex(200+200, 490+400, 0, 0), Vertex(280+400, 490+400, 0, 0), Vertex(280+400, 420+400, 0, 0)])
    #obs_list.append([Vertex(100+100, 220+300, 0, 0), Vertex(170+200, 290+300, 0, 0), Vertex(125+200, 200+300, 0, 0), Vertex(115+100, 150+300, 0, 0)])
    #obs_list.append([Vertex(500+300, 120+100, 0, 0), Vertex(480+300, 190+200, 0, 0), Vertex(575+300, 190+200, 0, 0), Vertex(560+300, 140+100, 0, 0)])
    #obs_list.append([Vertex(500, 420, 0, 0), Vertex(550, 490, 0, 0), Vertex(575, 400, 0, 0), Vertex(515, 350, 0, 0)])
    #obs_list.append([Vertex(320+600, 330+600, 0, 0), Vertex(380+700, 345+700, 0, 0), Vertex(400+700, 290+600, 0, 0), Vertex(350+600, 305+600, 0, 0)])

    # Button Definitions
    button_width = 100
    button_height = 100
    button_list = []
    button_list.append([Vertex(window_width, 1, 0, 0),
                        Vertex(window_width, button_height + 1, 0, 0),
                        Vertex(window_width + button_width, button_height + 1, 0, 0),
                        Vertex(window_width + button_width, 1, 0, 0)])
    button_list.append([Vertex(window_width, button_height + 3, 0, 0),
                        Vertex(window_width, (button_height * 2) + 3, 0, 0),
                        Vertex(window_width + button_width, (button_height * 2) + 3, 0, 0),
                        Vertex(window_width + button_width, button_height + 3, 0, 0)])

    # For near() vertices functionality, where radius value initiates:
    dimen = 4.0
    # for 2-D, unit_ball = 3.1415926 * R**2
    # for 4-D:
    R = 1.0  # radius of "unit ball"
    unit_ball = .5 * 3.1415926 * (R ** 4)
    obs_area = 4000.0  # estimated average obstacle 2-D area
    obs_volume = obs_area * (
                len(obs_list) - 1) * 4.0 * Settings.robo_vel_max  # total obstacle 4-D lebesgue obstacle space
    total_space = (window_length * window_width * 4.0 * Settings.robo_vel_max)  # total lebesgue space
    free_space = total_space - obs_volume  # total lebesgue free space
    gamma_min = 2.0 * ((1.0 + 1.0 / dimen) ** (1.0 / dimen)) * (
                (free_space / unit_ball) ** (1.0 / dimen))  # from theorems 34 to 39
    gamma = gamma_min * 500.0
