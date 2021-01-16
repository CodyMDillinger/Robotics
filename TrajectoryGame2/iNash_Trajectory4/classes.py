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


class Trajectory:  # storing policy values for bang-bang control and state trajectory
    def __init__(self, ux1, ts1x, uy1, ts1y, tf):
        self.u_x1 = ux1
        self.ts1_x = ts1x
        self.u_y1 = uy1
        self.ts1_y = ts1y
        self.t_f = tf
        self.num_disc_vals = 15  # num discrete pts to include on trajectory. including initial and final values
        self.states2 = []        # discrete samples from continuous trajectory
        self.states = []         # discrete samples from continuous trajectory
        for i in range(self.num_disc_vals):
            self.states.append(Vertex(0, 0, 0, 0))

    def add_states2(self, state_):  # since we do not know exact number of states with global time_step
        self.states2.append(state_)

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
    light_blue = (173, 216, 255)
    navy_blue = (0, 0, 128)
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
    robo_size = 8                         # size of dot representing robot in position plot
    robo_size_vel = 5                     # size of dot representing robot in velocity plot
    sample_bias_iterator = 1.0 / 4.0      # fraction of random sampling that uses biased velocity sampling
    robo_vel_max = 75                     # max velocity for all robots, size of velocity plot changes with this
    robo_finish_vel = 50                  # magnitude of maximum velocity allowed at goal set
    goal_set_size = 14                    # half width of position goal set box
    robo_mass = 1                         # mass of point-mass robot, for 4-d 2-pt boundary value solving
    max_actuator_force = 100.0             # maximum force used in bang-bang control
    force_normalized = max_actuator_force / robo_mass   # acceleration = force / mass
    button_text = ['Start 2D Sim',
                   'Start 3D Sim']        # INITIALIZE button text. Button location defined in Dimensions
    inter_robot_col_dist = robo_size * 1.2  # inter-robot distance to be considered a collision
    time_step = .1                        # how frequently each discrete robot position is calculated (seconds)
    simulation_speed = time_step / 2.0    # wait() time between GUI updates in simulation (not a resolution adjustment)
    collision_cost = 9999999.0            # inf cost for inter-robot collisions
##############################################################################################################


class Dimensions:  # pywindow, obstacles, radii, theorem 38 calculations
    def __init__(self):
        pass

    # size of pywindow display
    window_length = 650   # length is y length
    window_width = 1000    # width is x length
    line_width = 3
    tree_radius = 50.0    # size of radius for steering point towards random vertex
    eta = tree_radius * 1.000  # size of radius in comparison for min() function in near() function

    # Static Obstacle Definitions
    # Algorithm works for any number of obstacles with any number of vertices
    # Code will automatically account for any obstacle listed here.
    # Pay attention to vertex order
    window = [Vertex(0, 0, 0, 0), Vertex(0, window_length, 0, 0), Vertex(window_width, window_length, 0, 0),
              Vertex(window_width, 0, 0, 0)]
    obs_list = []
    obs_list.append(window)
    #obs_list.append([Vertex(100, 125, 0, 0), Vertex(100, 145, 0, 0), Vertex(150, 155, 0, 0), Vertex(175, 110, 0, 0), Vertex(150, 125, 0, 0)])
    #obs_list.append([Vertex(100, 305, 0, 0), Vertex(80, 325, 0, 0), Vertex(145, 355, 0, 0), Vertex(175, 290, 0, 0), Vertex(130, 325, 0, 0)])
    #obs_list.append([Vertex(110, 485, 0, 0), Vertex(70, 505, 0, 0), Vertex(155, 535, 0, 0), Vertex(165, 470, 0, 0), Vertex(140, 460, 0, 0)])
    obs_list.append([Vertex(550+40, 150, 0, 0), Vertex(550+40, 120, 0, 0), Vertex(320+40, 120, 0, 0), Vertex(320+40, 330, 0, 0),
                     Vertex(520+40, 330, 0, 0), Vertex(520+40, 480, 0, 0), Vertex(320+40, 480, 0, 0), Vertex(320+40, 510, 0, 0),
                     Vertex(550+40, 510, 0, 0), Vertex(550+40, 300, 0, 0), Vertex(350+40, 300, 0, 0), Vertex(350+40, 150, 0, 0)])
    obs_list.append([Vertex(650+40, 120, 0, 0), Vertex(680+40, 120, 0, 0), Vertex(680+40, 480, 0, 0), Vertex(850+40, 480, 0, 0),
                     Vertex(850+40, 120, 0, 0), Vertex(880+40, 120, 0, 0), Vertex(880+40, 510, 0, 0), Vertex(650+40, 510, 0, 0)])
    P = []
    P.append([Vertex(220 + 40, 120, 0, 0), Vertex(10 + 20, 120, 0, 0), Vertex(10 + 20, 510, 0, 0),
              Vertex(40 + 20, 510, 0, 0),
              Vertex(40 + 20, 330 - 10, 0, 0), Vertex(240 + 20, 330 - 10, 0, 0)])
    P.append([Vertex(40 + 20, 150, 0, 0), Vertex(210 + 20, 150, 0, 0), Vertex(210 + 20, 300 - 10, 0, 0),
              Vertex(40 + 20, 300 - 10, 0, 0)])
    #obs_list.append([Vertex(470, 120, 0, 0), Vertex(550, 120, 0, 0), Vertex(575, 140, 0, 0), Vertex(535, 155, 0, 0), Vertex(460, 160, 0, 0)])
    #obs_list.append([Vertex(710, 120, 0, 0), Vertex(750, 140, 0, 0), Vertex(775, 170, 0, 0), Vertex(735, 180, 0, 0), Vertex(715, 160, 0, 0)])
    #obs_list.append([Vertex(910, 120, 0, 0), Vertex(950, 130, 0, 0), Vertex(975, 180, 0, 0), Vertex(935, 170, 0, 0), Vertex(915, 170, 0, 0)])
    #obs_list.append([Vertex(925, 480, 0, 0), Vertex(950, 450, 0, 0), Vertex(975, 550, 0, 0), Vertex(935, 550, 0, 0), Vertex(930, 530, 0, 0)])
    #obs_list.append([Vertex(710, 490, 0, 0), Vertex(750, 490, 0, 0), Vertex(755, 520, 0, 0), Vertex(715, 530, 0, 0)])
    #obs_list.append([Vertex(835, 540, 0, 0), Vertex(865, 500, 0, 0), Vertex(820, 490, 0, 0), Vertex(825, 545, 0, 0)])
    #obs_list.append([Vertex(700, 305, 0, 0), Vertex(710, 365, 0, 0), Vertex(745, 355, 0, 0), Vertex(765, 290, 0, 0), Vertex(730, 285, 0, 0)])
    #obs_list.append([Vertex(910, 305, 0, 0), Vertex(910, 325, 0, 0), Vertex(945, 355, 0, 0), Vertex(975, 260, 0, 0), Vertex(930, 295, 0, 0)])
    #obs_list.append([Vertex(330, 505, 0, 0), Vertex(310, 525, 0, 0), Vertex(375, 535, 0, 0), Vertex(465, 510, 0, 0)])
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
    obs_area = 0#2000.0  # estimated average obstacle 2-D area
    obs_volume = obs_area * (
                len(obs_list) - 1) * 4.0 * Settings.robo_vel_max  # total obstacle 4-D lebesgue obstacle space
    total_space = (window_length * window_width * 4.0 * Settings.robo_vel_max)  # total lebesgue space
    free_space = total_space - obs_volume  # total lebesgue free space
    gamma_min = 2.0 * ((1.0 + 1.0 / dimen) ** (1.0 / dimen)) * (
                (free_space / unit_ball) ** (1.0 / dimen))  # from theorems 34 to 39
    gamma = gamma_min * 500.0
