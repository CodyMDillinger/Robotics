#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Cody Dillinger
# This contains functions relating to state dynamics, boundary value problems, and bang-bang controllers

from classes import Settings, Trajectory, Vertex
from math import *
import math
from pygame_simulator import get_position
##############################################################################################################


def bvp_fixed_tf(tf, pt1, pt2, dim1, dim2):  # solve 2-pt boundary value problem for 2 dimension, fixed final time
    # these calculations are solutions of the system of equations resulting from "bang-bang" but smaller slope
    a = getattr(pt1, dim2) - getattr(pt2, dim2)
    b = (2.0 * getattr(pt2, dim2) * tf) + (2.0 * getattr(pt1, dim1)) - (2.0* getattr(pt2, dim1))
    c = (getattr(pt2, dim1) * tf) - (getattr(pt1, dim1) * tf) - (.5 * getattr(pt1, dim2) * (tf**2)) - (.5 * getattr(pt2, dim2) * (tf**2.0))
    sqrt_term = (b**2.0) - (4.0*a*c)
    if sqrt_term < 0:
        return None, None, None, None
    else:
        if a == 0:     # if initial and final velocity happen to be the same
            ts1 = tf / 2.0
            ts2 = tf / 2.0
        else:
            ts1 = (-b + sqrt(sqrt_term)) / (2.0*a)
            ts2 = (-b - sqrt(sqrt_term)) / (2.0*a)
        um1 = (-a) / ((ts1 * 2.0) - tf)
        um2 = (-a) / ((ts2 * 2.0) - tf)
    return ts1, um1, ts2, um2
##############################################################################################################


# bang-bang control is optimal-time solution
def solve_bvp_2d(u, pt1, pt2, dim1, dim2):   # solve 2-pt boundary value problem for 2 dimension, free final time
    u_m = u * Settings.force_normalized
    Q = (getattr(pt2, dim2) - getattr(pt1, dim2)) / u_m
    a = u_m
    b = 2.0 * getattr(pt1, dim2)
    c = getattr(pt1, dim1) - getattr(pt2, dim1) - (getattr(pt1, dim2) * Q) - (.5 * u_m * (Q ** 2.0))
    sqrt_term = (b ** 2.0) - (4.0 * a * c)
    if sqrt_term < 0:
        return None, None, None, None
    else:
        ts1 = (-b + sqrt(sqrt_term)) / (2.0 * a)
        tf1 = (2.0 * ts1) - Q
        ts2 = (-b - sqrt(sqrt_term)) / (2.0 * a)
        tf2 = (2.0 * ts2) - Q
    return ts1, ts2, tf1, tf2
##############################################################################################################


def choose_switch_times(t1, t2, t3, t4, tf):    # selecting the two switch times from the 4 calculated in bvp_fixed
    if 0 < t1 < t2 < tf:
        ts1 = t1
        ts2 = t2
    elif 0 < t3 < t4 < tf:
        ts1 = t3
        ts2 = t4
    elif 0 < t2 < t1 < tf:
        ts1 = t2
        ts2 = t1
    elif 0 < t4 < t3 < tf:
        ts1 = t4
        ts2 = t3
    else:
        ts1 = None
        ts2 = None
    return ts1, ts2
##############################################################################################################


def choose_switch_time(ts1, tf1, ts2, tf2):   # selecting the switch time from the 2 calculated in bvp_2d
    if 0 < ts1 < tf1:
        ts = ts1
        tf = tf1
    elif 0 < ts2 < tf2:
        ts = ts2
        tf = tf2
    else:
        ts = None
        tf = None
    return ts, tf
##############################################################################################################


def choose_switch_time_fixed(ts1, ts2, u1, u2, tf):
    if 0 < ts1 < tf:
        ts = ts1
        u = u1
    elif 0 < ts2 < tf:
        ts = ts2
        u = u2
    else:
        ts = None
        u = None
    return ts, u
##############################################################################################################


def call_fixed(u_x, u_y, tf_x, tf_y, ts_x, ts_y, pt1, pt2):  # call fixed-final-time 2-pt BVP for x or y
    if tf_y > tf_x:            # if y direction has larger tf
        u_y = u_y * Settings.force_normalized
        tf = tf_y              # then choose tf_y since x cannot reach goal by that time
        ts1_y = ts_y
        ts2_y = ts_y
        tsx1, ux1, tsx2, ux2 = bvp_fixed_tf(tf, pt1, pt2, 'x', 'x_vel')
        ts1_x, u_x = choose_switch_time_fixed(tsx1, tsx2, ux1, ux2, tf)
    elif tf_y < tf_x:           # if x direction has larger tf
        u_x = u_x * Settings.force_normalized
        tf = tf_x               # then choose tf_x since y cannot reach goal by that time
        ts1_x = ts_x
        ts2_x = ts_x
        tsy1, uy1, tsy2, uy2 = bvp_fixed_tf(tf, pt1, pt2, 'y', 'y_vel')
        ts1_y, u_y = choose_switch_time_fixed(tsy1, tsy2, uy1, uy2, tf)
    else:                      # just so happened to have same exact final time calculation? weird
        tf = tf_x
        ts1_x = ts_x
        ts1_y = ts_y
        u_x = u_x * Settings.force_normalized
        u_y = u_y * Settings.force_normalized
    return ts1_x, ts1_y, tf, u_x, u_y
##############################################################################################################


def call_bvp_2d(u, pt1, pt2, dim1, dim2):
    ts1, ts2, tf1, tf2 = solve_bvp_2d(u, pt1, pt2, dim1, dim2)
    if ts1 is None:                              # if first attempt had no solution
        u = -u                                   # then reverse u order is guaranteed to have solution
        ts1, ts2, tf1, tf2 = solve_bvp_2d(u, pt1, pt2, dim1, dim2)
    if ts1 is None:
        return None, None
    else:
        ts, tf = choose_switch_time(ts1, tf1, ts2, tf2)
    if ts is None:
        u = -u                             # then attempt u(t) reverse order
        ts1, ts2, tf1, tf2 = solve_bvp_2d(u, pt1, pt2, dim1, dim2)
        if ts1 is None:
            return None, None
        else:
            ts, tf = choose_switch_time(ts1, tf1, ts2, tf2)
    return ts, tf, u
##############################################################################################################


def solve_bvp_4d(pt1, pt2):  # 4-D 2-pt BVP for double integrator, fixed init/final state, via uncoupled 2-D solving
    if pt2.x > pt1.x:        # u(t) optimal bang-bang controller +- order, first attempt determined by x direction
        u_x = 1.0              # sign of first control input u_x
    else:
        u_x = -1.0
    if pt2.y > pt1.y:        # u(t) optimal bang-bang controller +- order, first attempt determined by y direction
        u_y = 1.0              # sign of first control input u_y
    else:
        u_y = -1.0
    ts_x, tf_x, u_x = call_bvp_2d(u_x, pt1, pt2, 'x', 'x_vel')  # solve for optimal trajectory for x dimensions
    ts_y, tf_y, u_y = call_bvp_2d(u_y, pt1, pt2, 'y', 'y_vel')  # solve for optimal trajectory for y dimensions
    ts1_x, ts1_y, tf, u_x, u_y = call_fixed(u_x, u_y, tf_x, tf_y, ts_x, ts_y, pt1, pt2)    # choose 2d BVP soln that has larger tf
    # bang-bang controller is saved same way as bang-off-bang with two switch times that are equal
    trajectory = Trajectory(u_x, ts1_x, u_y, ts1_y, tf)
    # get_discrete_states(pt1, pt2, trajectory)
    get_equi_time_discrete_states(pt1, pt2, trajectory)
    return trajectory
##############################################################################################################


# similar to get_discrete_states() except this uses global time_step
# for easier inter-robot collision checking
def get_equi_time_discrete_states(pt1, pt2, traj):
    step = Settings.time_step * 5.0              # only used for collision checking, so don't need as many points
    num_steps = int(math.ceil(traj.t_f / step))  # larger time step to decrease computations
    states = [None] * num_steps
    for i in range(num_steps):
        time_ = i * step
        if time_ >= traj.t_f or i == num_steps - 1:
            states[i] = pt2
        else:
            states[i] = get_position(pt1, time_, traj)
        traj.add_statevals(states[i])
    return
##############################################################################################################


# uses global number of discrete states between two points
def get_discrete_states(pt1, pt2, traj):  # use switch times to sample from a continuous trajectory and save values
    ux = traj.u_x1
    uy = traj.u_y1
    v1_x = pt1.x_vel + (ux * traj.ts1_x)
    v1_y = pt1.y_vel + (uy * traj.ts1_y)
    x1 = pt1.x + (pt1.x_vel * traj.ts1_x) + (.5 * ux * (traj.ts1_x**2))
    y1 = pt1.y + (pt1.y_vel * traj.ts1_y) + (.5 * uy * (traj.ts1_y**2))
    time_step = traj.t_f / (traj.num_disc_vals - 1)
    for i in range(traj.num_disc_vals):
        time = time_step * i
        if time < traj.ts1_x:
            v_x = pt1.x_vel + (ux * time)
            x = pt1.x + (pt1.x_vel * time) + (.5 * ux * (time**2))
        else:
            v_x = v1_x - (ux * (time - traj.ts1_x))
            x = x1 + (v1_x * (time - traj.ts1_x)) - (.5 * ux * ((time - traj.ts1_x) ** 2))
        if time < traj.ts1_y:
            v_y = pt1.y_vel + (uy * time)
            y = pt1.y + (pt1.y_vel * time) + (.5 * uy * (time ** 2))
        else:
            v_y = v1_y - (uy * (time - traj.ts1_y))
            y = y1 + (v1_y * (time - traj.ts1_y)) - (.5 * uy * ((time - traj.ts1_y) ** 2))
        if i < (traj.num_disc_vals - 1):
            traj.states[i].x = x
            traj.states[i].y = y
            traj.states[i].x_vel = v_x
            traj.states[i].y_vel = v_y
        else:
            traj.states[i] = pt2
    return
##############################################################################################################
