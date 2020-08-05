#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Cody Dillinger
# This contains functions relating to state dynamics, boundary value problems, and bang-bang controllers

from classes import Settings, Trajectory, Vertex
from math import *
##############################################################################################################


def bvp_fixed_tf(tf, pt1, pt2, dim1, dim2):  # solve 2-pt boundary value problem for 2 dimension, fixed final time
    # these calculations are solutions of the system of equations resulting from "bang-bang" but smaller slope
    a = getattr(pt1, dim2) - getattr(pt2, dim2)
    b = (2 * getattr(pt2, dim2) * tf) + (2 * getattr(pt1, dim1)) - (2 * getattr(pt2, dim1))
    c = (getattr(pt2, dim1) * tf) - (getattr(pt1, dim1) * tf) - (.5 * getattr(pt1, dim2) * (tf**2)) - (.5 * getattr(pt2, dim2) * (tf**2))
    sqrt_term = (b**2) - (4*a*c)
    if sqrt_term < 0:
        #print 'sqrt term < 0 in FIXED calculation, ugh'
        return None, None, None, None
    else:
        if a == 0:     # if initial and final velocity happen to be the same
            #print 'initial and final velocity happen to be the same'
            ts1 = tf / 2
            ts2 = tf / 2
        else:
            ts1 = (-b + sqrt(sqrt_term)) / (2*a)
            ts2 = (-b - sqrt(sqrt_term)) / (2*a)
        um1 = (-a) / ((ts1 * 2) - tf)
        um2 = (-a) / ((ts2 * 2) - tf)
        #print 'bvp_fixed ts,u1, ts,u2:', ts1, um1, ts2, um2
    return ts1, um1, ts2, um2
##############################################################################################################


# bang-bang control is optimal-time solution
def solve_bvp_2d(u, pt1, pt2, dim1, dim2):   # solve 2-pt boundary value problem for 2 dimension, free final time
    #print 'u in solve_bvp_2d before multiplying by force_normalized', u
    u_m = u * Settings.force_normalized
    #print 'u_m in solve_bvp_2d:', u_m
    Q = (getattr(pt2, dim2) - getattr(pt1, dim2)) / u_m
    a = u_m
    b = 2 * getattr(pt1, dim2)
    c = getattr(pt1, dim1) - getattr(pt2, dim1) - (getattr(pt1, dim2) * Q) - (.5 * u_m * (Q ** 2))
    sqrt_term = (b ** 2) - (4 * a * c)
    if sqrt_term < 0:
        #print 'sqrt term < 0 , cannot achieve final state with this u1, u2, must reverse order'
        return None, None, None, None
    else:
        ts1 = (-b + sqrt(sqrt_term)) / (2 * a)
        tf1 = (2 * ts1) - Q
        ts2 = (-b - sqrt(sqrt_term)) / (2 * a)
        tf2 = (2 * ts2) - Q
        #print 'solve_bvp_2d ts1,2, tf1,2:', ts1, ts2, tf1, tf2
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
        #print 'bvp_fixed did not return values that make sense.'
        ts1 = None
        ts2 = None
    return ts1, ts2
##############################################################################################################


def choose_switch_time(ts1, tf1, ts2, tf2):   # selecting the switch time from the 2 calculated in bvp_2d
    #print 'possible ts1, tf1, ts2, tf2 in choose_switch_time, after solve_bvp_2d:', ts1, tf1, ts2, tf2
    if 0 < ts1 < tf1:
        ts = ts1
        tf = tf1
    elif 0 < ts2 < tf2:
        ts = ts2
        tf = tf2
    else:
        ts = None
        tf = None
        #print 'choose_switch_time returning None, None for ts, tf'
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
        #print 'choose_switch_time_fixed returning None, None for ts, u'
    return ts, u
##############################################################################################################


def call_fixed(u_x, u_y, tf_x, tf_y, ts_x, ts_y, pt1, pt2):  # call fixed-final-time 2-pt BVP for x or y
    if tf_y > tf_x:            # if y direction has larger tf
        u_y = u_y * Settings.force_normalized
        tf = tf_y              # then choose tf_y since x cannot reach goal by that time
        ts1_y = ts_y
        ts2_y = ts_y
        tsx1, ux1, tsx2, ux2 = bvp_fixed_tf(tf, pt1, pt2, 'x', 'x_vel')
        #print 'tf in call_fixed, from bvp_fixed:', tf, tsx1, ux1, tsx2, ux2
        #print 'possible tf, ts1, u1, ts2, u2, x values from fixed:', tf, tsx1, ux1, tsx2, ux2
        ts1_x, u_x = choose_switch_time_fixed(tsx1, tsx2, ux1, ux2, tf)
        #print 'u_x,y in if-else, in call_fixed, after choose_time:', u_x, u_y
        """if ts1_x is None:
            u_x = -u_x
            ts0_1x, ts0_2x, ts1_1x, ts1_2x = bvp_fixed_tf(u_x, tf, pt1, pt2, 'x', 'x_vel')
            #print 'possible tf, ts x values from fixed 2:', tf, ts0_1x, ts0_2x, ts1_1x, ts1_2x
            ts1_x, ts2_x = choose_switch_times(ts0_1x, ts0_2x, ts1_1x, ts1_2x, tf)
            #if ts1_x is None:
                #print 'bvp_fixed returned invalid solutions both u(t) options'"""
    elif tf_y < tf_x:           # if x direction has larger tf
        u_x = u_x * Settings.force_normalized
        tf = tf_x               # then choose tf_x since y cannot reach goal by that time
        ts1_x = ts_x
        ts2_x = ts_x
        tsy1, uy1, tsy2, uy2 = bvp_fixed_tf(tf, pt1, pt2, 'y', 'y_vel')
        #print 'possible tf, ts1, u1, ts2, u2, x values from fixed:', tf, tsy1, uy1, tsy2, uy2
        ts1_y, u_y = choose_switch_time_fixed(tsy1, tsy2, uy1, uy2, tf)
        #print 'u_x,y in if-else, in call_fixed, after choose_time:', u_x, u_y
        """if ts1_y is None:
            u_y = -u_y
            ts0_1y, ts0_2y, ts1_1y, ts1_2y = bvp_fixed_tf(u_y, tf, pt1, pt2, 'y', 'y_vel')
            #print 'possible tf, ts y values from fixed 2:', ts0_1y, ts0_2y, ts1_1y, ts1_2y
            ts1_y, ts2_y = choose_switch_times(ts0_1y, ts0_2y, ts1_1y, ts1_2y, tf)
            #if ts1_y is None:
                #print 'bvp_fixed returned invalid solutions for both u(t) options'"""
    else:                      # just so happened to have same exact final time calculation? weird
        tf = tf_x
        ts1_x = ts_x
        ts1_y = ts_y
        u_x = u_x * Settings.force_normalized
        u_y = u_y * Settings.force_normalized
    #print 'u_x,y in call_fixed, prior to returning', u_x, u_y
    return ts1_x, ts1_y, tf, u_x, u_y
##############################################################################################################


def call_bvp_2d(u, pt1, pt2, dim1, dim2):
    ts1, ts2, tf1, tf2 = solve_bvp_2d(u, pt1, pt2, dim1, dim2)
    if ts1 is None:                              # if first attempt had no solution
        #print 'first u attempt did not find any solution. trying reverse order'
        u = -u                                   # then reverse u order is guaranteed to have solution
        ts1, ts2, tf1, tf2 = solve_bvp_2d(u, pt1, pt2, dim1, dim2)
    if ts1 is None:
        #print 'something is wrong in bvp_2d. neither u(t) order found a solution'
        return None, None
    else:
        ts, tf = choose_switch_time(ts1, tf1, ts2, tf2)
    if ts is None:
        #print 'first u attempt found solution that doesnt make sense, trying reverse order'
        u = -u                             # then attempt u(t) reverse order
        ts1, ts2, tf1, tf2 = solve_bvp_2d(u, pt1, pt2, dim1, dim2)
        #print 'ts:', ts1, ts2
        #print 'tf', tf1, tf2
        if ts1 is None:
            #print 'two solutions from first u(t) attempt made no sense. no solution from second u(t) attempt'
            return None, None
        else:
            ts, tf = choose_switch_time(ts1, tf1, ts2, tf2)
            #if ts is None:
                #print 'neither solution from either u(t) attempt makes sense'
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
    #print 'first attempt u1_x,y:', u_x, u_y
    ts_x, tf_x, u_x = call_bvp_2d(u_x, pt1, pt2, 'x', 'x_vel')  # solve for optimal trajectory for x dimensions
    ts_y, tf_y, u_y = call_bvp_2d(u_y, pt1, pt2, 'y', 'y_vel')  # solve for optimal trajectory for y dimensions
    #print 'tf_x,y from solve_bvp_2d:', tf_x, tf_y
    #print 'ts_x,y from solve_bvp_2d:', ts_x, ts_y
    #print 'ux, uy from solve_bvp_2d:', u_x, u_y
    ts1_x, ts1_y, tf, u_x, u_y = call_fixed(u_x, u_y, tf_x, tf_y, ts_x, ts_y, pt1, pt2)    # choose 2d BVP soln that has larger tf
    #print 'tf after call_fixed:', tf
    #print 'ts_x, ux, ts_y, uy, from call_fixed:', ts1_x, u_x, ts1_y, u_y
    # bang-bang controller is saved same way as bang-off-bang with two switch times that are equal
    trajectory = Trajectory(u_x, ts1_x, u_y, ts1_y, tf)
    get_discrete_states(pt1, pt2, trajectory)
    return trajectory
##############################################################################################################


def get_discrete_states(pt1, pt2, traj):  # use switch times to sample from a continuous trajectory and save values
    #print 'getting discrete states'
    #print 'pt from:', pt1.x, pt1.y, pt1.x_vel, pt1.y_vel
    #print 'pt to:  ', pt2.x, pt2.y, pt2.x_vel, pt2.y_vel
    #print 'tf =', traj.t_f, 'ts_x, ts_y =', traj.ts1_x, traj.ts1_y
    ux = traj.u_x1
    uy = traj.u_y1
    #print 'ux, uy =', ux, uy
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
        # print 'time', time, 'pt:', x, y, v_x, v_y
    return
##############################################################################################################
