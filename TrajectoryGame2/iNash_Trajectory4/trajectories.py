#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Cody Dillinger
# This file contains functions related to the trajectories between two points


def get_traj_num(pt1, pt2):    # return index of trajectory in pt1.trajectories that ends at pt2
    traj_num = None
    for i in range(len(pt1.trajectories)):                              # check last point of each trajectory from pt1
        if pt1.trajectories[i].states2[len(pt1.trajectories[i].states2) - 1] == pt2:
            traj_num = i
            break
    if traj_num is None:
        print 'traj_num not returning an element number'
    return traj_num
##############################################################################################################
