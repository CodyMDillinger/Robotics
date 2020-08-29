#!/usr/bin/env python
# -*- coding: utf-8 -*-
# This file contains functions related to measurements of all of the other code, such as time recordings

import time
##############################################################################################################


def calculate_times(times, start_time):   # given real times in seconds, subtract the start time for each
    for i in range(len(times)):
        times[i] = times[i] - start_time
    return time.time() - start_time
##############################################################################################################
