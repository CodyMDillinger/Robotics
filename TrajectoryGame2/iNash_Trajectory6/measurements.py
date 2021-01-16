#!/usr/bin/env python
# -*- coding: utf-8 -*-
# This file contains functions related to measurements of all of the other code, such as time recordings

import time
import csv
##############################################################################################################


def write_data_to_csv(data, num_robots, type, filename):    # for saving timing and cost data for various trials
    if type == 'all':
        with open(filename, 'a') as csvfile:
            writer = csv.writer(csvfile, delimiter=',') #, quotechar='', quoting=csv.QUOTE_MINIMAL)
            for i in range(len(data)):
                data_row = []
                for j in range(len(data[i])):
                    #data_row.append([data[i][j].time, data[i][j].cost])
                    data_row.append(data[i][j].time)
                writer.writerow(data_row)
            writer.writerow([])
    elif type == 'Nash':
        with open(filename, 'a') as csvfile:
            writer = csv.writer(csvfile, delimiter=',')
            writer.writerow([data, num_robots])
    return
##############################################################################################################


def calculate_times(times, start_time):      # given real times in seconds, subtract the start time for each
    for i in range(len(times)):
        for j in range(len(times[i])):
            times[i][j].time = times[i][j].time - start_time
    return time.time() - start_time
##############################################################################################################


class MeasurementData:               # for time trial data
    def __init__(self, time_, cost_):
        self.time = time_             # time when new path was chosen
        self.cost = cost_             # cost of that new path
##############################################################################################################
