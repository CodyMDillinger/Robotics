#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Cody Dillinger
# Functions related to drawing pygame window

import pygame, math
from classes import Vertex, Colors, Color_, Dimensions, Axes, Settings
##############################################################################################################


def create_shape(pts):                           # given a set of Vertices, connect them in order
    for pt_num in range(len(pts) - 1):           # for each Vertex except the last
        pts[pt_num].obs_next = pts[pt_num + 1]   # connect the dots
    pts[len(pts) - 1].obs_next = pts[0]          # connect last Vertex back to first
    return pts
##############################################################################################################


def draw_shape(shape, pywindow):                                          # outline border of shape on pywindow
    for pt_num in range(len(shape)):                                      # for all vertices in the shape
        x1, y1 = shape[pt_num].x, shape[pt_num].y                         # point from
        x2, y2 = shape[pt_num].obs_next.x, shape[pt_num].obs_next.y       # point to
        if x1 == x2:                        # if vertical edge
            x1 = x1 - .5; x2 = x2 - .5      # show display slightly offset left
        elif y1 == y2:                      # if horizontal edge
            y1 = y1 - .5; y2 = y2 - .5      # show display slightly offset up
        pygame.draw.line(pywindow, Colors.black, (x1, y1), (x2, y2), Dimensions.line_width)   # connect edge between them
##############################################################################################################


def obstacle_generation():    # return array of obstacles, obstacle is array of Vertex objects connected in a loop
    obstacles = []
    for i in range(len(Dimensions.obs_list)):
        obstacles.append(create_shape(Dimensions.obs_list[i]))
    return obstacles
##############################################################################################################


def display_generation(x, y):
    x_ = x - Dimensions.button_width
    displays = []
    displays.append(create_shape([Vertex(0, 0, 0, 0), Vertex(0, y, 0, 0), Vertex(x, y, 0, 0), Vertex(x, 0, 0, 0)]))
    displays.append(create_shape([Vertex(0, 0, 0, 0), Vertex(0, y, 0, 0), Vertex(x_, y, 0, 0), Vertex(x_, 0, 0, 0)]))
    return displays
##############################################################################################################


def button_generation():
    buttons = []
    for i in range(len(Dimensions.button_list)):
        buttons.append(create_shape(Dimensions.button_list[i]))
    return buttons
##############################################################################################################


def label_plots(pywindow):
    x = Dimensions.window_width + 6
    y = Dimensions.window_length / 2
    y2 = Dimensions.window_length + Settings.robo_vel_max
    y3 = Dimensions.window_length + (3 * Settings.robo_vel_max)
    font = pygame.font.SysFont(None, 20)
    render = font.render('X Pos, Y Pos', True, Colors.dark_blue)
    render2 = font.render('X Pos, X-Vel', True, Colors.dark_blue)
    render3 = font.render('Y Pos, Y-Vel', True, Colors.dark_blue)
    pywindow.blit(render, (x, y))
    pywindow.blit(render2, (x, y2))
    pywindow.blit(render3, (x, y3))
    return
##############################################################################################################


def label_button(pywindow, button, i, button_text, color_, text_color):
    rect_ = pygame.Rect(Dimensions.window_width + 2, button[0].y, Dimensions.button_width, Dimensions.button_height)
    pygame.draw.rect(pywindow, color_, rect_)
    font = pygame.font.SysFont(None, 20)
    render = font.render(button_text, True, text_color)
    pywindow.blit(render, (button[1].x + 6, button[0].y + 20))
    return
##############################################################################################################


def add_colors(robo_color):                      # append colors to array to use for large numbers of robot trees
    robo_color.add_color(Color_(Colors.grey))
    robo_color.add_color(Color_(Colors.dark_red))
    robo_color.add_color(Color_(Colors.turquoise))
    robo_color.add_color(Color_(Colors.purple))
    robo_color.add_color(Color_(Colors.light_orange))
    robo_color.add_color(Color_(Colors.pink))
    robo_color.add_color(Color_(Colors.light_red))
    robo_color.add_color(Color_(Colors.light_blue))
    for i in range(len(robo_color.color_list) - 1):
        robo_color.color_list[i].next_ = robo_color.color_list[i + 1]                       # connect them in order
    robo_color.color_list[len(robo_color.color_list) - 1].next_ = robo_color.color_list[0]  # connect the last to the first
    return robo_color
##############################################################################################################


def color_init(num_robots_):                     # get array of colors same size as num of robots
    robo_colors = Color_((0, 0, 0))              # single color object, with array to be used for list of colors
    robo_colors = add_colors(robo_colors)        # add 9 colors connected in a loop
    colors = [None] * num_robots_
    color_ = robo_colors.color_list[0]
    for i in range(num_robots_):                 # for all bots
        colors[i] = color_.color_                # set color
        color_ = color_.next_
    return colors
##############################################################################################################


def world_to_x_plot(x, x_vel):     # convert world x coordinates to pixel values for
    return x, Dimensions.window_length + Dimensions.line_width + (Settings.robo_vel_max - x_vel)
##############################################################################################################


def world_to_y_plot(y, y_vel):     # convert world coordinates to pixel coordinates
    return y, Dimensions.window_length + 2 * Dimensions.line_width + 2 * Settings.robo_vel_max + (Settings.robo_vel_max - y_vel)
##############################################################################################################


def init_kd_axes():
    x = Axes('x')          # for easily scalable number of dimensions for kd-tree
    y = Axes('y')
    x_vel = Axes('x_vel')
    y_vel = Axes('y_vel')
    x.next_ = y
    y.next_ = x_vel
    x_vel.next_ = y_vel
    y_vel.next_ = x
    return x
##############################################################################################################


def draw_buttons(pywindow, buttons):
    for i in range(len(buttons)):
        draw_shape(buttons[i], pywindow)
        label_button(pywindow, buttons[i], i, Settings.button_text[i], Colors.dark_blue, Colors.white)
    return
##############################################################################################################


def draw_obstacles(pywindow, obstacles):
    for i in range(len(obstacles)):
        draw_shape(obstacles[i], pywindow)
    return
##############################################################################################################


def draw_extra(pywindow, extras):
    for i in range(len(extras)):
        draw_shape(extras[i], pywindow)
    x = Dimensions.window_length + Dimensions.line_width + Settings.robo_vel_max
    y = x + 2 * Settings.robo_vel_max + Dimensions.line_width
    xy = x + Settings.robo_vel_max
    pygame.draw.line(pywindow, Colors.black, (0, x), (Dimensions.window_width, x), 1)
    pygame.draw.line(pywindow, Colors.black, (0, y), (Dimensions.window_width, y), 1)
    pygame.draw.line(pywindow, Colors.black, (0, xy), (Dimensions.window_width, xy), Dimensions.line_width)
    return
##############################################################################################################


def draw_shapes(pywindow, buttons, obstacles, extra_displays):
    draw_buttons(pywindow, buttons)
    draw_obstacles(pywindow, obstacles)
    draw_extra(pywindow, extra_displays)
    pygame.display.flip()  # update display with these new shapes
    return
##############################################################################################################


# might not need
def re_draw_objects():   # re-draw objects
    return
##############################################################################################################


def init_pywindow(title):                                  # first function to be called from main()
    pygame.init()                                          # initialize usage of pygame
    pygame.display.set_caption(title)
    # display on pywindow three separate plots. x-y position, x.pos-x.vel, y.pos-y.vel
    x = Dimensions.window_width + Dimensions.button_width + 1
    y = Dimensions.window_length + 4 * Settings.robo_vel_max + 2 * Dimensions.line_width
    pywindow = pygame.display.set_mode((x, y))             # init size of window
    pywindow.fill(Colors.white)                            # set background of pygame window to white
    obstacles = obstacle_generation()
    buttons = button_generation()
    extra_displays = display_generation(x, y)
    label_plots(pywindow)
    draw_shapes(pywindow, buttons, obstacles, extra_displays)
    x = init_kd_axes()
    return pywindow, obstacles, x, buttons                 # kd tree will start first row comparing x
##############################################################################################################


def get_click_pos(i, pt_type, pt, pywindow, color_dot):    # prompt user to click pygame window, read position
    print 'Click robot', i, pt_type, 'Vertex'              # prompt user
    waiting = 1; exit_pressed = 0
    while waiting:                                         # loop until user has clicked, or clicked exit
        event_click = pygame.event.poll()                  # read whether user event has occurred
        if event_click.type == pygame.QUIT:                # if user clicked exit
            waiting = 0
            exit_pressed = 1
        elif event_click.type == pygame.MOUSEBUTTONDOWN and event_click.button == 1:  # if user clicked in pywindow
            x, y = pygame.mouse.get_pos()                                 # get position of the click
            x = float(x)
            y = float(y)
            pt.append(Vertex(x, y, 0, 0))                                 # array of all start/end positions
            pygame.draw.circle(pywindow, color_dot, (int(x), int(y)), Settings.robo_size, 0)    # display starting Vertex with color circle
            x_, y_ = world_to_x_plot(x, 0)
            x_ = int(x_)
            y_ = int(y_)
            pygame.draw.circle(pywindow, color_dot, (x_, y_), Settings.robo_size_vel, 0)
            x_, y_ = world_to_y_plot(y, 0)
            x_ = int(x_)
            y_ = int(y_)
            pygame.draw.circle(pywindow, color_dot, (x_, y_), Settings.robo_size_vel, 0)
            pygame.display.flip()                                         # update display with these new shapes
            waiting = 0
    return exit_pressed, pt
##############################################################################################################


def get_box_pts(x, y, delta_x, delta_y):
    pt1 = Vertex(x - delta_x, y - delta_y, 0, 0)
    pt2 = Vertex(x - delta_x, y + delta_y, 0, 0)
    pt3 = Vertex(x + delta_x, y + delta_y, 0, 0)
    pt4 = Vertex(x + delta_x, y - delta_y, 0, 0)
    return [pt1, pt2, pt3, pt4]
##############################################################################################################


def end_range(x, y, end_button):
    end = False
    if end_button[0].x <= x <= end_button[2].x and end_button[0].y <= y <= end_button[1].y:
        end = True
    return end
##############################################################################################################


def wait_for_gazebo_call(buttons):  # wait for user to click the call gazebo 3d simulator button
    waiting = True
    call_gazebo = False
    while waiting:
        event_click = pygame.event.poll()
        if event_click.type == pygame.QUIT:  # if user clicked exit
            waiting = False
        elif event_click.type == pygame.MOUSEBUTTONDOWN and event_click.button == 1:
            x, y = pygame.mouse.get_pos()
            if end_range(x, y, buttons[1]):
                call_gazebo = True
                waiting = False
    return call_gazebo
##############################################################################################################


def iterate_or_stop(pywindow, buttons, k, k_, increment, data):
    end_planning = False
    event_click = pygame.event.poll()
    if event_click.type == pygame.MOUSEBUTTONDOWN and event_click.button == 1:
        x, y = pygame.mouse.get_pos()
        if end_range(x, y, buttons[0]):
            end_planning = True
            label_button(pywindow, buttons[0], 0, 'Running 2D', Colors.dark_green, Colors.white)
            pygame.display.flip()
    if end_planning is False:      # allow path planning to end with a click
        if increment is True:
            k = k + 1
        nash_reached = True
        for i in range(len(data)):
            if data[i][0].cost == 9999:
                nash_reached = False
                break
        if nash_reached:
            k = k_
    else:
        k = k_                     # this will end the main loop
    return k
##############################################################################################################


def user_prompt(pywindow):         # prompt for num robots, start and end positions, mark on pywindow
    num_robots = input('Enter number of robots: ')
    robo_colors = color_init(num_robots)
    start = []                     # to be used for coordinates of robots start positions
    goal_set = []                  # to be used for coordinates of robots end positions
    sign = num_robots * [None]
    i = 0
    running_ = 1
    while i < num_robots and running_:                                 # until start/stop selected for all, or exited
        exit1, start = get_click_pos(i, 'start', start, pywindow, robo_colors[i])      # get start position from user, append
        start[i].tree_num = 1
        exit2, goal_set = get_click_pos(i, 'end', goal_set, pywindow, robo_colors[i])  # get end position from user, append
        goal_set[i].tree_num = 2
        if abs(goal_set[i].x - start[i].x) > 100:
            signx = True
        else:
            signx = False
        if abs(goal_set[i].y - start[i].y) > 100:
            signy = True
        else:
            signy = False
        sign[i] = [[math.copysign(1, goal_set[i].x - start[i].x), signx], [math.copysign(1, goal_set[i].y - start[i].y), signy]]
        i = i + 1
        if exit1 == 1 or exit2 == 1:                                   # end loop if user exited
            running_ = 0
    goal_set2 = []                # updating goal set to be a larger box, rather than just a goal Vertex
    for i in range(num_robots):   # make four Vertices with goal_set at center, to border continuous "goal set"
        pts1 = get_box_pts(goal_set[i].x, goal_set[i].y, Settings.goal_set_size, Settings.goal_set_size)
        x, y = world_to_x_plot(goal_set[i].x, 0)            # map x pos and x vel to pygame pixel x,y values
        x2, y2 = world_to_y_plot(goal_set[i].y, 0)          # map y pos and y vel to pygame pixel x,y values
        pts2 = get_box_pts(x, y, Settings.goal_set_size, Settings.robo_finish_vel)
        pts3 = get_box_pts(x2, y2, Settings.goal_set_size, Settings.robo_finish_vel)
        goal_box = create_shape(pts1)
        goal_vel_box = create_shape(pts2)
        goal_vel_box2 = create_shape(pts3)
        goal_set2.append(goal_box)
        draw_shape(goal_box, pywindow)
        draw_shape(goal_vel_box, pywindow)
        draw_shape(goal_vel_box2, pywindow)
    pygame.display.flip()
    return start, goal_set2, num_robots, robo_colors, sign, goal_set
##############################################################################################################
