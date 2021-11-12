#! /usr/bin/env python3

# Robot Planning Python Library (RPPL)
# Copyright (c) 2021 Alexander J. LaValle. All rights reserved.
# This software is distributed under the simplified BSD license.

import pygame, time
from pygame.locals import *
from rppl_util import *

Open = True
pstat = 0
circles = []
initial = 0
goal = 0
radius = 100
picked_initial = False
picked_goal = False
xmax = 800 # force a square environment

pygame.init()
screen = pygame.display.set_mode([xmax,ymax])
pygame.display.set_caption('Draw Circles')
screen.fill(black)
pygame.display.update()
print('\n*CONTROLS*\nLMB = Place Circle, ScrollWheel/Up&Down Arrows = Adjust size\nI = Pick initial, G = Pick goal\nSPACEBAR = Save to file\n')

while Open:
    time.sleep(0.005)
    pygame.display.update()
    mpos = pygame.mouse.get_pos()
    circle = [mpos[0],mpos[1],radius]
    screen.fill(black)
    for c in circles:
        pygame.draw.circle(screen,grey,(c[0],c[1]),c[2])
    pygame.draw.circle(screen,grey,(circle[0],circle[1]),circle[2])
    if picked_initial:
        pygame.draw.circle(screen,green,initial,5)
    if picked_goal:
        pygame.draw.circle(screen,red,goal,5)
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
            quit()
        if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
            problem = open('problem_circles.txt','a')
            problem.write(str(circles) + '\n' + str(initial) + '\n' + str(goal) + '\n')
            problem.close()
            print('Saved to problem_circles.txt')
            screen.fill(black)
            picked_initial = False
            picked_goal = False
            initial = 0
            goal = 0
            circles = []
        if event.type == pygame.KEYDOWN and event.key == pygame.K_r:
            screen.fill(black)
            picked_initial = False
            picked_goal = False
            initial = 0
            goal = 0
            circles = []
        if event.type == pygame.KEYDOWN and event.key == pygame.K_i:
            initial = mpos
            picked_initial = True
        if event.type == pygame.KEYDOWN and event.key == pygame.K_g:
            goal = mpos
            picked_goal = True
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 4 or event.type == pygame.KEYDOWN and event.key == pygame.K_UP:
            radius += 10
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 5 or event.type == pygame.KEYDOWN and event.key == pygame.K_DOWN:
            radius -= 10
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            circles.insert(0,(circle))
            pygame.draw.circle(screen,white,(circle[0],circle[1]),circle[2])