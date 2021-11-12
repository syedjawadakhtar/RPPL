#! /usr/bin/env python3

# Robot Planning Python Library (RPPL)
# Copyright (c) 2021 Alexander J. LaValle. All rights reserved.
# This software is distributed under the simplified BSD license.

import pygame, time
from pygame.locals import *
from rppl_util import *

Open = True
pstat = 0
polys = []
polycount = 0
initial = 0
goal = 0

pygame.init()
screen = pygame.display.set_mode([xmax,ymax])
pygame.display.set_caption('Draw Problem 2D')
screen.fill(black)
pygame.display.update()
print('\n*CONTROLS*\nLMB = Pick point in polygon, RMB = Finish current polygon\nI = Pick initial, G = Pick goal\nSPACEBAR = Save to file\n')


def is_clockwise(polygon):
    sum = 0.0
    for i in range(len(polygon)):
        sum += (polygon[i][0] - polygon[(i+1)%len(polygon)][0]) * (polygon[i][1] + polygon[(i+1)%len(polygon)][1])
    return sum < 0


while Open:
    time.sleep(0.005)
    pygame.display.update()
    mpos = pygame.mouse.get_pos()
    if pstat == 3: pstat = 0
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
            quit()
        if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
            polygons = open('problem_polygonal.txt','w')
            polygons.write(str(polys) + '\n' + str(initial) + '\n' + str(goal))
            polygons.close()
            print('Saved to problem_polygonal.txt')
            Open = False
        if event.type == pygame.KEYDOWN and event.key == pygame.K_r:
            screen.fill(black)
            polycount = 0
            polys = []
            pstat = 0
        if event.type == pygame.KEYDOWN and event.key == pygame.K_i:
            initial = mpos
            pygame.draw.circle(screen,green,mpos,4)
        if event.type == pygame.KEYDOWN and event.key == pygame.K_g:
            goal = mpos
            pygame.draw.circle(screen,red,mpos,4)
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 3 and pstat == 1 and len(polys[polycount]) > 2:
            pygame.draw.polygon(screen,white,polys[polycount])
            if is_clockwise(polys[polycount]):
                polys[polycount].reverse()
            polycount += 1
            pstat = 3
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and pstat == 1:
            polys[polycount].insert(0,mpos)
            pygame.draw.circle(screen,white,mpos,2)
            pygame.draw.line(screen,white,mpos,polys[polycount][1])
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and pstat == 0:
            polys.append([])
            pygame.draw.circle(screen,white,mpos,2)
            polys[polycount].insert(0,mpos)
            pstat = 1