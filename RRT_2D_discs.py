#! /usr/bin/env python3

# Robot Planning Python Library (RPPL)
# Copyright (c) 2021 Alexander J. LaValle. All rights reserved.
# This software is distributed under the simplified BSD license.

import pygame, time, random
from pygame.locals import *
import networkx as nx
from math import sqrt
from rppl_util import *
from rppl_globals import *
from networkx.algorithms.shortest_paths.generic import shortest_path

#global iterations, stepsize

iterations = 2000
stepsize = 5
numobst = 20   #number of obstacles

i = 0
Open = True
pstat = 0
stepping = True
restart = True
cycle = 6


def add_next_node(rp,closest,g):
    d = dist2(rp,g.nodes[closest]['point'])
    if d == 0:
        d = 1.0E-20
    diff = stepsize / d
    newpoint = ((rp[0] - g.nodes[closest]['point'][0]) * diff + g.nodes[closest]['point'][0],(rp[1] - g.nodes[closest]['point'][1]) * diff + g.nodes[closest]['point'][1])
    return newpoint

def find_closest_node(mpos,nodes):
    a = [dist2(mpos, nodes[0]['point']),0]
    for i in nodes:
        if i > 0:
            b = [dist2(mpos, nodes[i]['point']),i]
            if a[0] > b[0]:
                a = [dist2(mpos, nodes[i]['point']),i]
    return a[1]

print('\n*CONTROLS*\nR = Re-Run\nESC = Stop all\n')

while Open:
    G = nx.Graph()
    G.add_node(0, point=(xmax/2,ymax/2))
    pygame.init()
    screen = pygame.display.set_mode([xmax,ymax])
    screen.fill(black)
    pygame.draw.circle(screen,white,G.nodes[0]['point'],5)
    obstacles = create_random_discs(numobst,G.nodes[0]['point'])
    draw_discs(obstacles,screen)
    goal = pick_random_goal(obstacles)
    pygame.draw.circle(screen,red,goal,8)
    pygame.display.update()
    restart = False
    i = 0
    j = cycle

    while dist2(G.nodes[len(G.nodes)-1]['point'],goal) > stepsize:
        i += 1
        stepping = True
        rp = [random.randint(0,xmax),random.randint(0,ymax)]
        if j == cycle:
            j = 0
            rp = goal
        j += 1
        pygame.display.set_caption('RRT, Iteration: ' +str(i))
        closest = find_closest_node(rp,G.nodes)
        G.add_node(len(G.nodes),point=add_next_node(rp,closest,G))
        if not point_inside_discs(G.nodes[len(G.nodes)-1]['point'],obstacles):
            G.add_edge(len(G.nodes)-1,closest)
            pygame.draw.line(screen,white,G.nodes[len(G.nodes)-1]['point'],G.nodes[closest]['point'],2)
            while stepping == True:
                G.add_node(len(G.nodes),point=add_next_node(rp,len(G.nodes)-1,G))
                if not point_inside_discs(G.nodes[len(G.nodes)-1]['point'],obstacles):
                    G.add_edge(len(G.nodes)-1,len(G.nodes)-2)
                    pygame.draw.line(screen,white,G.nodes[len(G.nodes)-1]['point'],G.nodes[closest]['point'],2)
                    if dist2(rp, G.nodes[len(G.nodes)-1]['point']) <= stepsize:
                        stepping = False
                else:
                    G.remove_node(len(G.nodes)-1)
                    stepping = False
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    quit()
                if event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                    stepping = False
                    restart = True
                    G.add_node(len(G.nodes),point=goal)
        else:
            G.remove_node(len(G.nodes)-1)
        pygame.display.update()

        
    if not restart:
        path = shortest_path(G, source=0, target=len(G.nodes)-1)
        for p in range(len(path)-1):
            pygame.draw.line(screen,green,G.nodes[path[p]]['point'],G.nodes[path[p+1]]['point'],3)
    while not restart:
        time.sleep(0.01)
        pygame.display.update()
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                quit()
            if event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                restart = True
