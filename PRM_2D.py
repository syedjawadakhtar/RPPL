#! /usr/bin/env python3

# Robot Planning Python Library (RPPL)
# Copyright (c) 2021 Alexander J. LaValle. All rights reserved.
# This software is distributed under the simplified BSD license.

import pygame, time, random
from pygame.locals import *
import networkx as nx
from ast import literal_eval
from rppl_util import *
from rppl_globals import *
from polygon_triangulate import polygon_triangulate

numobst = 20    #number of obstacles
num_nodes = 200 #number of nodes
radius = 180    #maximum edge length

Open = True
pstat = 0
restart = True


problem = open('problem_polygonal.txt')
problines = problem.readlines()
problem.close()
obstacles = literal_eval(problines[0])
initial = literal_eval(problines[1])
goal = literal_eval(problines[2])
tilist = []
tlist = []

for o in obstacles:
    tilist.append(polygon_triangulate(len(o), [i[0] for i in o], [j[1] for j in o]))

# convert triangulation output to list of triangles
for o in range(len(obstacles)):
        for t in tilist[o]:
            tlist.append([obstacles[o][t[0]],obstacles[o][t[1]],obstacles[o][t[2]]])

print('\n*CONTROLS*\nR = Re-Run\nESC = Stop all\n')

while Open:
    G = nx.Graph()
    pygame.init()
    screen = pygame.display.set_mode([xmax,ymax])
    pygame.display.set_caption('Probabilistic Roadmap')
    screen.fill(black)
    pygame.display.update()
    restart = False
    nodesdone = False
    G.add_node(0, point=(initial))
    G.add_node(1, point=(goal))

    while len(G.nodes) < num_nodes:
        G.add_node(len(G.nodes), point=(random.uniform(0,xmax),random.uniform(0,ymax)))
        if point_in_triangles(G.nodes[len(G.nodes)-1]['point'],tlist):
            G.remove_node(len(G.nodes)-1)
    
    for j in range(len(G.nodes)):
        for k in range(len(G.nodes)):
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    quit()
            if j != k:
                if dist2(G.nodes[j]['point'],G.nodes[k]['point']) <= radius:
                    add_edge = True
                    for poly in obstacles:
                        if add_edge:
                            for point in range(len(poly)):
                                if lines_intersecting((G.nodes[j]['point'],G.nodes[k]['point']), (poly[point],poly[point-1])):
                                    add_edge = False
                                    break
                        else:
                            break
                    if add_edge:
                        G.add_edge(j,k)

    draw_polys(tlist, screen)
    draw_graph_edges(G, screen)
    
    if nx.has_path(G,0,1):
        path = nx.dijkstra_path(G,0,1)
        for l in range(len(path)):
            if l > 0:
                pygame.draw.line(screen,green,G.nodes[path[l]]['point'],G.nodes[path[l-1]]['point'],5)
    pygame.draw.circle(screen,green,initial,10)
    pygame.draw.circle(screen,red,goal,10)
    pygame.display.update()

    while not restart:
        time.sleep(0.01)
        pygame.display.update()
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                quit()
            if event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                restart = True