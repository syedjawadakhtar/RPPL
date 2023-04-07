#! /usr/bin/env python3

# Robot Planning Python Library (RPPL)
# Copyright (c) 2021 Alexander J. LaValle. All rights reserved.
# This software is distributed under the simplified BSD license.

import networkx as nx
from networkx.classes.function import get_node_attributes, set_node_attributes


# value iteration constants
failure_cost = 1.0E30
max_valits = 3 # For 3 step plans
cost = 0


def valit(graph, goal):
    # initialize values
    for n in graph.nodes:
        set_node_attributes(graph, {n:failure_cost}, 'value')
    set_node_attributes(graph, {goal:0.0}, 'value')
    print('Iteration: 0  ', get_node_attributes(graph, 'value'))
    
    # main loop
    i = 0
    # max_change = failure_cost
    while i < max_valits:
        update_list = []
        # max_change = 0.0
        for m in graph.nodes:
            best_cost = failure_cost
            for n in graph.neighbors(m):
                step_cost = graph.get_edge_data(m,n)['weight']
                cost = graph.nodes[n]['value'] + step_cost
                if cost < best_cost:
                    best_cost = cost
            update_list.append([m,best_cost])
        for u in update_list:
            set_node_attributes(graph, {u[0]:u[1]}, 'value')
        i += 1
        print('Iteration: ' +str(i), ' ', get_node_attributes(graph, 'value'))

# First, create the graph.
# This graph is from Homework 1 FunSTA 2 course.
G = nx.DiGraph()
G.add_nodes_from([0, 1, 2, 3, 4])
G.add_edge(0, 1, weight=2)
G.add_edge(1, 0, weight=1)
G.add_edge(1, 2, weight=4)
G.add_edge(2, 3, weight=3)
G.add_edge(2, 4, weight=7)
G.add_edge(3, 2, weight=1)
G.add_edge(3, 3, weight=1)
G.add_edge(3, 4, weight=1)

#This example is a linear bidirectional graph.
valit(G, 4)
