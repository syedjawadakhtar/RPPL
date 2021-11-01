#! /usr/bin/env python3

# Robot Planning Python Library (RPPL)
# Copyright (c) 2021 Alexander J. LaValle. All rights reserved.
# This software is distributed under the simplified BSD license.

import networkx as nx
from networkx.classes.function import get_node_attributes, set_node_attributes


# value iteration constants
failure_cost = 1.0E30
max_valits = 1000


def valit(graph, goal):
    # initialize values
    for n in graph.nodes:
        set_node_attributes(graph, {n:failure_cost}, 'value')
    set_node_attributes(graph, {goal:0.0}, 'value')
    print('Iteration: 0  ', get_node_attributes(G, 'value'))
    
    # main loop
    i = 0
    max_change = failure_cost
    while i < max_valits and max_change > 0.0:
        max_change = 0.0
        for m in graph.nodes:
            best_cost = failure_cost
            best_n = m
            for n in graph.neighbors(m):
                step_cost = graph.get_edge_data(m,n)['weight']
                cost = graph.nodes[n]['value'] + step_cost
                if cost < best_cost:
                    best_cost = cost
                    best_n = n
            stay_cost = graph.nodes[m]['value']
            if best_cost < stay_cost:
                if stay_cost - best_cost > max_change:
                    max_change = stay_cost - best_cost
                set_node_attributes(graph, {m:best_cost}, 'value')
                set_node_attributes(graph, {m:best_n}, 'next')
        i += 1
        print('Iteration: ' +str(i), ' ', get_node_attributes(G, 'value'))
    
# First, create the graph.
G = nx.DiGraph()
G.add_nodes_from([0, 1, 2, 3, 4])
G.add_edge(0, 0, weight=2)
G.add_edge(0, 1, weight=2)
G.add_edge(1, 2, weight=1)
G.add_edge(1, 3, weight=4)
G.add_edge(2, 0, weight=1)
G.add_edge(2, 3, weight=1)
G.add_edge(3, 2, weight=1)
G.add_edge(3, 4, weight=1)

# Compute the optimal cost to go.
valit(G, 3)