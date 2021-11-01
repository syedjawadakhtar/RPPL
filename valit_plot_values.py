#! /usr/bin/env python3

# Robot Planning Python Library (RPPL)
# Copyright (c) 2021 Alexander J. LaValle. All rights reserved.
# This software is distributed under the simplified BSD license.

from ast import literal_eval
from valit_examples import *
import matplotlib.pyplot as plt
import numpy as np

data = open('valit_data.txt')
datalines = data.readlines()
data.close()

values = literal_eval(datalines[0])
points = literal_eval(datalines[1])
dims = literal_eval(datalines[2])

fig = plt.figure()
ax = plt.axes(projection='3d')

vi = list(values.items())
pti = list(points.items())

x = np.zeros((dims,dims))
y = np.zeros((dims,dims))
z = np.zeros((dims,dims))
k = 0

for i in range(dims):
    for j in range(dims):
        x[i,j] = pti[k][1][0]
        y[i,j] = pti[k][1][1]
        z[i,j] = vi[k][1]
        k += 1
print(z)
ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap='viridis', edgecolor='none',vmin=-1,vmax=10000)
ax.set_zlim(0, 2000)
plt.show()
















'''
for i in range(len(pti)):
    xi.append(pti[i][1][0])
    yi.append(pti[i][1][1])
    zi.append(vi[i][1])


xi = []
yi = []
zi = []

ax.scatter(xi, yi, zi)
ax.set_zlim(0,2000)

plt.show()
'''